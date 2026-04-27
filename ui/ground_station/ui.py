"""UT-CORE ground station UI. PyQt5 + pyqtgraph.

Tabs: Node Health, Telemetry Plots, Frame Log.
Run: python ui.py
"""

import queue
import sys
import time
from collections import defaultdict, deque

import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

import can_proto
from csv_logger import CsvLogger
from udp_receiver import Frame, UdpReceiver

UDP_PORT = 5005
HEARTBEAT_TIMEOUT_S = 5.0
PLOT_WINDOW_S = 60.0
FRAME_LOG_MAX = 500
UI_TICK_MS = 100

# Nodes to show in the health grid (order matters for layout)
HEALTH_NODES = [
    can_proto.CDH_ID, can_proto.EPS_ID, can_proto.COMMS_ID, can_proto.ADCS_ID,
    can_proto.MOTOR_ID, can_proto.GNSS_ID, can_proto.SOLAR_ID,
]

CLASS_ROW_COLOR = {
    can_proto.CLS_HEARTBEAT: QtGui.QColor(220, 240, 255),
    can_proto.CLS_COMMAND:   QtGui.QColor(255, 235, 205),
    can_proto.CLS_CMD_RESP:  QtGui.QColor(255, 245, 225),
    can_proto.CLS_TELEMETRY: QtGui.QColor(225, 255, 225),
    can_proto.CLS_HEALTH:    QtGui.QColor(255, 225, 225),
}


# ---------------------------------------------------------------------------
# Node Health tab
# ---------------------------------------------------------------------------
class NodeHealthTab(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self._boxes: dict[int, QtWidgets.QFrame] = {}
        self._labels: dict[int, QtWidgets.QLabel] = {}
        self._last_seen: dict[int, float] = {}
        self._count: dict[int, int] = defaultdict(int)

        grid = QtWidgets.QGridLayout(self)
        grid.setSpacing(12)
        for i, node_id in enumerate(HEALTH_NODES):
            box = QtWidgets.QFrame()
            box.setFrameShape(QtWidgets.QFrame.Box)
            box.setLineWidth(2)
            box.setMinimumSize(180, 110)
            vbox = QtWidgets.QVBoxLayout(box)
            title = QtWidgets.QLabel(can_proto.name_node(node_id))
            title.setAlignment(QtCore.Qt.AlignCenter)
            font = title.font(); font.setPointSize(14); font.setBold(True)
            title.setFont(font)
            status = QtWidgets.QLabel("never seen\n0 frames")
            status.setAlignment(QtCore.Qt.AlignCenter)
            vbox.addWidget(title)
            vbox.addWidget(status)
            grid.addWidget(box, i // 4, i % 4)
            self._boxes[node_id] = box
            self._labels[node_id] = status
            self._set_color(box, "grey")

    @staticmethod
    def _set_color(box: QtWidgets.QFrame, color: str) -> None:
        palette = {"green": "#b7e1b7", "red": "#e8b0b0", "grey": "#d0d0d0"}
        box.setStyleSheet(
            f"QFrame {{ background-color: {palette[color]}; border: 2px solid #555; }}"
        )

    def on_frame(self, f: Frame) -> None:
        if f.cls != can_proto.CLS_HEARTBEAT or f.opcode != can_proto.OP_HEARTBEAT:
            return
        # Prefer payload source for heartbeat, but fall back to CAN-ID source.
        # Some bridge-generated heartbeats may set one source field differently.
        node_id = f.payload_src if f.payload_src in self._boxes else f.src
        if node_id not in self._boxes:
            return
        self._last_seen[node_id] = f.rx_wall_time
        self._count[node_id] += 1

    def refresh(self) -> None:
        now = time.time()
        for node_id, box in self._boxes.items():
            last = self._last_seen.get(node_id)
            count = self._count.get(node_id, 0)
            if last is None:
                self._set_color(box, "grey")
                self._labels[node_id].setText("never seen\n0 frames")
                continue
            age = now - last
            color = "green" if age < HEARTBEAT_TIMEOUT_S else "red"
            self._set_color(box, color)
            self._labels[node_id].setText(
                f"last: {age:0.1f} s ago\n{count} frames"
            )


# ---------------------------------------------------------------------------
# Telemetry Plots tab
# ---------------------------------------------------------------------------
class TelemetryTab(QtWidgets.QWidget):
    BYTE_LABELS = ["p2", "p3", "p4", "p5", "p6", "p7"]

    def __init__(self):
        super().__init__()

        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Node:"))
        self.node_combo = QtWidgets.QComboBox()
        self.node_combo.addItem("ANY", None)
        for nid in HEALTH_NODES:
            self.node_combo.addItem(can_proto.name_node(nid), nid)
        top.addWidget(self.node_combo)

        top.addWidget(QtWidgets.QLabel("Opcode:"))
        self.op_combo = QtWidgets.QComboBox()
        self.op_combo.addItem("ANY", None)
        for op, nm in can_proto.OPCODE_NAMES.items():
            self.op_combo.addItem(nm, op)
        top.addWidget(self.op_combo)
        top.addStretch()

        # Six stacked plots, one per payload byte.
        self._plots: list[pg.PlotWidget] = []
        self._curves: list[pg.PlotDataItem] = []
        self._x: list[deque] = []
        self._y: list[deque] = []
        plot_area = pg.GraphicsLayoutWidget()
        colors = ["r", "g", "b", "c", "m", "y"]
        for i, label in enumerate(self.BYTE_LABELS):
            p = plot_area.addPlot(row=i, col=0)
            p.setLabel("left", label)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setYRange(0, 255)
            curve = p.plot(pen=pg.mkPen(colors[i], width=1.5))
            self._plots.append(p)
            self._curves.append(curve)
            self._x.append(deque())
            self._y.append(deque())
        self._plots[-1].setLabel("bottom", "time (s)")

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(plot_area)

        self._t0 = time.time()
        self._need_redraw = False

    def on_frame(self, f: Frame) -> None:
        node_filter = self.node_combo.currentData()
        op_filter = self.op_combo.currentData()
        if node_filter is not None and f.src != node_filter:
            return
        if op_filter is not None and f.opcode != op_filter:
            return

        t = f.rx_wall_time - self._t0
        values = (f.p2, f.p3, f.p4, f.p5, f.p6, f.p7)
        cutoff = t - PLOT_WINDOW_S
        for i, v in enumerate(values):
            xs, ys = self._x[i], self._y[i]
            xs.append(t); ys.append(v)
            while xs and xs[0] < cutoff:
                xs.popleft(); ys.popleft()
        self._need_redraw = True

    def refresh(self) -> None:
        if not self._need_redraw:
            return
        for curve, xs, ys in zip(self._curves, self._x, self._y):
            curve.setData(list(xs), list(ys))
        self._need_redraw = False


# ---------------------------------------------------------------------------
# Motor Speeds tab
# ---------------------------------------------------------------------------
class MotorSpeedsTab(QtWidgets.QWidget):
    RPM_MAX = 12000

    def __init__(self):
        super().__init__()

        self._rpm_labels: dict[int, QtWidgets.QLabel] = {}
        self._age_labels: dict[int, QtWidgets.QLabel] = {}
        self._bars: dict[int, QtWidgets.QProgressBar] = {}
        self._last_seen: dict[int, float] = {}
        self._rpm: dict[int, int] = {}

        title = QtWidgets.QLabel("Live MOTOR telemetry (OP_SET_WHEEL_RPM)")
        title_font = title.font()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)

        table = QtWidgets.QGridLayout()
        table.addWidget(QtWidgets.QLabel("Motor"), 0, 0)
        table.addWidget(QtWidgets.QLabel("Speed (rpm)"), 0, 1)
        table.addWidget(QtWidgets.QLabel("RPM Bar (-12000 .. +12000)"), 0, 2)
        table.addWidget(QtWidgets.QLabel("Last update"), 0, 3)

        for row, motor_idx in enumerate(range(1, 5), start=1):
            motor_name = QtWidgets.QLabel(f"M{motor_idx}")
            motor_name.setAlignment(QtCore.Qt.AlignCenter)

            rpm_val = QtWidgets.QLabel("--")
            rpm_font = rpm_val.font()
            rpm_font.setPointSize(20)
            rpm_font.setBold(True)
            rpm_val.setFont(rpm_font)
            rpm_val.setAlignment(QtCore.Qt.AlignCenter)

            bar = QtWidgets.QProgressBar()
            bar.setRange(-self.RPM_MAX, self.RPM_MAX)
            bar.setValue(0)
            bar.setFormat("%v rpm")
            bar.setAlignment(QtCore.Qt.AlignCenter)
            bar.setMinimumWidth(420)

            age_val = QtWidgets.QLabel("never")
            age_val.setAlignment(QtCore.Qt.AlignCenter)

            table.addWidget(motor_name, row, 0)
            table.addWidget(rpm_val, row, 1)
            table.addWidget(bar, row, 2)
            table.addWidget(age_val, row, 3)

            self._rpm_labels[motor_idx] = rpm_val
            self._age_labels[motor_idx] = age_val
            self._bars[motor_idx] = bar

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(title)
        layout.addLayout(table)
        layout.addStretch()

    @staticmethod
    def _decode_signed_i16(msb: int, lsb: int) -> int:
        value = ((msb & 0xFF) << 8) | (lsb & 0xFF)
        if value & 0x8000:
            value -= 0x10000
        return value

    @staticmethod
    def _bar_chunk_color_for_rpm(rpm: int, rpm_max: int) -> str:
        # Color maps magnitude only: green (slow) -> orange -> red (fast).
        ratio = min(abs(rpm) / float(rpm_max), 1.0)
        if ratio <= 0.5:
            # green(76,175,80) to orange(255,152,0)
            t = ratio / 0.5
            r = int(76 + (255 - 76) * t)
            g = int(175 + (152 - 175) * t)
            b = int(80 + (0 - 80) * t)
        else:
            # orange(255,152,0) to red(229,57,53)
            t = (ratio - 0.5) / 0.5
            r = int(255 + (229 - 255) * t)
            g = int(152 + (57 - 152) * t)
            b = int(0 + (53 - 0) * t)
        return f"rgb({r}, {g}, {b})"

    def _apply_bar_style(self, motor_idx: int, rpm: int) -> None:
        color = self._bar_chunk_color_for_rpm(rpm, self.RPM_MAX)
        bar = self._bars[motor_idx]
        bar.setStyleSheet(
            "QProgressBar {"
            " border: 1px solid #777;"
            " border-radius: 4px;"
            " text-align: center;"
            " background-color: #f2f2f2;"
            "}"
            f"QProgressBar::chunk {{ background-color: {color}; }}"
        )

    def on_frame(self, f: Frame) -> None:
        if f.src != can_proto.MOTOR_ID:
            return
        if f.cls != can_proto.CLS_TELEMETRY or f.opcode != can_proto.OP_SET_WHEEL_RPM:
            return

        motor_idx = int(f.p2)
        if motor_idx not in self._rpm_labels:
            return

        rpm = self._decode_signed_i16(f.p3, f.p4)
        self._rpm[motor_idx] = rpm
        self._last_seen[motor_idx] = f.rx_wall_time

    def refresh(self) -> None:
        now = time.time()
        for motor_idx in self._rpm_labels:
            rpm = self._rpm.get(motor_idx)
            last = self._last_seen.get(motor_idx)
            self._rpm_labels[motor_idx].setText("--" if rpm is None else f"{rpm}")
            bar_rpm = 0 if rpm is None else max(-self.RPM_MAX, min(self.RPM_MAX, rpm))
            self._bars[motor_idx].setValue(bar_rpm)
            self._apply_bar_style(motor_idx, bar_rpm)
            if last is None:
                self._age_labels[motor_idx].setText("never")
            else:
                self._age_labels[motor_idx].setText(f"{now - last:0.1f} s ago")


# ---------------------------------------------------------------------------
# CDH Temperature tab
# ---------------------------------------------------------------------------
class TemperatureTab(QtWidgets.QWidget):
    TEMP_ADDRESSES = [0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D]
    ERROR_SENTINEL = -128
    COLD_C = 15.0
    HOT_C = 40.0

    def __init__(self):
        super().__init__()
        self._cards: dict[int, QtWidgets.QFrame] = {}
        self._value_labels: dict[int, QtWidgets.QLabel] = {}
        self._age_labels: dict[int, QtWidgets.QLabel] = {}
        self._last_seen: dict[int, float] = {}
        self._temp_c: dict[int, int | None] = {}

        title = QtWidgets.QLabel("Live CDH board temperatures")
        title_font = title.font()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)

        subtitle = QtWidgets.QLabel("Blue <= 15 C   ->   Red >= 40 C")
        subtitle.setAlignment(QtCore.Qt.AlignCenter)

        grid = QtWidgets.QGridLayout()
        grid.setSpacing(12)

        for idx, addr in enumerate(self.TEMP_ADDRESSES):
            card = QtWidgets.QFrame()
            card.setFrameShape(QtWidgets.QFrame.Box)
            card.setLineWidth(2)
            card.setMinimumSize(240, 140)
            card_layout = QtWidgets.QVBoxLayout(card)

            sensor_name = QtWidgets.QLabel(f"T{idx + 1}  (0x{addr:02X})")
            sensor_font = sensor_name.font()
            sensor_font.setPointSize(13)
            sensor_font.setBold(True)
            sensor_name.setFont(sensor_font)
            sensor_name.setAlignment(QtCore.Qt.AlignCenter)

            temp_label = QtWidgets.QLabel("-- C")
            temp_label.setAlignment(QtCore.Qt.AlignCenter)
            temp_font = temp_label.font()
            temp_font.setPointSize(22)
            temp_font.setBold(True)
            temp_label.setFont(temp_font)

            age_label = QtWidgets.QLabel("never")
            age_label.setAlignment(QtCore.Qt.AlignCenter)
            age_font = age_label.font()
            age_font.setPointSize(11)
            age_label.setFont(age_font)

            card_layout.addWidget(sensor_name)
            card_layout.addWidget(temp_label)
            card_layout.addWidget(age_label)

            row = idx // 3
            col = idx % 3
            grid.addWidget(card, row, col)

            self._cards[addr] = card
            self._value_labels[addr] = temp_label
            self._age_labels[addr] = age_label
            self._set_card_style(card, "#d0d0d0")

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addLayout(grid)
        layout.addStretch()

    @staticmethod
    def _to_signed(byte_val: int) -> int:
        return byte_val - 256 if byte_val >= 128 else byte_val

    @staticmethod
    def _set_card_style(card: QtWidgets.QFrame, bg_color: str) -> None:
        card.setStyleSheet(
            "QFrame {"
            f" background-color: {bg_color};"
            " border: 2px solid #555;"
            " border-radius: 8px;"
            "}"
        )

    @classmethod
    def _temp_to_color(cls, temp_c: int | None) -> str:
        if temp_c is None:
            return "#b5b5b5"
        if temp_c <= cls.COLD_C:
            t = 0.0
        elif temp_c >= cls.HOT_C:
            t = 1.0
        else:
            t = (temp_c - cls.COLD_C) / (cls.HOT_C - cls.COLD_C)

        # Blue (cold) -> Red (hot)
        r = int(60 + (230 - 60) * t)
        g = int(150 + (70 - 150) * t)
        b = int(235 + (60 - 235) * t)
        return f"rgb({r}, {g}, {b})"

    def on_frame(self, f: Frame) -> None:
        if f.src != can_proto.CDH_ID:
            return
        if f.cls != can_proto.CLS_TELEMETRY or f.opcode != can_proto.OP_CDH_TEMP_TELEM:
            return

        values = [f.p2, f.p3, f.p4, f.p5, f.p6, f.p7]
        now = f.rx_wall_time
        for addr, raw in zip(self.TEMP_ADDRESSES, values):
            temp_c = self._to_signed(raw)
            self._temp_c[addr] = None if temp_c == self.ERROR_SENTINEL else temp_c
            self._last_seen[addr] = now

    def refresh(self) -> None:
        now = time.time()
        for addr in self.TEMP_ADDRESSES:
            temp_c = self._temp_c.get(addr)
            last = self._last_seen.get(addr)

            self._value_labels[addr].setText("ERR" if temp_c is None else f"{temp_c} C")
            self._set_card_style(self._cards[addr], self._temp_to_color(temp_c))
            if last is None:
                self._age_labels[addr].setText("never")
            else:
                self._age_labels[addr].setText(f"{now - last:0.1f} s ago")


# ---------------------------------------------------------------------------
# Frame Log tab
# ---------------------------------------------------------------------------
class FrameLogTab(QtWidgets.QWidget):
    COLUMNS = ["Time", "SRC", "DST", "Class", "Opcode",
               "p2", "p3", "p4", "p5", "p6", "p7"]

    def __init__(self):
        super().__init__()
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Filter:"))
        self.filter_edit = QtWidgets.QLineEdit()
        self.filter_edit.setPlaceholderText("substring (e.g. 'EPS', 'TELEMETRY', 'PING')")
        top.addWidget(self.filter_edit)

        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        top.addWidget(self.pause_btn)

        self.table = QtWidgets.QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(self.table)

        self._pending: deque[Frame] = deque()

    def on_frame(self, f: Frame) -> None:
        if self.pause_btn.isChecked():
            return
        self._pending.append(f)

    def refresh(self) -> None:
        if not self._pending:
            return
        needle = self.filter_edit.text().strip().upper()

        drained: list[Frame] = []
        while self._pending:
            drained.append(self._pending.popleft())

        for f in drained:
            src = can_proto.name_node(f.src)
            dst = can_proto.name_node(f.dst)
            cls_name = can_proto.name_class(f.cls)
            op_name = can_proto.name_opcode(f.opcode, f.cls)
            if needle:
                hay = f"{src} {dst} {cls_name} {op_name}".upper()
                if needle not in hay:
                    continue

            row = self.table.rowCount()
            self.table.insertRow(row)
            t_str = time.strftime("%H:%M:%S", time.localtime(f.rx_wall_time))
            t_str += f".{int((f.rx_wall_time % 1) * 1000):03d}"
            values = [t_str, src, dst, cls_name, op_name,
                      f.p2, f.p3, f.p4, f.p5, f.p6, f.p7]
            color = CLASS_ROW_COLOR.get(f.cls)
            for c, v in enumerate(values):
                item = QtWidgets.QTableWidgetItem(str(v))
                if color is not None:
                    item.setBackground(color)
                self.table.setItem(row, c, item)

        # Trim old rows.
        while self.table.rowCount() > FRAME_LOG_MAX:
            self.table.removeRow(0)
        self.table.scrollToBottom()


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"UT-CORE Ground Station  —  UDP :{UDP_PORT}")
        self.resize(1100, 700)

        self.health_tab = NodeHealthTab()
        self.plots_tab = TelemetryTab()
        self.motor_tab = MotorSpeedsTab()
        self.temp_tab = TemperatureTab()
        self.log_tab = FrameLogTab()

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self.health_tab, "Node Health")
        tabs.addTab(self.plots_tab,  "Telemetry Plots")
        tabs.addTab(self.motor_tab,  "Motor Speeds")
        tabs.addTab(self.temp_tab,   "Temp")
        tabs.addTab(self.log_tab,    "Frame Log")
        self.setCentralWidget(tabs)

        self.status = self.statusBar()
        self._rx_count = 0
        self._rx_count_last = 0
        self._last_rate_tick = time.time()

        # Frame handoff from receiver thread -> UI thread.
        self._queue: queue.Queue[Frame] = queue.Queue(maxsize=10000)
        self.logger = CsvLogger(log_dir="logs")
        self.rx = UdpReceiver(port=UDP_PORT)
        self.rx.add_callback(self._enqueue)
        self.rx.start()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(UI_TICK_MS)

    def _enqueue(self, f: Frame) -> None:
        try:
            self._queue.put_nowait(f)
        except queue.Full:
            pass  # drop on overflow

    def _tick(self) -> None:
        # Drain receiver queue in the UI thread.
        drained = 0
        while True:
            try:
                f = self._queue.get_nowait()
            except queue.Empty:
                break
            drained += 1
            self.logger.on_frame(f)
            self.health_tab.on_frame(f)
            self.plots_tab.on_frame(f)
            self.motor_tab.on_frame(f)
            self.temp_tab.on_frame(f)
            self.log_tab.on_frame(f)

        self._rx_count += drained

        self.health_tab.refresh()
        self.plots_tab.refresh()
        self.motor_tab.refresh()
        self.temp_tab.refresh()
        self.log_tab.refresh()

        now = time.time()
        dt = now - self._last_rate_tick
        if dt >= 1.0:
            rate = (self._rx_count - self._rx_count_last) / dt
            self._rx_count_last = self._rx_count
            self._last_rate_tick = now
            self.status.showMessage(
                f"Total frames: {self._rx_count}    Rate: {rate:6.1f} fps"
            )

    def closeEvent(self, ev: QtGui.QCloseEvent) -> None:
        self.rx.stop()
        self.logger.close()
        super().closeEvent(ev)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, background="w", foreground="k")
    win = MainWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
