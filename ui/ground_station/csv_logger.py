"""Per-source-node CSV logger. Opens one file per node on first sight."""

import csv
import os
import threading
from datetime import datetime

import can_proto
from udp_receiver import Frame

CSV_HEADER = [
    "rx_wall_time", "timestamp_ms",
    "priority", "src", "src_name", "dst", "dst_name",
    "cls", "cls_name", "opcode", "opcode_name",
    "p2", "p3", "p4", "p5", "p6", "p7",
    "can_id_hex",
]


class CsvLogger:
    """Writes one CSV per source node into `log_dir/<NODE>_YYYY-MM-DD.csv`."""

    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        self._date = datetime.now().strftime("%Y-%m-%d")
        self._files: dict[int, tuple] = {}   # src -> (file handle, csv writer)
        self._lock = threading.Lock()

    def _writer_for(self, src: int):
        if src in self._files:
            return self._files[src][1]
        name = can_proto.name_node(src).replace("/", "_")
        path = os.path.join(self.log_dir, f"{name}_{self._date}.csv")
        new_file = not os.path.exists(path)
        fh = open(path, "a", newline="", encoding="utf-8")
        w = csv.writer(fh)
        if new_file:
            w.writerow(CSV_HEADER)
            fh.flush()
        self._files[src] = (fh, w)
        return w

    def on_frame(self, f: Frame) -> None:
        with self._lock:
            w = self._writer_for(f.src)
            w.writerow([
                f"{f.rx_wall_time:.3f}", f.timestamp_ms,
                f.priority, f.src, can_proto.name_node(f.src),
                f.dst, can_proto.name_node(f.dst),
                f.cls, can_proto.name_class(f.cls),
                f.opcode, can_proto.name_opcode(f.opcode),
                f.p2, f.p3, f.p4, f.p5, f.p6, f.p7,
                f"0x{f.can_id:08X}",
            ])
            self._files[f.src][0].flush()

    def close(self) -> None:
        with self._lock:
            for fh, _ in self._files.values():
                fh.close()
            self._files.clear()
