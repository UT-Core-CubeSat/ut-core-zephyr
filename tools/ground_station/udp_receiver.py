"""UDP listener for M5 Stamp S3 CAN bridge. Decodes 21-byte packets."""

import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

import can_proto

UDP_PACKET_FMT = "<QIB8s"                     # timestamp_ms, can_id, dlc, data
UDP_PACKET_SIZE = struct.calcsize(UDP_PACKET_FMT)  # 21


@dataclass
class Frame:
    # Raw fields from the M5
    timestamp_ms: int      # M5's millis() at capture time
    rx_wall_time: float    # laptop time.time() on receipt
    can_id: int
    dlc: int
    data: bytes

    # Decoded CAN ID fields
    priority: int
    src: int
    dst: int
    op: int

    # Payload: p0..p7 = data[0..7]; None for index >= dlc
    p0: Optional[int]
    p1: Optional[int]
    p2: Optional[int]
    p3: Optional[int]
    p4: Optional[int]
    p5: Optional[int]
    p6: Optional[int]
    p7: Optional[int]

    @property
    def payload(self) -> bytes:
        """Only the first dlc bytes of data."""
        return self.data[:min(self.dlc, 8)]

    @property
    def len_ok(self) -> bool:
        """True if dlc matches the fixed length for this opcode."""
        return self.dlc == can_proto.op_len(self.op)

    @classmethod
    def from_bytes(cls, pkt: bytes, rx_wall_time: float) -> "Frame":
        ts, can_id, dlc, data = struct.unpack(UDP_PACKET_FMT, pkt)
        dec = can_proto.decode_id(can_id)
        n = min(dlc, 8)
        d = [data[i] if i < n else None for i in range(8)]
        return cls(
            timestamp_ms=ts, rx_wall_time=rx_wall_time,
            can_id=can_id, dlc=dlc, data=data,
            priority=dec["priority"], src=dec["src"],
            dst=dec["dst"], op=dec["op"],
            p0=d[0], p1=d[1], p2=d[2], p3=d[3], p4=d[4], p5=d[5], p6=d[6], p7=d[7],
        )


class UdpReceiver:
    """Background-threaded UDP listener. Fans frames out to callbacks."""

    def __init__(self, host: str = "0.0.0.0", port: int = 5005):
        self.host = host
        self.port = port
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._callbacks: list = []

    def add_callback(self, cb) -> None:
        """cb(frame: Frame) -- called from receiver thread."""
        self._callbacks.append(cb)

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.5)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                pkt, _addr = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(pkt) != UDP_PACKET_SIZE:
                continue
            frame = Frame.from_bytes(pkt, time.time())
            for cb in self._callbacks:
                try:
                    cb(frame)
                except Exception as e:  # don't kill the receiver thread
                    print(f"callback error: {e}")


if __name__ == "__main__":
    # Quick smoke test: print every frame received.
    rx = UdpReceiver()
    rx.add_callback(lambda f: print(
        f"[{f.timestamp_ms:>10} ms] "
        f"{can_proto.name_node(f.src):>9} -> {can_proto.name_node(f.dst):<9} "
        f"{can_proto.name_category(f.op):<9} "
        f"op=0x{f.op:03X} {can_proto.name_opcode(f.op):<18} "
        f"dlc={f.dlc}{'' if f.len_ok else '!'} "
        f"p={' '.join(f'{b:3}' for b in f.payload)}"
    ))
    rx.start()
    print(f"Listening on UDP :5005 ... Ctrl+C to quit")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        rx.stop()
