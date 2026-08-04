# mrj4_client/transport_serial.py

from __future__ import annotations
from typing import Optional
import time, logging
from .mrj4_client import ByteStr, MRJ4Error

logger = logging.getLogger("mrj4.serial")
logger.setLevel(logging.DEBUG)

# console_handler = logging.StreamHandler()
# console_handler.setLevel(logging.INFO)
# console_handler.setFormatter(logging.Formatter("[%(name)s] %(levelname)s: %(message)s"))

fh = logging.FileHandler("mrj4_serial_log.csv", mode="a", encoding="utf-8")
fh.setLevel(logging.DEBUG)

formatter = logging.Formatter("%(message)s")
fh.setFormatter(formatter)
logger.addHandler(fh)

class SerialTransport:
    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 0.05) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: Optional["serial.Serial"] = None  # type: ignore

    def _log_io(self, direction: str, data: ByteStr) -> None:
        """IO 로그 기록 (epoch ms 단위)"""
        ts = f"{time.time():.3f}"  # 초 단위 + 밀리초
        logger.info(f"[{ts}] {direction.upper()} {len(data)}B: {data.hex(' ').upper()}")

    def open(self) -> None:
        try:
            import serial
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,   # 데이터 8비트
                parity=serial.PARITY_EVEN,   # 짝수 패리티
                stopbits=serial.STOPBITS_ONE,# 1 스톱비트
                write_timeout=self.timeout,
            )
            logger.info(f"Serial opened: {self.port} @ {self.baudrate}")
        except Exception as e:
            raise MRJ4Error(f"Serial open failed: {e}")

    def close(self) -> None:
        if self._ser:
            try:
                self._ser.close()
                logger.info("Serial closed")
            finally:
                self._ser = None

    def write(self, data: ByteStr) -> int:
        if not self._ser:
            raise MRJ4Error("Serial not open")
        try:
            written = self._ser.write(data)
            self._log_io("write", data)
            return written
        except Exception as e:
            raise MRJ4Error(f"Serial write failed: {e}")

    def read(self, n: int) -> ByteStr:
        if not self._ser:
            raise MRJ4Error("Serial not open")
        try:      
            data = self._ser.read(n)
            if data:
                self._log_io("read", data)
            return data
        except Exception as e:
            raise MRJ4Error(f"Serial read failed: {e}")

    def in_waiting(self) -> int:
        if not self._ser:
            return 0
        try:
            return self._ser.in_waiting
        except Exception as e:
            raise MRJ4Error(f"Serial in_waiting failed: {e}")
