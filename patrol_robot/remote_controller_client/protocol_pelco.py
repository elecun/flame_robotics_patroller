from __future__ import annotations
from typing import Optional, List
import logging 

logger = logging.getLogger("rc.protocol")
logger.setLevel(logging.ERROR)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.DEBUG)
console_handler.setFormatter(logging.Formatter("[%(name)s] %(levelname)s: %(message)s"))

if not logger.hasHandlers():  # 중복 방지
    logger.addHandler(console_handler)

class RCResponse:
    """
    RemoteController 응답 패킷 파싱 헬퍼 (Pelco-D 규격).
    payload 전체(bytes)를 받아서 addr/cmd1/cmd2/data/checksum 필드 분리.
    """
    __slots__ = ("raw", "addr", "cmd1", "cmd2", "data1", "data2", "checksum")

    def __init__(self, payload: bytes):
        self.raw = payload
        if len(payload) != 7 or payload[0] != 0xFF:
            raise ValueError(f"Invalid RemoteController frame: {payload.hex(' ')}")

        _, addr, c1, c2, d1, d2, csum = payload
        calc = (addr + c1 + c2 + d1 + d2) & 0xFF
        if csum != calc:
            raise ValueError(
                f"Checksum mismatch in RemoteController frame: "
                f"got=0x{csum:02X}, calc=0x{calc:02X}, frame={payload.hex(' ')}"
            )

        self.addr = addr
        self.cmd1 = c1
        self.cmd2 = c2
        self.data1 = d1
        self.data2 = d2
        self.checksum = csum

    def data_str(self) -> str:
        """data1, data2를 사람이 읽기 쉽게 반환"""
        return f"d1=0x{self.data1:02X} d2=0x{self.data2:02X}"

    def __repr__(self):
        return (f"<RCResponse addr=0x{self.addr:02X} "
                f"cmd1=0x{self.cmd1:02X} cmd2=0x{self.cmd2:02X} "
                f"data1=0x{self.data1:02X} data2=0x{self.data2:02X} "
                f"checksum=0x{self.checksum:02X}>")

class StreamParser:
    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> List[RCResponse]:
        out: List[RCResponse] = []
        if not chunk:
            return out
        self._buf.extend(chunk)
        logger.debug(f"feed: got {len(chunk)}B, buffer={self._buf.hex(' ')}")

        while True:
            # 헤더(0xFF) 찾기
            try:
                hdr = self._buf.index(0xFF)
            except ValueError:
                self._buf.clear()
                break
            if hdr > 0:
                del self._buf[:hdr]

            if len(self._buf) < 7:
                break  # 아직 프레임 미완성

            frame = bytes(self._buf[:7])

            try:
                resp = RCResponse(frame)
                out.append(resp)
                logger.debug(f"complete frame={frame.hex(' ')}")
            except Exception as e:
                logger.debug(f"failed to parse frame: {e}")
                # 한 바이트 앞으로 밀고 재동기화
                del self._buf[0]
                continue

            # 처리 완료된 프레임 버퍼에서 제거
            del self._buf[:7]

        return out


class RCPacketBuilder:
    """
    Pelco-D 프레임 빌더

    프레임 형식:
      0xFF | addr(1B) | cmd1(1B) | cmd2(1B) | data1(1B) | data2(1B) | checksum(1B)

    * checksum = (addr + cmd1 + cmd2 + data1 + data2) & 0xFF
    * 길이는 항상 7바이트
    """

    def build(
        self,
        *,
        addr: int,
        cmd1: int,
        cmd2: int,
        data1: int = 0x00,
        data2: int = 0x00,
    ) -> bytes:
        if not (0 <= addr <= 0xFF):
            raise ValueError(f"addr must be 0~255, got {addr}")
        if not (0 <= cmd1 <= 0xFF and 0 <= cmd2 <= 0xFF):
            raise ValueError(f"cmd1/cmd2 must be 0~255, got {cmd1}/{cmd2}")
        if not (0 <= data1 <= 0xFF and 0 <= data2 <= 0xFF):
            raise ValueError(f"data1/data2 must be 0~255, got {data1}/{data2}")

        checksum = (addr + cmd1 + cmd2 + data1 + data2) & 0xFF
        frame = bytes([0xFF, addr, cmd1, cmd2, data1, data2, checksum])
        return frame

