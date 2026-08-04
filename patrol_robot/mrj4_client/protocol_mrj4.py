from __future__ import annotations
from typing import Optional, List, Union
import logging

logger = logging.getLogger("mrj4.protocol")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_handler.setFormatter(logging.Formatter("[%(name)s] %(levelname)s: %(message)s"))

if not logger.hasHandlers():  # 중복 방지
    logger.addHandler(console_handler)

ByteStr = bytes

# Control codes
SOH = 0x01  # 01H
STX = 0x02  # 02H
ETX = 0x03  # 03H
EOT = 0x04  # 04H

ALLOWED_DATA_LEN = {0, 4, 8, 12, 16}


def _hex_str_to_ascii_bytes(val: str, width: int) -> ByteStr:
    """
    16진수 문자열을 지정된 폭의 ASCII 바이트로 변환하고 검증
    
    Args:
        val: 변환할 16진수 문자열 (예: '04', '85', '0A')
        width: 출력 폭 (문자 수)
    
    Returns:
        ASCII 인코딩된 바이트
    
    Note:
        입력 문자열은 대문자로 정규화되며, 영숫자만 허용됨
        주로 MRJ4 프로토콜의 명령 코드, 데이터 번호 등에 사용
    """
    s = str(val).upper()
    if len(s) != width:
        raise ValueError(f"ASCII field width mismatch: expect {width}, got {len(s)} ('{s}')")
    # 영숫자만 허용 (필요시 완화 가능)
    if not s.isalnum():
        raise ValueError(f"ASCII field must be alphanumeric: '{s}'")
    return s.encode("ascii")

def _checksum_ascii_lowbyte(station_to_etx: ByteStr) -> ByteStr:
    """
    [station..ETX] 합의 하위 1바이트 → 16진수 2자리(대문자) ASCII.
    예: 합 = 0x1FC → 0xFC → b'F' b'C' (0x46 0x43)
    """
    total_lo = (sum(station_to_etx) & 0xFF)
    return f"{total_lo:02X}".encode("ascii")

class MRJ4Response:
    """
    MR-J4 응답 패킷 파싱 헬퍼.
    payload 전체(bytes)를 받아서 station/error/data/checksum 필드 분리.
    """
    __slots__ = ("raw", "station", "errorcode", "data", "checksum")

    def __init__(self, payload: bytes):
        self.raw = payload
        if len(payload) < 6 or payload[0] != 0x02 or payload[-3] != 0x03:
            raise ValueError(f"Invalid MR-J4 response frame: {payload.hex(' ')}")

        self.station = chr(payload[1])      # ASCII → 문자
        self.errorcode = chr(payload[2])    # ASCII → 문자
        self.data = payload[3:-3]           # ETX 전까지가 data
        self.checksum = payload[-2:]        # raw checksum (ASCII HEX 2B)

    def data_str(self) -> str:
        """data 영역을 ASCII로 해석"""
        try:
            return self.data.decode("ascii")
        except Exception:
            return self.data.hex(" ")
        
    def data_int(self) -> int:
        """data 영역을 hex 문자열로 보고 int로 해석"""
        try:
            return int(self.data.decode("ascii"), 16)
        except Exception:
            raise ValueError(f"Cannot convert data to int: {self.data!r}")

    def parse_alarm_no(self) -> str:
        """AL.__형태로 반환"""
        try:
            raw_value = self.data.decode("ascii")[2:4]
            value_to_show = "  " if raw_value == 'FF' else raw_value
            
            return "AL."+value_to_show
        except Exception:
            return self.data.hex(" ")
    
    def parse_status_data(self) -> int | float | str:
        """
        상태 표시 데이터 파싱
        - 총 12바이트 (ASCII)
        - [0:2] 예약 "00"
        - [2] 소수점 위치 (0~6)
        - [3] 표시 타입 (0=10진수, 1=16진수)
        - [4:] 값 (8자리)
        """
        ascii_str = self.data.decode("ascii")
        if len(ascii_str) != 12:
            raise ValueError(f"Invalid length: {len(ascii_str)}, expected 12")

        decimal_pos = int(ascii_str[2])
        display_type = ascii_str[3]
        value_str = ascii_str[4:]

        # value_str은 16진수 표현 (예: "FFFFFFFF")
        raw_val = int(value_str, 16)

        # 음수 보정 (32bit signed 기준)
        if raw_val & 0x80000000:
            raw_val -= 0x100000000

        if display_type == "0":  # 10진수 모드
            # 소수점 위치 적용
            if decimal_pos > 0:
                return raw_val / (10 ** (decimal_pos - 1))
            else:
                return raw_val
        elif display_type == "1":  # 16진수 모드
            return raw_val
        else:
            raise ValueError(f"Unknown display type: {display_type}")

    def parse_parameter(self) -> int | float | str:
        """
        MRJ4 파라미터 데이터 파싱 (12바이트 ASCII)
        
        MRJ4에 저장된 각종 파라미터 값을 파싱합니다.
        
        데이터 구조:
        - [0] 예약 영역
        - [1] 부호 유무 (0번 비트: 0=부호있음, 1=부호없음)
        - [2] 표시 타입 (0번 비트: 0=16진수, 1=10진수)
              쓰기 타입 (1번 비트: 0=즉시유효, 1=전원재투입후유효)
        - [3] 소수점 위치 (0~6)
        - [4:] 8바이트 16진수 값
        
        표시 형식:
        - 10진수 표시: 소수점 위치에 따라 값 계산
        - 16진수 표시 (소수점 위치 0): 16진수 값 그대로 반환
        - 특수 16진 표시 (소수점 위치 ≠ 0): F는 공백으로 처리하여 16진수 문자열 반환
        
        Returns:
            int | float | str: 파싱된 파라미터 값
        """
        ascii_str = self.data.decode("ascii")
        if len(ascii_str) != 12:
            raise ValueError(f"파라미터 데이터 길이 오류: {len(ascii_str)}, 예상: 12")

        # 데이터[1]: 부호 유무 (0번 비트: 0=부호있음, 1=부호없음)
        sign_control_byte = int(ascii_str[1], 16)
        has_sign = (sign_control_byte & 0x01) == 0  # 0번 비트가 0이면 부호있음

        # 데이터[2]: 표시 타입과 쓰기 타입
        display_write_control_byte = int(ascii_str[2], 16)
        is_decimal = (display_write_control_byte & 0x01) != 0  # 0번 비트: 1=10진수, 0=16진수

        # 데이터[3]: 소수점 위치
        decimal_position = int(ascii_str[3], 16)
        if decimal_position > 6:
            raise ValueError(f"소수점 위치 범위 오류: {decimal_position}, 최대: 6")

        # 8바이트 16진수 값 파싱 (데이터[4:])
        value_str = ascii_str[4:12]  # 8자리 16진수
        if len(value_str) != 8:
            raise ValueError(f"파라미터 값 영역 길이 오류: {len(value_str)}, 예상: 8")

        if is_decimal:
            # 10진수 표시 형식
            raw_value = int(value_str, 16)
            
            # 부호 처리 (32bit signed 기준)
            if has_sign and (raw_value & 0x80000000):
                raw_value -= 0x100000000
            
            # 소수점 위치에 따른 값 계산
            # 소수점 위치는 뒤에서 몇 번째 자리에 점을 찍는지를 의미
            # 예: 소수점 위치 2 = 뒤에서 2번째 자리 = 10^1로 나누기
            if decimal_position > 0:
                return raw_value / (10 ** (decimal_position - 1))
            else:
                return raw_value
        else:
            # 16진수 표시 형식
            if decimal_position == 0:
                # 일반 16진수 표시: 숫자 값으로 반환
                raw_value = int(value_str, 16)
                
                # 부호 처리 (32bit signed 기준)
                if has_sign and (raw_value & 0x80000000):
                    raw_value -= 0x100000000
                
                return raw_value
            else:
                # 특수 16진 표시: F는 공백으로 처리하여 16진수 문자열 반환
                # TODO: 추정 구현 - MRJ4 매뉴얼 확인 필요
                # 실제 특수 16진 표시 형식의 정확한 동작 방식을 확인해야 함
                # F를 공백으로 치환하고 앞의 0과 공백을 제거
                hex_display = value_str.replace('F', ' ').lstrip('0 ')
                if not hex_display:  # 모든 문자가 0이나 F인 경우
                    hex_display = '0'
                return hex_display


    def __repr__(self):
        return (f"<MRJ4Response station={self.station!r} "
                f"errorcode={self.errorcode!r} "
                f"data={self.data_str()!r} "
                f"checksum={self.checksum.decode('ascii','ignore')!r}>")

class StreamParser:
    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> List[MRJ4Response]:
        out: List[MRJ4Response] = []
        if not chunk:
            return out
        self._buf.extend(chunk)
        logger.debug(f"[PARSER] feed: got {len(chunk)}B, buffer={self._buf.hex(' ')}")

        while True:
            # STX 찾기
            try:
                stx = self._buf.index(0x02)
            except ValueError:
                self._buf.clear()
                break
            if stx > 0:
                del self._buf[:stx]

            if len(self._buf) < 6:
                break

            try:
                etx = self._buf.index(0x03, 3)
            except ValueError:
                break

            if len(self._buf) < etx + 3:
                break

            csum = bytes(self._buf[etx+1:etx+3])
            body = bytes(self._buf[1:etx+1])
            calc = _checksum_ascii_lowbyte(body)
            if csum != calc:
                logger.debug(f"[PARSER] checksum mismatch got={csum} calc={calc}, resync")
                del self._buf[0]
                continue

            frame_end = etx + 3
            frame = bytes(self._buf[:frame_end])
            logger.debug(f"[PARSER] complete frame={frame.hex(' ')}")

            try:
                resp = MRJ4Response(frame)
                out.append(resp)
            except Exception as e:
                logger.debug(f"[PARSER] failed to parse frame: {e}")

            del self._buf[:frame_end]

        return out
    
    def reset(self):
        logger.debug("[PARSER] buffer reset")
        self._buf.clear()

class MRJ4PacketBuilder:
    """
    프레임:
      SOH(1) | station(1 ASCII) | command(2 ASCII) | STX(1) |
      data_no(2 ASCII) | data(var) | ETX(1) | checksum(2 ASCII HEX)

    * EOT(0x04)는 패킷에 넣지 않음(상수만 유지).
    * station/command/data_no는 모두 문자열로 받아 ASCII로 인코딩.
    * data 길이: {0,4,8,12,16}
    * data는 bytes, int, float 타입 지원 (command/data_no에 따라 자동 파싱)
    """
    
    def _parse_parameter_group_write_data(self, value: Union[int, float]) -> ByteStr:
        """
        파라미터 그룹 쓰기용 데이터 파싱
        입력된 int값을 ASCII로 인코딩된 4byte hex값으로 변환
        
        Args:
            value: 파라미터 그룹 값 (int 또는 float)
            
        Returns:
            ASCII 인코딩된 4바이트 16진수 문자열
            
        Example:
            2 -> b'0002'
            15 -> b'000F'
        """
        int_value = int(value)
        if int_value < 0 or int_value > 0xFFFF:
            raise ValueError(f"파라미터 그룹 값은 0~65535 범위여야 합니다: {int_value}")
        
        return f"{int_value:04X}".encode("ascii")
    
    def _parse_parameter_value_write_data(self, value: Union[int, float]) -> ByteStr:
        """
        파라미터 값 쓰기용 데이터 파싱
        복잡한 파싱 로직이 필요한 경우를 위한 함수 자리
        
        Args:
            value: 파라미터 값 (int 또는 float)
            
        Returns:
            파싱된 바이트 데이터
            
        Note:
            현재는 기본 구현만 제공, 추후 상세 파싱 로직 추가 예정
        """
        # TODO: 실제 파라미터 값 쓰기 파싱 로직 구현
        # 현재는 8바이트 16진수 ASCII로 변환하는 기본 구현
        int_value = int(value)
        if int_value < 0:
            # 음수 처리 (32bit signed)
            int_value = int_value & 0xFFFFFFFF
        
        return f"{int_value:08X}".encode("ascii")
    
    def _parse_data_by_command(self, command: str, data_no: str, data: Union[int, float, ByteStr]) -> ByteStr:
        """
        command와 data_no 조합에 따라 적절한 데이터 파싱 수행
        
        Args:
            command: 명령 코드
            data_no: 데이터 번호
            data: 파싱할 데이터 (int, float, 또는 bytes)
            
        Returns:
            파싱된 바이트 데이터
        """
        if isinstance(data, bytes):
            return data
        
        # 명령 코드를 대문자로 정규화
        cmd_upper = command.upper()
        data_no_upper = data_no.upper()
        
        # 파라미터 그룹 쓰기 (command='85', data_no='00')
        if cmd_upper == '85' and data_no_upper == '00':
            return self._parse_parameter_group_write_data(data)
        
        # 파라미터 값 쓰기 (command='94')
        elif cmd_upper == '94':
            return self._parse_parameter_value_write_data(data)
        
        # 기타 경우: 기본 8바이트 16진수 ASCII 변환
        else:
            int_value = int(data)
            if int_value < 0:
                # 음수 처리 (32bit signed)
                int_value = int_value & 0xFFFFFFFF
            
            return f"{int_value:08X}".encode("ascii")

    def build(
        self,
        *,
        station: str,
        command: str,
        data_no: str,
        data: Optional[Union[ByteStr, int, float]] = None,
    ) -> ByteStr:
        # 데이터 파싱 처리
        if data is None:
            payload = b""
        else:
            payload = self._parse_data_by_command(command, data_no, data)
        
        if len(payload) not in ALLOWED_DATA_LEN:
            raise ValueError(f"data length must be one of {sorted(ALLOWED_DATA_LEN)}, got {len(payload)}")

        sta_b = _hex_str_to_ascii_bytes(station, 1)  # '0'..'9' 1자리
        cmd_b = _hex_str_to_ascii_bytes(command, 2)  # 예: '04' -> b'04'
        no_b  = _hex_str_to_ascii_bytes(data_no, 2)  # 예: '01' -> b'01'

        # [station..ETX] (SOH 제외)
        station_to_etx = sta_b + cmd_b + bytes([STX]) + no_b + payload + bytes([ETX])

        # 체크섬 (ASCII HEX 2바이트)
        csum_b = _checksum_ascii_lowbyte(station_to_etx)

        # 최종 프레임: SOH + [station..ETX] + checksum
        frame = bytes([SOH]) + station_to_etx + csum_b
        return frame
