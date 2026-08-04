from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Protocol, Dict, Any, List
import time, threading, logging, copy

logger = logging.getLogger("mrj4.client")
logger.setLevel(logging.WARN)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.DEBUG)
console_handler.setFormatter(logging.Formatter("[%(name)s] %(levelname)s: %(message)s"))

if not logger.hasHandlers():  # 중복 방지
    logger.addHandler(console_handler)

BIT_SERVO = 1 << 0  # SON : Servo on
BIT_CCW_LIMIT = 1 << 1  # LSP : 정회전 스트로크 엔드
BIT_CW_LIMIT = 1 << 2  # LSN : 역회전 스트로크 엔드

MASK_SPEED = 0b11 << 8
BIT_SPEED1 = 0b01 << 8  # SP2[b9], SP1[b8] : 2비트 조합으로 속도 1, 2, 3중 선택
BIT_SPEED2 = 0b10 << 8
BIT_SPEED3 = 0b11 << 8

MASK_MOTOR_RUN = 0b11 << 11
BIT_MOTOR_RUN_CCW = 0b01 << 11  # ST2[b12], ST1[b11] : 역회전 기동, 정회전 기동
BIT_MOTOR_RUN_CW = 0b10 << 11

INPUT_DEVICE_ONOFF = {
    'command': '92',
    'data_no': '60'
}

READ_HARD_INPUT_DEVICE_STATE = {
    'command': '12',
    'data_no': '00'
}

READ_SOFT_INPUT_DEVICE_STATE = {
    'command': '12',
    'data_no': '60'
}

READ_TORQUE_UNIT = {
    'command': '01',
    'data_no': '0A'
}

READ_TORQUE = {
    'command': '01',
    'data_no': '8A'
}

READ_CURRENT_ALARM_NO = {
    'command': '02',
    'data_no': '00'
}

READ_TORQUE_WHEN_ALARM = {
    'command': '35',
    'data_no': '0A'
}

READ_PARAMETER_GROUP = {
    'command': '04',
    'data_no': '01'
}

WRITE_PARAMETER_GROUP = {
    'command': '85',
    'data_no': '00'
}

# 스피드 메모리 읽기 명령 상수
READ_SPEED_MEMORY_1 = {
    'command': '15',
    'data_no': '05'
}

READ_SPEED_MEMORY_2 = {
    'command': '15',
    'data_no': '06'
}

READ_SPEED_MEMORY_3 = {
    'command': '15',
    'data_no': '07'
}

# 스피드 메모리 쓰기 명령 상수
WRITE_SPEED_MEMORY_1 = {
    'command': '94',
    'data_no': '05'
}

WRITE_SPEED_MEMORY_2 = {
    'command': '94',
    'data_no': '06'
}

WRITE_SPEED_MEMORY_3 = {
    'command': '94',
    'data_no': '07'
}

ByteStr = bytes
Timestamp = float

class MRJ4Error(Exception):
    pass

class ConnectState(Enum):
    DISCONNECTED = auto()
    CONNECTING   = auto()
    CONNECTED    = auto()
    DISCONNECTING= auto()

# ---- TODO 인터페이스 인 것 같은데 현재 상태와 맞는지, 필요한지 확인 필요 ----
class Transport(Protocol):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def write(self, data: ByteStr) -> int: ...
    def read(self, n: int) -> ByteStr: ...
    def in_waiting(self) -> int: ...

class Parser(Protocol):
    def feed(self, chunk: ByteStr) -> List["Packet"]: ...
    def reset(self) -> None: ...

@dataclass
class Packet:
    payload: ByteStr
    timestamp: Timestamp = field(default_factory=time.time)
# ------------------------------------------------------------------------

@dataclass
class CommState:
    # 송신 패킷
    packet: Optional[ByteStr] = None
    first_send_time: Optional[float] = None

    # 송신 에러복구
    tries_timeout: int = 0
    tries_comm_error: int = 0
    last_send_time: Optional[float] = None

    # 수신
    response: Optional[Packet] = None
    last_recv_time: Optional[float] = None

    # 파싱 메타 : 송신 패킷에서 추출했음
    cmd: Optional[str] = None
    data_no: Optional[str] = None
    # 파싱된 결과를 담는 일반 컨테이너
    parsed: Dict[str, Any] = field(default_factory=dict)
    parsing_error_msg: Optional[str] = None

    # 결과 상태 플래그
    result_ready: bool = False

    comm_success: bool = False
    comm_timeout: bool = False
    comm_error: bool = False
    comm_exception: bool = False
    comm_multiple_resp: bool = False
    parsing_success: bool = False
    parsing_error: bool = False

    def mark_comm_success(self):
        # parsing단계가 남아있어서 result_ready는 셋 안함
        self.comm_success = True

    def mark_parsing_success(self):
        self.parsing_success = True
        self.result_ready = True

    def mark_comm_timeout(self):
        self.result_ready = True
        self.comm_timeout = True

    def mark_comm_error(self):
        self.result_ready = True
        self.comm_error = True

    def mark_comm_exception(self):
        self.result_ready = True
        self.comm_exception = True

    def mark_comm_multiple_resp(self):
        self.result_ready = True
        self.comm_multiple_resp = True

    def mark_parsing_error(self):
        self.result_ready = True
        self.parsing_error = True

    def duration(self) -> Optional[float]:
        if self.first_send_time and self.last_recv_time:
            return self.last_recv_time - self.first_send_time
        return None
    
class MRJ4Request:
    '''
    station: str
    command: str
    data_no: str
    raw_data: bytes or int or float
    encoded_data: bytes
    packet: bytes
    '''
    __slots__ = ("station", "command", "data_no", "raw_data", "encoded_data", "packet")

    def __init__(self, station, command, data_no, raw_data = None):
        self.station = station
        self.command = command
        self.data_no = data_no
        self.raw_data = raw_data

# TODO protocol 파일로 옮길 예정
class MRJ4Communicator:

    def __init__(self, transport: Transport, parser: Parser, builder) -> None:
        self._RETRY_LIMIT = 3
        self._TIMEOUT = 0.3

        self._transport = transport
        self._parser = parser
        self._builder = builder

        # 연결 상태
        self.state = ConnectState.DISCONNECTED

        # 통신 상태 관리. guarded by _lock
        self._lock = threading.Lock()
        self._current_comm_state: Optional[CommState] = None
        self._is_comm_complete: bool = False
        self._is_parsing_complete: bool = False

        # 수신 스레드
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = threading.Event()

        # watchdog
        self._watchdog_thread: Optional[threading.Thread] = None
        self._watchdog_stop = threading.Event()

    def connect(self) -> None:
        if self.state != ConnectState.DISCONNECTED:
            return
        self.state = ConnectState.CONNECTING
        try:
            self._transport.open()
            self._start_comm_thread()
            self.state = ConnectState.CONNECTED
        except Exception as e:
            self.state = ConnectState.DISCONNECTED
            raise

    def disconnect(self) -> None:
        if self.state == ConnectState.DISCONNECTED:
            return
        self.state = ConnectState.DISCONNECTING
        self._stop_comm_thread()
        try:
            self._transport.close()
        finally:
            self.state = ConnectState.DISCONNECTED

    def set_request(self, request):
        """
        설명
        
        Args:
            MRJ4Request 객체
            추후 리스트로 변경 가능성 있음
            
        Returns:

        Raises:
            
        Example:
        """
        if request.raw_data is None:
            self._set_current_packet_to_process(request.station, request.command, request.data_no)
        else:
            if type(request.raw_data) is bytes:
                self._set_current_packet_to_process(request.station, request.command, request.data_no, request.raw_data)
        
    def is_complete(self) -> bool:
        return self._is_parsing_complete
    
    def get_comm_state(self) -> Optional[CommState]:
        with self._lock:
            return copy.deepcopy(self._current_comm_state)

    def clear_comm_state(self):
        if self._is_parsing_complete:
            self._clear_current_packet()
        else:
            raise MRJ4Error("처리중인 통신이 완료되지 않았습니다.")


    # --- 통신 처리 스레드의 데이터를 관리하는 기능들
    def _set_current_packet_to_process(self, station: str, command: str, data_no: str, data: Optional[bytes] = None) -> None:
        frame = self._builder.build(station=station, command=command, data_no=data_no, data=data)
        
        # TODO 주석처리된 부분은 MRJ4Communicator에서 packet_state를 어떻게 처리할지 결정 후에 수정 필요
        # entry = None
        with self._lock:
            if self._current_comm_state is not None:
                if not self._is_parsing_complete:
                    raise MRJ4Error("이전 요청 응답 대기 중 → 순차 전송만 가능")
                else:
                    raise MRJ4Error("이전 요청의 결과가 남아있음 → clear후 전송 가능")
            
            self._current_comm_state = CommState(packet=frame)
            # entry = copy.deepcopy(self._current_comm_state)

        # self.packet_state = entry
        self._is_comm_complete = False
        self._is_parsing_complete = False

        self._send_packet()

    # --- 통신 처리 스레드 관련 기능들
    def _send_packet(self) -> None:
        with self._lock:
            if self._current_comm_state is None or self._current_comm_state.packet is None:
                raise MRJ4Error("송신할 패킷이 설정되지 않았습니다.")
            
            # 패킷 스냅샷
            frame = bytes(self._current_comm_state.packet)
            # 전송 시간 기록
            now = time.time()
            if self._current_comm_state.first_send_time is None:
                self._current_comm_state.first_send_time = now
            self._current_comm_state.last_send_time = now
        logger.debug(f"[TX] {len(frame)}B: {frame.hex(' ').upper()}")
        self._transport.write(frame)

    def _send_packet_without_lock(self) -> None:
        if self._current_comm_state is None or self._current_comm_state.packet is None:
            raise MRJ4Error("송신할 패킷이 설정되지 않았습니다.")
        
        # 패킷 스냅샷
        frame = bytes(self._current_comm_state.packet)
        # 전송 시간 기록
        now = time.time()
        if self._current_comm_state.first_send_time is None:
            self._current_comm_state.first_send_time = now
        self._current_comm_state.last_send_time = now

        logger.debug(f"[TX] {len(frame)}B: {frame.hex(' ').upper()}")
        self._transport.write(frame)

    def _parse_received_packet(self):
        # TODO 현재 구조로는 _update_mrj4_client_state_with_packet랑 _clear_mrj4_packet를 꼭 쌍을 호출해야 하는 것 같은데.. 이런 제약사항이 있으면 나중에 실수하지 않을까?..
        # TODO 상태로 업데이트할 수 있는게 뭘까. Response에 나와있는 패킷의 command에 따라 알람, 토크, 인풋디바이스 상태 업데이트하기?
        # TODO 파싱중에 에러나는 것도 PacketState에서 관리해야 하는지. 그러면 거기에 parsing_error 필드를 추가하고 파싱 로직을 넣어야 하는지..
        
        # TODO 주석처리된 부분은 MRJ4Communicator에서 packet_state를 어떻게 처리할지 결정 후에 수정 필요

        with self._lock:
            if self._current_comm_state is None:
                # TODO 스레드에서 돌아서 raise가 main으로 전파가 안됨. 추후 검토 필요
                raise MRJ4Error("송신할 패킷이 설정되지 않았습니다.")
            else:
                if not self._is_comm_complete:
                    # TODO 스레드에서 돌아서 raise가 main으로 전파가 안됨. 추후 검토 필요
                    raise MRJ4Error("통신이 완료되지 않았습니다.")
        
            # self.packet_state = entry
            if not self._current_comm_state.comm_success or not self._current_comm_state.response:
                # TODO 위에 raise한거랑 통합해야됨. raise대신에 이런 식으로 에러 결과를 바깥쪽으로 전달하도록 수정해야겠음
                self._current_comm_state.mark_parsing_error()  # TODO 이게 맞을지 생각해보기
                self._is_parsing_complete = True
                return
        
            # packet에서 cmd, data_no 추출
            if len(self._current_comm_state.packet) < 7:
                # TODO 얘는 assert로 바꿔야될 것 같은데 그렇게 하면 이후 처리는 어떻게 되는거지?..
                self._current_comm_state.mark_parsing_error()
                self._is_parsing_complete = True
                raise ValueError(f"Frame too short: {self._current_comm_state.packet.hex(' ')}")
        
            # TODO 나중에 command, data_no 등을 추출하는 기능이 있는 패킷 객체를 만드는 건 어떨까
            cmd_str = self._current_comm_state.packet[2:4].decode("ascii").upper()
            data_no_str =  self._current_comm_state.packet[5:7].decode("ascii").upper()
            
            pk = self._current_comm_state.response

            try:
                # 원본 엔트리에 meta 저장
                self._current_comm_state.cmd = cmd_str
                self._current_comm_state.data_no = data_no_str

                if cmd_str == READ_CURRENT_ALARM_NO['command'] and data_no_str == READ_CURRENT_ALARM_NO['data_no']:
                    # 알람 번호는 문자열로 저장
                    alarm = pk.parse_alarm_no()
                    logger.info(f"현재 알람 : {alarm}")
                    self._current_comm_state.parsed['alarm'] = alarm

                elif cmd_str == READ_TORQUE['command'] and data_no_str == READ_TORQUE['data_no']:
                    # 토크 값은 파싱해서 저장
                    torque = pk.parse_status_data()
                    logger.info(f"현재 토크 : {torque}")
                    self._current_comm_state.parsed['torque'] = torque

                elif cmd_str == READ_SOFT_INPUT_DEVICE_STATE['command'] and data_no_str == READ_SOFT_INPUT_DEVICE_STATE['data_no']:
                    # 입력 상태는 정수값으로 저장
                    soft_input_device_state = pk.data_int()
                    logger.info(f"현재 소프트 인풋 디바이스 상태 : int : {soft_input_device_state}")
                    bin_str = bin(soft_input_device_state)[2:].zfill(32)
                    formatted_bin_str = " ".join(bin_str[i:i+4] for i in range(0, len(bin_str), 4))
                    logger.info(f"현재 소프트 인풋 디바이스 상태 : bin : {formatted_bin_str}")
                    self._current_comm_state.parsed['soft_input_device_state'] = soft_input_device_state
                
                elif cmd_str == READ_HARD_INPUT_DEVICE_STATE['command'] and data_no_str == READ_HARD_INPUT_DEVICE_STATE['data_no']:
                    hard_input_device_state = pk.data_int()
                    logger.info(f"현재 하드 인풋 디바이스 상태 : {hard_input_device_state}")
                    self._current_comm_state.parsed['hard_input_device_state'] = hard_input_device_state

                elif cmd_str == READ_PARAMETER_GROUP['command'] and data_no_str == READ_PARAMETER_GROUP['data_no']:
                    # 파라미터 그룹 값 읽기 응답 처리
                    parameter_group = pk.data_str()
                    logger.info(f"현재 파라미터 그룹: {parameter_group}")
                    self._current_comm_state.parsed['parameter_group'] = parameter_group

                elif cmd_str == WRITE_PARAMETER_GROUP['command'] and data_no_str == WRITE_PARAMETER_GROUP['data_no']:
                    # 파라미터 그룹 설정 완료 응답 처리
                    logger.info("파라미터 그룹 설정 완료")

                elif cmd_str == READ_SPEED_MEMORY_1['command'] and data_no_str == READ_SPEED_MEMORY_1['data_no']:
                    # 스피드 메모리 1 읽기 응답 처리 - parse_parameter() 사용하여 상세 정보 파싱
                    try:
                        speed_value = pk.parse_parameter()
                        raw_data = pk.data_str()
                        
                        # 상세 로깅: 값, 타입, 쓰기방식 표시
                        ascii_str = raw_data
                        display_write_control_byte = int(ascii_str[2], 16)
                        is_decimal = (display_write_control_byte & 0x01) != 0
                        is_immediate_write = (display_write_control_byte & 0x02) == 0
                        
                        display_type = "10진수" if is_decimal else "16진수"
                        write_type = "즉시유효" if is_immediate_write else "전원재투입후유효"
                        
                        logger.info(f"스피드 메모리 1: 값={speed_value}, 타입={display_type}, 쓰기방식={write_type}")

                        self._current_comm_state.parsed['speed1'] = speed_value
                    except Exception as e:
                        # 파싱 실패 시 원본 데이터로 로깅
                        speed_value = pk.data_str()
                        logger.warning(f"스피드 메모리 1 파싱 실패: {speed_value} (오류: {e})")
                        self._current_comm_state.parsed['speed1'] = speed_value
                        self._current_comm_state.parsing_error_msg = str(e)

                elif cmd_str == READ_SPEED_MEMORY_2['command'] and data_no_str == READ_SPEED_MEMORY_2['data_no']:
                    # 스피드 메모리 2 읽기 응답 처리 - parse_parameter() 사용하여 상세 정보 파싱
                    try:
                        speed_value = pk.parse_parameter()
                        raw_data = pk.data_str()
                        
                        # 상세 로깅: 값, 타입, 쓰기방식 표시
                        ascii_str = raw_data
                        display_write_control_byte = int(ascii_str[2], 16)
                        is_decimal = (display_write_control_byte & 0x01) != 0
                        is_immediate_write = (display_write_control_byte & 0x02) == 0
                        
                        display_type = "10진수" if is_decimal else "16진수"
                        write_type = "즉시유효" if is_immediate_write else "전원재투입후유효"
                        
                        logger.info(f"스피드 메모리 2: 값={speed_value}, 타입={display_type}, 쓰기방식={write_type}")

                        self._current_comm_state.parsed['speed2'] = speed_value
                    except Exception as e:
                        # 파싱 실패 시 원본 데이터로 로깅
                        speed_value = pk.data_str()
                        logger.warning(f"스피드 메모리 2 파싱 실패: {speed_value} (오류: {e})")
                        self._current_comm_state.parsed['speed2'] = speed_value
                        self._current_comm_state.parsing_error_msg = str(e)

                elif cmd_str == READ_SPEED_MEMORY_3['command'] and data_no_str == READ_SPEED_MEMORY_3['data_no']:
                    # 스피드 메모리 3 읽기 응답 처리 - parse_parameter() 사용하여 상세 정보 파싱
                    try:
                        speed_value = pk.parse_parameter()
                        raw_data = pk.data_str()
                        
                        # 상세 로깅: 값, 타입, 쓰기방식 표시
                        ascii_str = raw_data
                        display_write_control_byte = int(ascii_str[2], 16)
                        is_decimal = (display_write_control_byte & 0x01) != 0
                        is_immediate_write = (display_write_control_byte & 0x02) == 0
                        
                        display_type = "10진수" if is_decimal else "16진수"
                        write_type = "즉시유효" if is_immediate_write else "전원재투입후유효"
                        
                        logger.info(f"스피드 메모리 3: 값={speed_value}, 타입={display_type}, 쓰기방식={write_type}")

                        self._current_comm_state.parsed['speed3'] = speed_value
                    except Exception as e:
                        # 파싱 실패 시 원본 데이터로 로깅
                        speed_value = pk.data_str()
                        logger.warning(f"스피드 메모리 3 파싱 실패: {speed_value} (오류: {e})")
                        self._current_comm_state.parsed['speed3'] = speed_value
                        self._current_comm_state.parsing_error_msg = str(e)

                self._current_comm_state.mark_parsing_success()
                self._is_parsing_complete = True

            except Exception as e:
                logger.error(f"응답 해석 실패: {e}")
                self._current_comm_state.parsing_error_msg = str(e)
                # TODO mark에 파싱에러로 해줘야됨
                self._current_comm_state.mark_parsing_error()
                self._is_parsing_complete = True

    def _clear_current_packet(self):
        # TODO 로그/이력 저장
        with self._lock:
            self._current_comm_state = None
            self._is_comm_complete = False
            self._is_parsing_complete = False

    def _start_comm_thread(self):
        self._start_watchdog()
        self._start_rx_loop()

    def _stop_comm_thread(self):
        self._stop_watchdog()
        self._stop_rx_loop()

    # -------- watchdog (타임아웃 관리) --------
    def _start_watchdog(self) -> None:
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog, name="mrj4-watchdog", daemon=True
        )
        self._watchdog_thread.start()

    def _stop_watchdog(self) -> None:
        self._watchdog_stop.set()
        if (
            self._watchdog_thread
            and self._watchdog_thread.is_alive()
            and threading.current_thread() is not self._watchdog_thread
        ):
            self._watchdog_thread.join(timeout=1.0)
        self._watchdog_thread = None

    # TODO _lock 안에서 0.1초 쉬는거 -> 나중에 처리하기로 함
    def _watchdog(self):
        while not self._watchdog_stop.is_set():
            time.sleep(0.05)

            need_disconnect = False
            
            with self._lock:
                if not self._current_comm_state or self._is_comm_complete:
                    continue

                entry = self._current_comm_state
                now = time.time()

                if entry.last_send_time and (now - entry.last_send_time > self._TIMEOUT):
                    
                    self._transport.write(bytes([0x04]))
                    self._parser.reset()

                    if entry.tries_timeout < self._RETRY_LIMIT:
                        # 매뉴얼 규정: 300ms 이상 응답 없음 → EOT 송신 후 재송신
                        try:
                            logger.warning("응답 없음 → EOT 송신 (수신 상태 초기화)")
                            time.sleep(0.1)
                            self._send_packet_without_lock()
                            entry.tries_timeout += 1
                            logger.warning(
                                f"타임아웃 재송신 {entry.tries_timeout}/{self._RETRY_LIMIT}"
                            )
                        except Exception as e:
                            # TODO clear 전에 내부적으로 통신 결과 및 상태 업데이트 하고 에러나서 끝났다는, 복구하기 전에 더이상 진행 안된다는 내용도 올려야 함
                            entry.mark_comm_exception()
                            self._is_comm_complete = True
                            need_disconnect = True
                    else:
                        logger.error("타임아웃 3회 → disconnect")
                        entry.mark_comm_timeout()
                        self._is_comm_complete = True
                        need_disconnect = True
                        # TODO clear 전에 내부적으로 통신 결과 및 상태 업데이트 하고 에러나서 끝났다는, 복구하기 전에 더이상 진행 안된다는 내용도 올려야 함
                       
            if self._is_comm_complete:
                self._parse_received_packet()

            if need_disconnect:
                self.disconnect()


    # -------- 수신 루프 --------
    def _start_rx_loop(self) -> None:
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_worker, name="mrj4-rx", daemon=True)
        self._rx_thread.start()

    def _stop_rx_loop(self) -> None:
        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        self._rx_thread = None

    def _rx_worker(self) -> None:
        while not self._rx_stop.is_set():
            try:
                n = self._transport.in_waiting()
                if n <= 0:
                    time.sleep(0.005)
                    continue
                chunk = self._transport.read(n)
                if not chunk:
                    continue
                for pk in self._parser.feed(chunk):
                    self._handle_response(pk)
            except Exception as e:
                time.sleep(0.05)
                # TODO 시리얼 통신 끊기는 경우일 것 같은데.. 어떻게 처리하지?
    
    # -------- 응답 처리 --------
    # TODO _lock 안에서 logger 찍는게 괜찮은건지 모르겠네 -> 나중에 처리하기로 함
    def _handle_response(self, resp) -> None:
        logger.info(f"응답 수신: error={resp.errorcode}, data={resp.data_str()}")

        need_disconnect = False

        with self._lock:
            entry = self._current_comm_state
            if not entry:
                return
            
            entry.last_recv_time = time.time()
            entry.response = resp

            if resp.errorcode in "Aa":  # 정상
                entry.mark_comm_success()
                self._is_comm_complete = True

            elif resp.errorcode in "BbCcDdEeFf":  # 통신 오류 응답
                if entry.tries_comm_error < self._RETRY_LIMIT:
                    try:
                        self._send_packet_without_lock()
                        entry.tries_comm_error += 1
                        logger.warning(
                            f"오류코드 {resp.errorcode} → 재송신 {entry.tries_comm_error}/{self._RETRY_LIMIT}"
                        )
                    except Exception:
                        # TODO clear 전에 내부적으로 통신 결과 및 상태 업데이트 하고 에러나서 끝났다는, 복구하기 전에 더이상 진행 안된다는 내용도 올려야 함
                        
                        entry.mark_comm_exception()
                        self._is_comm_complete = True
                        need_disconnect = True
                else:
                    logger.error("통신오류 응답 3회 → disconnect")
                    # TODO clear 전에 내부적으로 통신 결과 및 상태 업데이트 하고 에러나서 끝났다는, 복구하기 전에 더이상 진행 안된다는 내용도 올려야 함
                    entry.mark_comm_error()
                    self._is_comm_complete = True
                    need_disconnect = True

        if self._is_comm_complete:
                self._parse_received_packet()
        
        if need_disconnect:
            self.disconnect()


class MRJ4Client:
    def __init__(self, communicator) -> None:
        self._communicator = communicator

        # 클라이언트 상태
        # TODO client용 상태 변수들에도 뭔가 다른 lock을 걸어놔야 할까 싶다
        self.torque: int = 0
        self.alarm: str = ''
        self.input_device_state: int = 0
        self.is_state_updated = False
        self.packet_state: Optional[CommState] = None
        
        # input device 제어 변수 (32비트)
        # TODO 초기값 바꿀까.. 무서우니까.. 아니면 처음에 업데이트 되기 전까지 그냥 사용 못하게 할까?
        self.input_device_control_data: int = 0

    # 연결
    def connect(self) -> None:
        self._communicator.connect()

    def disconnect(self) -> None:
        self._communicator.disconnect()

    def is_comm_complete(self) -> bool:
        return self._communicator.is_complete()

    def try_update_from_comm(self) -> bool:
        """
        Communicator에서 통신/파싱이 끝난 CommState 스냅샷을 가져와
        Client의 상태(alarm, torque, input_device_state 등)를 갱신한다.
        성공적으로 반영하면 Communicator의 상태를 clear 한다.
        Returns:
            bool: 이번 호출에서 무언가를 반영/정리했으면 True
        """
        st = self._communicator.get_comm_state()
        if not st or not st.result_ready:
            return False

        # 성공 케이스
        if st.parsing_success and st.parsed:
            # 공통 키들만 우선 반영. 필요하면 확장.
            if 'alarm' in st.parsed:
                self.alarm = st.parsed['alarm']
            if 'torque' in st.parsed:
                self.torque = st.parsed['torque']
            if 'soft_input_device_state' in st.parsed:
                self.input_device_state = st.parsed['soft_input_device_state']
            # 하드 입력 상태도 원하면 필드 추가해서 반영 가능:
            # if 'hard_input_state' in st.parsed: self.hard_input_device_state = st.parsed['hard_input_state']

            self.packet_state = st  # 히스토리/디버깅용으로 보관해도 됨
            self.is_state_updated = True

        # 에러/타임아웃 등의 메타도 필요하면 여기서 확인
        # elif st.timeout: ...
        # elif st.comm_error: ...
        # elif st.exception: ...

        # Communicator의 현재 엔트리 비우기 (다음 요청 가능)
        self._communicator.clear_comm_state()
        return True

    def set_input_device_control_data_for_recv_value(self):
        self.input_device_control_data = self.input_device_state

    def _update_input_device(self) -> None:
        if (self.input_device_control_data & (1 << 0)) == 0:
            self.input_device_control_data |= BIT_SERVO
            logger.error("안전상의 이유로 서보를 끌 수 없습니다. 서보를 다시 켭니다.")
            # return  # 원래 return하려고 했는데 그 이후 동작이 모호해져서 우선 그냥 서보 켜는 설정 하고 그대로 세팅 해버리는걸로 했음
        
        """input_device_control_data를 ASCII 8자리로 변환해 전송"""
        data_ascii = f"{self.input_device_control_data:08X}".encode("ascii")
        # self._set_mrj4_packet(station="0", command=INPUT_DEVICE_ONOFF['command'], data_no=INPUT_DEVICE_ONOFF['data_no'], data=data_ascii)
        self._communicator.set_request(MRJ4Request(station="0", command=INPUT_DEVICE_ONOFF['command'], data_no=INPUT_DEVICE_ONOFF['data_no'], raw_data=data_ascii))

    # TODO init 값을 바꾸자.. 아니다 이게 왜필요하냐.. 걍 MRJ4에서 받아오면 되는데
    # def servo_init(self):
    #     self.input_device_control_data = 0
    #     self._update_input_device()

    def servo_on(self):
        self.input_device_control_data |= BIT_SERVO
        self._update_input_device()

    def servo_off(self):
        self.input_device_control_data &= ~BIT_SERVO
        self._update_input_device()

    def cw_limit_on(self):
        self.input_device_control_data |= BIT_CW_LIMIT
        self._update_input_device()

    def cw_limit_off(self):
        self.input_device_control_data &= ~BIT_CW_LIMIT
        self._update_input_device()

    def ccw_limit_on(self):
        self.input_device_control_data |= BIT_CCW_LIMIT
        self._update_input_device()

    def ccw_limit_off(self):
        self.input_device_control_data &= ~BIT_CCW_LIMIT
        self._update_input_device()

    def select_speed_1(self):
        self.input_device_control_data &= ~MASK_SPEED
        self.input_device_control_data |= BIT_SPEED1
        self._update_input_device()

    def select_speed_2(self):
        self.input_device_control_data &= ~MASK_SPEED
        self.input_device_control_data |= BIT_SPEED2
        self._update_input_device()

    def select_speed_3(self):
        self.input_device_control_data &= ~MASK_SPEED
        self.input_device_control_data |= BIT_SPEED3
        self._update_input_device()

    def run_motor_cw(self):
        self.input_device_control_data &= ~MASK_MOTOR_RUN
        self.input_device_control_data |= BIT_MOTOR_RUN_CW
        self._update_input_device()

    def run_motor_ccw(self):
        self.input_device_control_data &= ~MASK_MOTOR_RUN
        self.input_device_control_data |= BIT_MOTOR_RUN_CCW
        self._update_input_device()

    def run_motor_stop(self):
        self.input_device_control_data &= ~MASK_MOTOR_RUN
        self._update_input_device()

    def read_hard_input_device_state(self) -> None:
        """하드웨어 입력장치 상태 읽기 요청"""
        # self._set_mrj4_packet(station="0", command=READ_HARD_INPUT_DEVICE_STATE['command'], data_no=READ_HARD_INPUT_DEVICE_STATE['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=READ_HARD_INPUT_DEVICE_STATE['command'], data_no=READ_HARD_INPUT_DEVICE_STATE['data_no']))

    def read_soft_input_device_state(self) -> None:
        """소프트웨어 입력장치 상태 읽기 요청"""
        # self._set_mrj4_packet(station="0", command=READ_SOFT_INPUT_DEVICE_STATE['command'], data_no=READ_SOFT_INPUT_DEVICE_STATE['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=READ_SOFT_INPUT_DEVICE_STATE['command'], data_no=READ_SOFT_INPUT_DEVICE_STATE['data_no']))

    def read_torque_unit(self) -> None:
        """현재 토크 단위 및 심볼 읽기 요청"""
        # self._set_mrj4_packet(station="0", command=READ_TORQUE_UNIT['command'], data_no=READ_TORQUE_UNIT['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=READ_TORQUE_UNIT['command'], data_no=READ_TORQUE_UNIT['data_no']))
    
    def read_torque(self) -> None:
        """현재 토크 읽기 요청"""
        # self._set_mrj4_packet(station="0", command=READ_TORQUE['command'], data_no=READ_TORQUE['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=READ_TORQUE['command'], data_no=READ_TORQUE['data_no']))

    def read_current_alarm_no(self) -> None:
        """현재 발생 중인 알람 번호 읽기 요청"""
        # self._set_mrj4_packet(station="0", command=READ_CURRENT_ALARM_NO['command'], data_no=READ_CURRENT_ALARM_NO['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=READ_CURRENT_ALARM_NO['command'], data_no=READ_CURRENT_ALARM_NO['data_no']))

    def read_torque_when_alarm(self, station: str = "0") -> None:
        """알람 발생 시점의 토크 읽기 요청"""
        # self._set_mrj4_packet(station="0", command=READ_TORQUE_WHEN_ALARM['command'], data_no=READ_TORQUE_WHEN_ALARM['data_no'])
        self._communicator.set_request(MRJ4Request(station=station, command=READ_TORQUE_WHEN_ALARM['command'], data_no=READ_TORQUE_WHEN_ALARM['data_no']))

    def read_parameter_group(self) -> None:
        """현재 파라미터 그룹 값 읽기 요청
        
        응답은 PacketState를 통해 확인
        """
        # self._set_mrj4_packet(station="0", command=READ_PARAMETER_GROUP['command'], data_no=READ_PARAMETER_GROUP['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=READ_PARAMETER_GROUP['command'], data_no=READ_PARAMETER_GROUP['data_no']))

    def write_parameter_group(self, group_value: int) -> None:
        """파라미터 그룹 값 설정 요청
        
        Args:
            group_value: 설정할 파라미터 그룹 값 (0: 기본, 2: 스피드 메모리 접근용)
            
        Raises:
            MRJ4Error: 유효하지 않은 파라미터 그룹 값인 경우
            
        응답은 PacketState를 통해 확인
        """
        # 유효성 검사
        self._validate_parameter_group(group_value)
        # TODO group_value 인코딩 필요
        
        # 새로운 파싱 기능을 사용하여 int 값을 직접 전달
        # self._set_mrj4_packet(station="0", command=WRITE_PARAMETER_GROUP['command'], data_no=WRITE_PARAMETER_GROUP['data_no'], data=group_value)
        self._communicator.set_request(MRJ4Request(station="0", command=WRITE_PARAMETER_GROUP['command'], data_no=WRITE_PARAMETER_GROUP['data_no'], raw_data=group_value))

    def _validate_parameter_group(self, group_value: int) -> None:
        """파라미터 그룹 값 유효성 검사
        
        Args:
            group_value: 검사할 파라미터 그룹 값
            
        Raises:
            MRJ4Error: 유효하지 않은 파라미터 그룹 값인 경우
        """
        valid_groups = [0, 2]  # 0: 기본값, 2: 스피드 메모리 접근용
        if group_value not in valid_groups:
            raise MRJ4Error(f"Invalid parameter group: {group_value}. Must be one of {valid_groups}")

    def _validate_memory_number(self, memory_number: int) -> None:
        """스피드 메모리 번호 유효성 검사
        
        Args:
            memory_number: 검사할 스피드 메모리 번호
            
        Raises:
            MRJ4Error: 유효하지 않은 메모리 번호인 경우
        """
        valid_memory_numbers = [1, 2, 3]  # 스피드 메모리 1, 2, 3만 허용
        if memory_number not in valid_memory_numbers:
            raise MRJ4Error(f"Invalid memory number: {memory_number}. Must be one of {valid_memory_numbers}")

    def _validate_speed_value(self, speed_value: int) -> None:
        """스피드 값 유효성 검사
        
        Args:
            speed_value: 검사할 스피드 값
            
        Raises:
            MRJ4Error: 유효하지 않은 스피드 값인 경우
        """
        min_speed = 1      # 최소 스피드 값
        max_speed = 7900   # 최대 스피드 값
        if not (min_speed <= speed_value <= max_speed):
            raise MRJ4Error(f"Invalid speed value: {speed_value}. Must be between {min_speed} and {max_speed}")

    def read_speed_memory(self, memory_number: int) -> None:
        """특정 스피드 메모리 값 읽기 요청 (비동기)
        
        Args:
            memory_number: 스피드 메모리 번호 (1, 2, 3)
            
        Raises:
            MRJ4Error: 유효하지 않은 메모리 번호인 경우
            
        응답은 PacketState를 통해 확인
        """
        # 메모리 번호 유효성 검사
        self._validate_memory_number(memory_number)
        
        # 메모리 번호에 따라 해당 명령 상수 선택
        if memory_number == 1:
            command_info = READ_SPEED_MEMORY_1
        elif memory_number == 2:
            command_info = READ_SPEED_MEMORY_2
        elif memory_number == 3:
            command_info = READ_SPEED_MEMORY_3
        
        # 패킷 전송
        # self._set_mrj4_packet(station="0", command=command_info['command'], data_no=command_info['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=command_info['command'], data_no=command_info['data_no']))

    def read_speed_memory_1(self) -> None:
        """스피드 메모리 1 값 읽기 요청 (비동기)
        
        응답은 PacketState를 통해 확인
        """
        # self._set_mrj4_packet(station="0", command=READ_SPEED_MEMORY_1['command'], data_no=READ_SPEED_MEMORY_1['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=READ_SPEED_MEMORY_1['command'], data_no=READ_SPEED_MEMORY_1['data_no']))

    def read_speed_memory_2(self) -> None:
        """스피드 메모리 2 값 읽기 요청 (비동기)
        
        응답은 PacketState를 통해 확인
        """
        # self._set_mrj4_packet(station="0", command=READ_SPEED_MEMORY_2['command'], data_no=READ_SPEED_MEMORY_2['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=READ_SPEED_MEMORY_2['command'], data_no=READ_SPEED_MEMORY_2['data_no']))

    def read_speed_memory_3(self) -> None:
        """스피드 메모리 3 값 읽기 요청 (비동기)
        
        응답은 PacketState를 통해 확인
        """
        # self._set_mrj4_packet(station="0", command=READ_SPEED_MEMORY_3['command'], data_no=READ_SPEED_MEMORY_3['data_no'])
        self._communicator.set_request(MRJ4Request(station="0", command=READ_SPEED_MEMORY_3['command'], data_no=READ_SPEED_MEMORY_3['data_no']))

    def write_speed_memory(self, memory_number: int, speed_value: int) -> None:
        """특정 스피드 메모리에 값 설정 요청 (비동기)
        
        Args:
            memory_number: 스피드 메모리 번호 (1, 2, 3)
            speed_value: 설정할 스피드 값 (1 ~ 7900)
            
        Raises:
            MRJ4Error: 유효하지 않은 메모리 번호나 스피드 값인 경우
            
        응답은 PacketState를 통해 확인
        """
        # 유효성 검사
        self._validate_memory_number(memory_number)
        self._validate_speed_value(speed_value)
        # TODO speed_value 인코딩 필요
        
        # 메모리 번호에 따라 해당 명령 상수 선택
        if memory_number == 1:
            command_info = WRITE_SPEED_MEMORY_1
        elif memory_number == 2:
            command_info = WRITE_SPEED_MEMORY_2
        elif memory_number == 3:
            command_info = WRITE_SPEED_MEMORY_3
        
        # 새로운 파싱 기능을 사용하여 int 값을 직접 전달
        # self._set_mrj4_packet(station="0", command=command_info['command'], data_no=command_info['data_no'], data=speed_value)
        self._communicator.set_request(MRJ4Request(station="0", command=command_info['command'], data_no=command_info['data_no'], raw_data=speed_value))

    def write_speed_memory_1(self, speed_value: int) -> None:
        """스피드 메모리 1에 값 설정 요청 (비동기)
        
        Args:
            speed_value: 설정할 스피드 값 (1 ~ 7900)
            
        Raises:
            MRJ4Error: 유효하지 않은 스피드 값인 경우
            
        응답은 PacketState를 통해 확인
        """
        # 유효성 검사
        self._validate_speed_value(speed_value)
        # TODO speed_value 인코딩 필요
        
        # 새로운 파싱 기능을 사용하여 int 값을 직접 전달
        # self._set_mrj4_packet(station="0", command=WRITE_SPEED_MEMORY_1['command'], data_no=WRITE_SPEED_MEMORY_1['data_no'], data=speed_value)
        self._communicator.set_request(MRJ4Request(station="0", command=WRITE_SPEED_MEMORY_1['command'], data_no=WRITE_SPEED_MEMORY_1['data_no'], raw_data=speed_value))

    def write_speed_memory_2(self, speed_value: int) -> None:
        """스피드 메모리 2에 값 설정 요청 (비동기)
        
        Args:
            speed_value: 설정할 스피드 값 (1 ~ 7900)
            
        Raises:
            MRJ4Error: 유효하지 않은 스피드 값인 경우
            
        응답은 PacketState를 통해 확인
        """
        # 유효성 검사
        self._validate_speed_value(speed_value)
        # TODO speed_value 인코딩 필요
        
        # 새로운 파싱 기능을 사용하여 int 값을 직접 전달
        # self._set_mrj4_packet(station="0", command=WRITE_SPEED_MEMORY_2['command'], data_no=WRITE_SPEED_MEMORY_2['data_no'], data=speed_value)
        self._communicator.set_request(MRJ4Request(station="0", command=WRITE_SPEED_MEMORY_2['command'], data_no=WRITE_SPEED_MEMORY_2['data_no'], raw_data=speed_value))

    def write_speed_memory_3(self, speed_value: int) -> None:
        """스피드 메모리 3에 값 설정 요청 (비동기)
        
        Args:
            speed_value: 설정할 스피드 값 (1 ~ 7900)
            
        Raises:
            MRJ4Error: 유효하지 않은 스피드 값인 경우
            
        응답은 PacketState를 통해 확인
        """
        # 유효성 검사
        self._validate_speed_value(speed_value)
        # TODO speed_value 인코딩 필요
        
        # 새로운 파싱 기능을 사용하여 int 값을 직접 전달
        # self._set_mrj4_packet(station="0", command=WRITE_SPEED_MEMORY_3['command'], data_no=WRITE_SPEED_MEMORY_3['data_no'], data=speed_value)
        self._communicator.set_request(MRJ4Request(station="0", command=WRITE_SPEED_MEMORY_3['command'], data_no=WRITE_SPEED_MEMORY_3['data_no'], raw_data=speed_value))

