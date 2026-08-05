from __future__ import annotations
import threading
import queue
import time
import json
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Optional
import argparse
import signal
import atexit
import sys
import logging

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False

logger = logging.getLogger("ftm")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.DEBUG)
console_handler.setFormatter(logging.Formatter("[%(name)s] %(levelname)s: %(message)s"))

if not logger.hasHandlers():  # 중복 방지
    logger.addHandler(console_handler)

from mrj4_client import (
    MRJ4Client, MRJ4Communicator, MRJ4PacketBuilder,
    SerialTransport as MRJ4SerialTransport,
    StreamParser as MRJ4StreamParser
)

from remote_controller_client import (
    RCClient,
    SerialTransport as RCSerialTransport,
    Event as RCEvent,
    RCPacketBuilder,
    StreamParser as RCStreamParser,
)

from pymodbus.client import ModbusTcpClient #pip3 install pymodbus

# ===== 소비자가 처리할 커맨드 정의 =====
class Cmd(Enum):
    FOLD = auto()
    UNFOLD = auto()
    STOP_FOLDING = auto()
    SELECT_FOLD_SPEED = auto()
    SET_CCW_LIMIT = auto()
    SET_CW_LIMIT = auto()
    RESET_CCW_LIMIT = auto()
    RESET_CW_LIMIT = auto()

    RAISE_MAST = auto()
    LOWER_MAST = auto()
    STOP_MAST = auto()

    TILT_UP = auto()
    TILT_DOWN = auto()
    TILT_STOP = auto()
    PAN_CW = auto()
    PAN_CCW = auto()
    PAN_STOP = auto()

    # 기타 확장용
    NOOP = auto()      # 아무것도 안 함(테스트/디버그용)
    SHUTDOWN = auto()  # 워커 스레드 종료


@dataclass
class Command:
    kind: Cmd
    args: tuple[Any, ...] = ()
    kwargs: dict[str, Any] = None

    def __post_init__(self):
        if self.kwargs is None:
            self.kwargs = {}


class FoldableTelescopicMast:
    def __init__(self, servo_port_name:str, metal_sensor_addr: str, rc_port_name: str) -> None:
        """폴딩 가능한 텔레스코픽 마스트 제어 클래스의 인스턴스를 초기화합니다."""
        self._folding_servo_port_name = servo_port_name
        self._folding_servo = None  # 폴딩 서보 모터 제어 객체
        self._folding_servo_state = None  # 폴딩 서보 상태 객체

        # ── 폴딩 서보 폴링 상태머신(단일 플라이트) ──
        self._fsrv_seq = ("alarm", "torque", "soft_input")  # 라운드로빈 순서
        self._fsrv_idx = 0           # 다음에 보낼 종류의 인덱스
        self._fsrv_inflight = False  # 현재 송신·수신 진행 중인지
        self._fsrv_last_send = 0.0   # 마지막 송신 시각 (스로틀링용)
        self._fsrv_min_gap = 0.05     # 연속 송신 간 최소 간격(필요시 >0로)

        # ── 서보 전용 큐(사용자 서보 커맨드만 들어감) ──
        self._servo_q: queue.Queue[Command] = queue.Queue(maxsize=128)

        self._mast_and_pantilt = None # 텔레스코픽 마스트 및 팬틸트 제어 객체
        # self.mast_and_pantilt_state = None  # 텔레스코픽 마스트 및 팬틸트 상태 객체

        self._metal_sensors_addr = metal_sensor_addr # 디바이스 IP:PORT
        self._metal_sensors = None  # 금속감지센서 제어 객체
        self._metal_sensors_state = None  # 금속감지센서 상태 객체

        self._rc_port_name = rc_port_name
        self._rc = None  # 리모콘 제어 객체
        self._rc_state = None  # 리모콘 상태 객체

        # ===== 생산자-소비자 인프라 =====
        self._q: queue.Queue[Command] = queue.Queue(maxsize=128)  # backpressure 가능
        self._stop_evt = threading.Event()
        self._worker: Optional[threading.Thread] = None

        # ===== Proxy SUB 소켓 (TelescopicMast → 마스트 제어 명령 수신) =====
        self._proxy_ipc_address = "/tmp/iae_patrol_v1_telescopic_mast_proxy.ipc"
        self._proxy_zmq_ctx: Optional[Any] = None
        self._proxy_sub_socket: Optional[Any] = None
        self._proxy_sub_thread: Optional[threading.Thread] = None
        self._proxy_sub_running = False

        # 폴링 주기(초): 상황에 맞게 조정
        self._sensor_poll_interval = 0.05   # 20Hz
        self._rc_poll_interval = 0.02       # 50Hz
        self._fold_servo_poll_interval = 0.05  # 20Hz (상황 따라 10~50Hz 조정)

        # 내부 타이머
        self._last_sensor_poll = 0.0
        self._last_rc_poll = 0.0
        self._last_fold_servo_poll = 0.0

        # ===== 데이터 예시 =====
        # 최근 폴딩서보 상태 캐시(예시)
        self._folding_servo_state = {
            "alarm": None,
            "torque": None,
            "limit_fold": False,
            "limit_unfold": False,
        }

    def init_devices(self) -> None:
        self._init_folding_servo(self._folding_servo_port_name)
        self._init_mast_and_pantilt()
        self._init_metal_sensors(self._metal_sensors_addr)
        self._init_rc(self._rc_port_name)

    def test_devivces(self) -> bool:
        result = list()
        result.append(self._test_folding_servo())
        result.append(self._test_mast_and_pantilt())
        result.append(self._test_metal_sensors())
        result.append(self._test_rc())

        return (not False in result)

    def deinit_devices(self) -> None:
        self._deinit_folding_servo()
        self._deinit_mast_and_pantilt()
        self._deinit_metal_sensors()
        self._deinit_rc()

    # 폴딩 언폴딩 관련 메서드

    def fold(self) -> None:
        """펼쳐진 텔레스코픽 마스트를 접습니다."""
        logger.debug("Put fold cmd...")
        self._enqueue_servo(Command(Cmd.FOLD))

    def unfold(self) -> None:
        """폴딩된 텔레스코픽 마스트를 펼칩니다."""
        logger.debug("Put unfold cmd...")
        self._enqueue_servo(Command(Cmd.UNFOLD))

    def stop_folding_action(self) -> None:
        """폴딩 또는 언폴딩 동작을 즉시 중지합니다."""
        logger.debug("Put stop folding cmd...")
        self._enqueue_servo(Command(Cmd.STOP_FOLDING))

    def select_folding_speed(self, speed_level: int) -> None:
        """
        폴딩 및 언폴딩 속도를 선택합니다.
        
        :param speed: 속도 레벨 (1, 2, 3)
        """
        if speed_level not in [1, 2, 3]:
            raise ValueError("Speed must be 1 or 2 or 3.")
        logger.debug("Put select folding speed cmd...")
        self._enqueue_servo(Command(Cmd.SELECT_FOLD_SPEED, args=(speed_level,)))
    
    def get_folding_state(self) -> str:
        """
        현재 폴딩 상태를 반환합니다.
        
        :return: 'folded', 'unfolded', 'folding', 'unfolding', 'stopped' 중 하나
        """
        # 실제 구현에서는 하드웨어 상태를 읽어와야 합니다.
        state = "stopped"
        return state
    
    def _enqueue_servo(self, cmd: Command) -> None:
        try:
            self._servo_q.put_nowait(cmd)
        except queue.Full:
            # 정책 선택: 가장 오래된 것 드롭/무시/경고 중 택1
            logger.warn("[servo-q] dropped cmd due to full queue:", cmd.kind.name)


    # 텔레스코픽 마스트 관련 메서드
    
    def raise_mast(self) -> None:
        """텔레스코픽 마스트를 올립니다."""
        logger.debug("Put raise mast cmd...")
        self._q.put(Command(Cmd.RAISE_MAST))

    def lower_mast(self) -> None:
        """텔레스코픽 마스트를 내립니다."""
        logger.debug("Put lower mast cmd...")
        self._q.put(Command(Cmd.LOWER_MAST))

    def stop_mast_action(self) -> None:
        """텔레스코픽 마스트의 동작을 즉시 중지합니다."""
        logger.debug("Put stop mast action cmd...")
        self._q.put(Command(Cmd.STOP_MAST))

    
    # 팬틸트 관련 메서드
        
    def tilt_up(self) -> None:
        """팬틸트를 위로 움직입니다."""
        logger.debug("Put tilt up cmd...")
        self._q.put(Command(Cmd.TILT_UP))
    
    def tilt_down(self) -> None:
        """팬틸트를 아래로 움직입니다."""
        logger.debug("Put tilt down cmd...")
        self._q.put(Command(Cmd.TILT_DOWN))

    def tilt_stop(self) -> None:
        """팬틸트의 움직임을 즉시 중지합니다."""
        logger.debug("Put stop tilt cmd...")
        self._q.put(Command(Cmd.TILT_STOP))

    def pan_ccw(self) -> None:
        """팬을 반시계 방향으로 움직입니다."""
        logger.debug("Put pan ccw cmd...")
        self._q.put(Command(Cmd.PAN_CCW))

    def pan_cw(self) -> None:
        """팬을 시계 방향으로 움직입니다."""
        logger.debug("Put pan cw cmd...")
        self._q.put(Command(Cmd.PAN_CW))

    def pan_stop(self) -> None:
        """팬의 움직임을 즉시 중지합니다."""
        logger.debug("Put stop pan cmd...")
        self._q.put(Command(Cmd.PAN_STOP))

    
    # 센서 관련 메서드    
    def get_sensor_state(self) -> dict:
        """
        폴딩 상태를 나타내는 4개의 금속감지센서 상태를 반환합니다.
        """
        if not self._metal_sensors:
            return

        rr = self._metal_sensors.read_discrete_inputs(address=0, count=4) #FC02를 이용해 DI Value Read Request
        if rr.isError():
            raise RuntimeError(f"Modbus 오류: {rr}")
        
        di = rr.bits[:4]  # [DI0, DI1, DI2, DI3]

        logger.debug(f"metal sensors DI0={int(di[0])} DI1={int(di[1])} DI2={int(di[2])} DI3={int(di[3])} | t={time.perf_counter():7.2f} ms")

        # 실제 구현에서는 하드웨어 센서 상태를 읽어와야 합니다.
        # TODO 아래 매핑이 맞는지 확인 필요
        sensor_state = {
            'fold_limit': (int(di[0]) == 1),
            'mid_limit': (int(di[1]) == 1),
            'vertical_limit': (int(di[2]) == 1),
            'unfold_limit': (int(di[3]) == 1)
        }
        return sensor_state
    

    # 리모콘 관련 메서드
    def get_rc_command(self) -> str:
        """
        리모콘으로부터 들어온 명령을 반환합니다.
        
        :return: 명령 문자열
        """
        # 실제 구현에서는 리모콘 입력을 읽어와야 합니다.
        command = "NO_COMMAND"
        return command
    

    # ======== 워커 스레드 관리 ========
    def start(self) -> None:
        """routine() 워커 스레드를 시작한다."""
        logger.info("Call routine start...")
        if self._worker and self._worker.is_alive():
            return
        self._stop_evt.clear()
        self._worker = threading.Thread(target=self._routine, name="FTM-routine", daemon=True)
        self._worker.start()

        # Proxy SUB 시작 (TelescopicMast로부터 마스트 제어 명령 수신)
        self._start_proxy_subscriber()

    def stop(self, join: bool = True, timeout: Optional[float] = 2.0) -> None:
        """워커 스레드를 종료한다."""
        logger.info("Call routine stop...")
        self._stop_evt.set()

        # Proxy SUB 종료
        self._stop_proxy_subscriber()

        # 큐가 꽉 차 있어도 멈추지 않게 비블로킹으로 깨우기
        try:
            self._q.put_nowait(Command(Cmd.SHUTDOWN))
        except queue.Full:
            pass  # 이미 꽉 찼다면 워커가 timeout으로 깨어남

        # 선택: 서보 큐 비우기(잔여 커맨드 드랍)
        try:
            while True:
                self._servo_q.get_nowait()
                self._servo_q.task_done()
        except queue.Empty:
            pass

        if join and self._worker:
            self._worker.join(timeout=timeout)
            if self._worker.is_alive():
                logger.warn("[warn] worker did not stop within timeout")

    
    # ======== 소비자(워커): 큐에서 커맨드를 꺼내 실행 + 주기적 폴링 ========
    def _routine(self) -> None:
        """통합 제어 루틴(소비자 스레드). 큐를 소비하고, 센서/RC를 주기적으로 폴링한다."""
        logger.debug("[routine] start")
        now0 = time.monotonic()
        self._last_sensor_poll = now0
        self._last_rc_poll = now0
        self._last_fold_servo_poll = now0

        # 메인 루프: get(timeout=…)으로 blocking + 폴링 타임슬라이스를 섞는다
        while not self._stop_evt.is_set():
            now = time.monotonic()

            # (A) 완료된 서보 통신 있으면 항상 흡수 (inflight 여부와 무관)
            if self._folding_servo:
                cli: MRJ4Client = self._folding_servo
                if cli.is_comm_complete():
                    self._complete_fold_servo_read(cli)

            # 1) 센서 폴링
            if now - self._last_sensor_poll >= self._sensor_poll_interval:
                self._poll_sensors()
                self._last_sensor_poll = now

            # 2) RC 폴링 → 명령 맵핑하여 큐에 스스로 넣을 수도 있음(자체 생산)
            if now - self._last_rc_poll >= self._rc_poll_interval:
                self._poll_rc()
                self._last_rc_poll = now

            # (B) 서보 전용 큐에서 1건만 꺼내 전송 시도 (사용자 액션 우선)
            if not self._fsrv_inflight:
                try:
                    servo_cmd = self._servo_q.get_nowait()
                except queue.Empty:
                    servo_cmd = None
                if servo_cmd is not None:
                    sent = self._send_servo_command(servo_cmd)
                    if not sent:
                        # 지금 보낼 수 없으면 뒤로 재삽입(백프레셔 정책은 상황 맞게)
                        try:
                            self._servo_q.put_nowait(servo_cmd)
                        except queue.Full:
                            logger.warn("[servo-q] requeue failed: full")
                    else:
                        self._servo_q.task_done()

            # (C) 폴딩서보 폴링
            if now - self._last_fold_servo_poll >= self._fold_servo_poll_interval:
                self._fold_servo_poll_step(now)
                self._last_fold_servo_poll = now

            # 3) 큐 소비(커맨드 처리)
            try:
                timeout = min(self._sensor_poll_interval,
                              self._rc_poll_interval,
                              self._fold_servo_poll_interval)
                cmd: Command = self._q.get(timeout=timeout)  # 루틴 회전 인터벌 제어용으로 get 사용
            except queue.Empty:
                continue

            try:
                if cmd.kind is Cmd.SHUTDOWN:
                    logger.debug("[routine] shutdown requested")
                    break
                self._dispatch(cmd)
            except Exception as e:
                # 하드웨어 오류나 예외는 여기서 로깅/격리
                logger.warn(f"[routine] command {cmd.kind.name} failed: {e!r}")
            finally:
                self._q.task_done()

        logger.debug("[routine] stop")

    # ======== 내부 헬퍼들 ========
    def _try_put_nonblocking(self, cmd: Command) -> None:
        try:
            self._q.put_nowait(cmd)
        except queue.Full:
            # 정책: 조용히 드랍하거나, 경고 로그를 찍거나, 기존 STOP 중복 방지 등
            # logger.warn(f"[queue] drop {cmd.kind.name}")
            pass

    # ======== Proxy SUB 소켓 (TelescopicMast IPC 명령 수신) ========

    def _start_proxy_subscriber(self) -> None:
        """ZMQ SUB 소켓을 생성하여 TelescopicMast proxy IPC에서 마스트 제어 명령을 수신한다."""
        if not ZMQ_AVAILABLE:
            logger.warning("[proxy-sub] zmq not available, proxy subscriber disabled")
            return
        if self._proxy_sub_running:
            return

        self._proxy_sub_running = True
        self._proxy_sub_thread = threading.Thread(
            target=self._proxy_sub_worker, name="FTM-proxy-sub", daemon=True
        )
        self._proxy_sub_thread.start()
        logger.info(f"[proxy-sub] Proxy subscriber started on ipc://{self._proxy_ipc_address}")

    def _stop_proxy_subscriber(self) -> None:
        """Proxy SUB 소켓 및 수신 스레드를 종료한다."""
        self._proxy_sub_running = False
        if self._proxy_sub_thread and self._proxy_sub_thread.is_alive():
            self._proxy_sub_thread.join(timeout=2.0)
        self._proxy_sub_thread = None

        if self._proxy_sub_socket:
            try:
                self._proxy_sub_socket.close(linger=0)
            except Exception:
                pass
            self._proxy_sub_socket = None

        if self._proxy_zmq_ctx:
            try:
                self._proxy_zmq_ctx.term()
            except Exception:
                pass
            self._proxy_zmq_ctx = None

    def _proxy_sub_worker(self) -> None:
        """ZMQ SUB 수신 워커: mast_command 토픽의 JSON 메시지를 파싱하여 대응 함수를 호출한다."""
        try:
            self._proxy_zmq_ctx = zmq.Context()
            self._proxy_sub_socket = self._proxy_zmq_ctx.socket(zmq.SUB)
            self._proxy_sub_socket.setsockopt(zmq.LINGER, 0)
            self._proxy_sub_socket.setsockopt(zmq.RCVTIMEO, 500)  # 500ms timeout for clean shutdown
            self._proxy_sub_socket.setsockopt(zmq.RECONNECT_IVL, 500)
            self._proxy_sub_socket.connect(f"ipc://{self._proxy_ipc_address}")
            self._proxy_sub_socket.subscribe(b"mast_command")
            logger.info(f"[proxy-sub] Connected to ipc://{self._proxy_ipc_address}")
        except Exception as e:
            logger.error(f"[proxy-sub] Failed to create SUB socket: {e}")
            self._proxy_sub_running = False
            return

        command_dispatch = {
            "raise_mast": self.raise_mast,
            "lower_mast": self.lower_mast,
            "stop_mast_action": self.stop_mast_action,
        }

        while self._proxy_sub_running and not self._stop_evt.is_set():
            try:
                multipart = self._proxy_sub_socket.recv_multipart()
                if len(multipart) >= 2:
                    topic = multipart[0]
                    payload_bytes = multipart[1]
                    if topic == b"mast_command" and payload_bytes:
                        data = json.loads(payload_bytes.decode('utf-8'))
                        cmd_name = data.get("command", "")
                        handler = command_dispatch.get(cmd_name)
                        if handler:
                            logger.info(f"[proxy-sub] Received command: {cmd_name}")
                            handler()
                        else:
                            logger.warning(f"[proxy-sub] Unknown command: {cmd_name}")
            except zmq.Again:
                continue  # recv timeout, check stop condition
            except (zmq.ContextTerminated, zmq.ZMQError) as e:
                logger.debug(f"[proxy-sub] ZMQ error ({e}), stopping")
                break
            except Exception as e:
                logger.error(f"[proxy-sub] Error processing command: {e}")

        logger.info("[proxy-sub] Proxy subscriber worker exited")
    
    def _send_servo_command(self, cmd: Command) -> bool:
        """
        서보 커맨드 1건을 전송. 이미 in-flight면 False.
        MRJ4Client 호출이 성공적으로 set_request까지 가면 True 반환하며 in-flight 진입.
        """
        if self._stop_evt.is_set():
            return False
        if self._folding_servo is None:
            return False
        if self._fsrv_inflight:
            return False

        try:
            if cmd.kind is Cmd.FOLD:
                logger.debug("[CMD] FOLD"); self._do_fold()
            elif cmd.kind is Cmd.UNFOLD:
                logger.debug("[CMD] UNFOLD"); self._do_unfold()
            elif cmd.kind is Cmd.STOP_FOLDING:
                logger.debug("[CMD] STOP_FOLDING"); self._do_stop_folding()
            elif cmd.kind is Cmd.SELECT_FOLD_SPEED:
                speed, = cmd.args
                logger.debug(f"[CMD] SELECT_FOLD_SPEED({speed})"); self._do_select_fold_speed(speed)
            elif cmd.kind is Cmd.SET_CCW_LIMIT:
                logger.debug("[CMD] SET CCW LIMIT"); self._do_set_folding_ccw_limit()
            elif cmd.kind is Cmd.RESET_CCW_LIMIT:
                logger.debug("[CMD] RESET CCW LIMIT"); self._do_reset_folding_ccw_limit()
            elif cmd.kind is Cmd.SET_CW_LIMIT:
                logger.debug("[CMD] SET CW LIMIT"); self._do_set_folding_cw_limit()
            elif cmd.kind is Cmd.RESET_CW_LIMIT:
                logger.debug("[CMD] RESET CW LIMIT"); self._do_reset_folding_cw_limit()
            else:
                return True  # 서보 커맨드가 아니면 처리한 걸로

            self._fsrv_inflight = True
            self._fsrv_last_send = time.monotonic()
            return True
        except Exception as e:
            logger.warn(f"[servo] send failed: {e!r}")
            return False

    def _fold_servo_poll_step(self, now: float) -> None:
        if self._stop_evt.is_set():
            return
        if self._folding_servo is None:
            return
        
        cli: MRJ4Client = self._folding_servo

        # 진행 중이면 완료 체크
        if self._fsrv_inflight:
            return
        
        # 비행 중이 아니면: 먼저 서보 큐에 남은 사용자 커맨드가 있는지부터 확인
        # (있다면 폴링은 뒤로 미룸. 사용자 액션 우선)
        if not self._servo_q.empty():
            return

        # 폴링 전송(라운드로빈)
        if (now - self._fsrv_last_send) < self._fsrv_min_gap:
            return

        kind = self._fsrv_seq[self._fsrv_idx]
        try:
            if kind == "alarm":
                cli.read_current_alarm_no()
            elif kind == "torque":
                cli.read_torque()
            elif kind == "soft_input":
                cli.read_soft_input_device_state()
            else:
                # 알 수 없으면 다음으로 스킵
                self._fsrv_idx = (self._fsrv_idx + 1) % len(self._fsrv_seq)
                return

            self._fsrv_inflight = True
            self._fsrv_last_send = now
            # 다음 회차 종류는 완료 시점에 인덱스 넘김
        except Exception as e:
            # 송신 실패 → 다음 종류로 넘어가고 in-flight 해제
            logger.warn(f"[fold-servo] send failed({kind}): {e!r}")
            self._fsrv_idx = (self._fsrv_idx + 1) % len(self._fsrv_seq)
            self._fsrv_inflight = False


    def _complete_fold_servo_read(self, cli: "MRJ4Client") -> None:
        """
        통신/파싱이 완료된 경우 Client가 결과를 흡수(try_update_from_comm)하고
        로컬 캐시(self._folding_servo_state)를 최신화한다.
        """
        # Client가 Communicator에서 스냅샷을 가져와 파싱 성공 시
        # self.alarm/self.torque/self.input_device_state를 갱신하고
        # Communicator.clear까지 수행함.
        try:
            cli.try_update_from_comm()
        except Exception as e:
            logger.warn(f"[fold-servo] try_update_from_comm failed: {e!r}")

        # 캐시 업데이트(적용 여부와 무관하게 최신값을 반영)
        # - 알람: 문자열
        # - 토크: 수치
        # - 소프트 인풋 상태: int (클라이언트 필드명은 input_device_state)
        if isinstance(cli.alarm, str):
            self._folding_servo_state["alarm"] = cli.alarm
        if isinstance(cli.torque, (int, float)):
            self._folding_servo_state["torque"] = cli.torque
        if isinstance(cli.input_device_state, int):
            soft_state = cli.input_device_state
            # 비트 매핑은 실제 프로토콜에 맞춰 조정
            # 여기서는 예시로 LSN/LSP 의미의 자리 가정
            # BIT_CCW_LIMIT = 1 << 1  # LSP : 정회전(CCW, unfold) 스트로크 엔드
            # BIT_CW_LIMIT = 1 << 2  # LSN : 역회전(CW, fold) 스트로크 엔드
            self._folding_servo_state["limit_fold"] = bool(soft_state & (1 << 2))
            self._folding_servo_state["limit_unfold"] = bool(soft_state & (1 << 1))

        # 다음 종류로 라운드로빈
        self._fsrv_idx = (self._fsrv_idx + 1) % len(self._fsrv_seq)
        self._fsrv_inflight = False

    def _poll_sensors(self) -> None:
        new_state = self.get_sensor_state()

        # 이전 상태가 None이면 첫 업데이트로 간주
        if self._metal_sensors_state is None:
            self._metal_sensors_state = new_state
            return

        # 이전 상태와 다를 때만 처리
        if new_state != self._metal_sensors_state:
            self._metal_sensors_state = new_state

            sensor_on_cnt = 0
            if new_state.get("fold_limit"): sensor_on_cnt += 1
            if new_state.get("unfold_limit"): sensor_on_cnt += 1
            if new_state.get("vertical_limit"): sensor_on_cnt += 1
            if new_state.get("mid_limit"): sensor_on_cnt += 1

            if sensor_on_cnt > 1:
                # 물리적으로 동시에 참이면 이상 상황 → 전부 정지
                self._enqueue_servo(Command(Cmd.STOP_FOLDING))
                self._try_put_nonblocking(Command(Cmd.STOP_MAST))

            # 보호 로직. limit 닿았을 때 LIMIT 걸고 모터 정지, 센서에 닿지 않았을 때 LIMIT 해제
            if new_state.get("fold_limit"):
                self._enqueue_servo(Command(Cmd.RESET_CW_LIMIT))
                self._enqueue_servo(Command(Cmd.STOP_FOLDING))
            else:
                self._enqueue_servo(Command(Cmd.SET_CW_LIMIT))
            
            if new_state.get("vertical_limit"):
                self._enqueue_servo(Command(Cmd.RESET_CCW_LIMIT))
                self._enqueue_servo(Command(Cmd.STOP_FOLDING))
            else:
                self._enqueue_servo(Command(Cmd.SET_CCW_LIMIT))

            # 다른 센서 조건도 여기에 추가 가능

    def _poll_rc(self) -> None:
        rc_cmd = self.get_rc_command()
        # 예시 맵핑: 실제 문자열은 네 리모컨 프로토콜에 맞춰 바꿔
        mapping = {
            "FOLD": Command(Cmd.FOLD),
            "UNFOLD": Command(Cmd.UNFOLD),
            "FOLD_STOP": Command(Cmd.STOP_FOLDING),
            "SPEED1": Command(Cmd.SELECT_FOLD_SPEED, args=(1,)),
            "SPEED2": Command(Cmd.SELECT_FOLD_SPEED, args=(2,)),
            "SPEED3": Command(Cmd.SELECT_FOLD_SPEED, args=(3,)),

            "MAST_UP": Command(Cmd.RAISE_MAST),
            "MAST_DOWN": Command(Cmd.LOWER_MAST),
            "MAST_STOP": Command(Cmd.STOP_MAST),

            "TILT_UP": Command(Cmd.TILT_UP),
            "TILT_DOWN": Command(Cmd.TILT_DOWN),
            "TILT_STOP": Command(Cmd.TILT_STOP),

            "PAN_CW": Command(Cmd.PAN_CW),
            "PAN_CCW": Command(Cmd.PAN_CCW),
            "PAN_STOP": Command(Cmd.PAN_STOP),
        }
        cmd = mapping.get(rc_cmd)
        if cmd:
            # RC는 고빈도라 디바운싱/중복 억제 로직을 둘 수 있음
            self._try_put_nonblocking(cmd)

    def _dispatch(self, cmd: Command) -> None:
        """커맨드를 실제 하드웨어 API로 연결."""
        k = cmd.kind
        # TODO 서보 큐 잘돌아가는지 확인하고 지울 예정
        # if k is Cmd.FOLD:
        #     logger.debug("[CMD] FOLD"); self._do_fold()
        # elif k is Cmd.UNFOLD:
        #     logger.debug("[CMD] UNFOLD"); self._do_unfold()
        # elif k is Cmd.STOP_FOLDING:
        #     logger.debug("[CMD] STOP_FOLDING"); self._do_stop_folding()
        # elif k is Cmd.SELECT_FOLD_SPEED:
        #     speed, = cmd.args
        #     logger.debug(f"[CMD] SELECT_FOLD_SPEED({speed})"); self._do_select_fold_speed(speed)
        # elif k is Cmd.SET_CCW_LIMIT:
        #     logger.debug("[CMD] SET CCW LIMIT"); self._do_set_folding_ccw_limit()
        # elif k is Cmd.RESET_CCW_LIMIT:
        #     logger.debug("[CMD] RESET CCW LIMIT"); self._do_reset_folding_ccw_limit()
        # elif k is Cmd.SET_CW_LIMIT:
        #     logger.debug("[CMD] SET CW LIMIT"); self._do_set_folding_cw_limit()
        # elif k is Cmd.RESET_CW_LIMIT:
        #     logger.debug("[CMD] RESET CW LIMIT"); self._do_reset_folding_cw_limit()

        if k is Cmd.RAISE_MAST:
            logger.debug("[CMD] RAISE_MAST"); self._do_raise_mast()
        elif k is Cmd.LOWER_MAST:
            logger.debug("[CMD] LOWER_MAST"); self._do_lower_mast()
        elif k is Cmd.STOP_MAST:
            logger.debug("[CMD] STOP_MAST"); self._do_stop_mast()

        elif k is Cmd.TILT_UP:
            logger.debug("[CMD] TILT_UP"); self._do_tilt_up()
        elif k is Cmd.TILT_DOWN:
            logger.debug("[CMD] TILT_DOWN"); self._do_tilt_down()
        elif k is Cmd.TILT_STOP:
            logger.debug("[CMD] TILT_STOP"); self._do_tilt_stop()
        elif k is Cmd.PAN_CW:
            logger.debug("[CMD] PAN_CW"); self._do_pan_cw()
        elif k is Cmd.PAN_CCW:
            logger.debug("[CMD] PAN_CCW"); self._do_pan_ccw()
        elif k is Cmd.PAN_STOP:
            logger.debug("[CMD] PAN_STOP"); self._do_pan_stop()
        elif k is Cmd.NOOP:
            pass
        else:
            logger.debug(f"[CMD] Unhandled: {k}")

    # ===== 실제 하드웨어 호출부(여기선 logger.debug로 대체) =====

    def _init_folding_servo(self, port_name):
        logger.debug("Init folding servo...")

        tr = MRJ4SerialTransport(port=port_name)
        ps = MRJ4StreamParser()
        bd = MRJ4PacketBuilder()

        com = MRJ4Communicator(tr, ps, bd)
        self._folding_servo = MRJ4Client(com)
        self._folding_servo.connect()

    def _init_mast_and_pantilt(self):
        logger.debug("Init mast and pantilt...")

    def _init_metal_sensors(self, addr):
        logger.debug("Init metal sensors...")

        self._metal_sensors = ModbusTcpClient(addr.split(':')[0], port=addr.split(':')[1])
        self._metal_sensors.connect()

    def _init_rc(self, port_name):
        logger.debug("Init rc...")

        transport = RCSerialTransport(port=port_name)
        parser = RCStreamParser()
        builder = RCPacketBuilder()
        self._rc = RCClient(transport=transport, parser=parser, builder=builder)

        self._rc.on_connected(lambda **kw: logger.debug("[RC] CONNECTED"))
        self._rc.on_disconnected(lambda **kw: logger.debug("[RC] DISCONNECTED"))
        self._rc.on_error(lambda error, **kw: logger.debug(f"[RC] ERROR: {error}"))

        # def on_frame(data, **kw):
        #     logger.debug(f"[RC] FRAME {len(data)}B: {data.hex(' ')}")

        def on_packet(packet, **kw):
            logger.debug(f"[RC] PACKET: {packet}")
            logger.debug(f"   addr={packet.addr}, cmd1={packet.cmd1}, cmd2={packet.cmd2}, data1={packet.data1}, data2={packet.data2}, checksum={packet.checksum}")

            # pan/tilt
            if packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 8 and packet.data1 == 0 and packet.data2 == 32:
                self.tilt_up()
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 16 and packet.data1 == 0 and packet.data2 == 32:
                self.tilt_down()

            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 4 and packet.data1 == 32 and packet.data2 == 0:
                self.pan_cw()  # cw, ccw가 바뀌었을 수 있음. 확인
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 2 and packet.data1 == 32 and packet.data2 == 0:
                self.pan_ccw()

            # mast
            elif packet.addr == 1 and packet.cmd1 == 1 and packet.cmd2 == 70 and packet.data1 == 0 and packet.data2 == 1:
                self.raise_mast()
            elif packet.addr == 1 and packet.cmd1 == 1 and packet.cmd2 == 70 and packet.data1 == 0 and packet.data2 == 2:
                self.lower_mast()

            # Top Left Button On
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 6 and packet.data1 == 10 and packet.data2 == 26:
                logger.debug('Top Left Button On')

            # Top Right Button On
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 9 and packet.data1 == 0 and packet.data2 == 6:
                logger.debug('Top Right Button On')
            # Top Right Button Off
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 11 and packet.data1 == 0 and packet.data2 == 6:
                logger.debug('Top Right Button Off')

            # LED L/R
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 9 and packet.data1 == 0 and packet.data2 == 2:
                logger.debug('LED L')
                self.unfold()
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 9 and packet.data1 == 0 and packet.data2 == 4:
                logger.debug('LED R')
                self.fold()

            # LED Brightness
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 96 and packet.data1 == 0:
                logger.debug(f'brightness {packet.data2}')
                self.stop_folding_action()

            # RESET
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 12 and packet.data1 == 0 and packet.data2 == 1:
                logger.debug('RESET')

            # common stop
            elif packet.addr == 1 and packet.cmd1 == 0 and packet.cmd2 == 0 and packet.data1 == 0 and packet.data2 == 0:
                self.tilt_stop()
                self.pan_stop()
                self.stop_mast_action()
                logger.debug('Top Left Button Off')
            

        # self._rc.on_frame(on_frame)
        self._rc.on_packet(on_packet)

        self._rc.connect()

    
    def _test_folding_servo(self) -> bool:
        logger.debug("Test folding servo...")

        if not self._folding_servo:
            return False

        def wait_for_result(client: MRJ4Client, timeout: float = 2.0) -> bool:
            """
            현재 송신한 패킷에 대한 응답이 올 때까지 동기식으로 기다린다.
            성공/실패 여부를 PacketState로 반환.
            """
            start = time.time()
            while time.time() - start < timeout:
                time.sleep(0.01)
                if not client.is_comm_complete():
                    continue
                else:
                    client.try_update_from_comm()
                    return True
            return False
    
        # 알람 읽기 테스트                
        logger.info(f"알람 번호 읽기 요청")
        self._folding_servo.read_current_alarm_no()
        result = wait_for_result(self._folding_servo, timeout=2.0)
        if not result:
            logger.error(f"알람 번호 읽기 실패: wait_for_result 타임아웃")
            return False
        else:
            pkt = self._folding_servo.packet_state
            if pkt.result_ready and pkt.parsing_success:
                logger.info(f"알람 번호 읽기 성공: 응답={pkt.response}")
                logger.info(f"결과 값 in packet_state={pkt.parsed['alarm']}")
                logger.info(f"결과 값 in client={self._folding_servo.alarm}")
                
                # TODO 알람 뭐냐에 따라 동작 해야 할지 말아야 할지 그런것도 있지 않을까.. 확인 후 구현 필요할 듯..
                if not self._folding_servo.alarm == "AL.  " and not self._folding_servo.alarm == "AL.16" and not self._folding_servo.alarm == "AL.99": return False
            else:
                logger.error(f"알람 번호 읽기 실패: "
                            f"R={pkt.result_ready}, S={pkt.comm_success}, "
                            f"T={pkt.comm_timeout}, C={pkt.comm_error}, E={pkt.comm_exception}")
                return False
            
        # TODO 필요 시 토크 읽기 추가 : "토크 읽기", self._folding_servo.read_torque

        # 입력 상태 읽기 테스트                
        logger.info(f"입력 상태 읽기 요청")
        self._folding_servo.read_soft_input_device_state()
        result = wait_for_result(self._folding_servo, timeout=2.0)
        if not result:
            logger.error(f"입력 상태 읽기 실패: wait_for_result 타임아웃")
            return False
        else:
            pkt = self._folding_servo.packet_state
            if pkt.result_ready and pkt.parsing_success:
                logger.info(f"입력 상태 읽기 성공: 응답={pkt.response}")
                logger.info(f"결과 값 in packet_state={pkt.parsed['soft_input_device_state']}")
                logger.info(f"결과 값 in client={self._folding_servo.input_device_state}")
                self._folding_servo.set_input_device_control_data_for_recv_value()  # 디바이스에 설정되어있는 값에서부터 수정하여 제어하려고
            else:
                logger.error(f"입력 상태 읽기 실패: "
                            f"R={pkt.result_ready}, S={pkt.comm_success}, "
                            f"T={pkt.comm_timeout}, C={pkt.comm_error}, E={pkt.comm_exception}")
                return False
        return True
    
    def _ready_folding_servo(self) -> bool:
        logger.debug("ready folding servo...")

        if not self._folding_servo:
            return False

        def wait_for_result(client: MRJ4Client, timeout: float = 2.0) -> bool:
            """
            현재 송신한 패킷에 대한 응답이 올 때까지 동기식으로 기다린다.
            성공/실패 여부를 PacketState로 반환.
            """
            start = time.time()
            while time.time() - start < timeout:
                time.sleep(0.01)
                if not client.is_comm_complete():
                    continue
                else:
                    client.try_update_from_comm()
                    return True
            return False
        
        # 입력 상태 읽어서 client 내부에 업데이트하고, 그 값으로부터 수정하면서 제어하기
        logger.info(f"입력 상태 읽기 요청")
        self._folding_servo.read_soft_input_device_state()
        result = wait_for_result(self._folding_servo, timeout=2.0)
        if not result:
            logger.error(f"입력 상태 읽기 실패: wait_for_result 타임아웃")
            return False
        else:
            pkt = self._folding_servo.packet_state
            if pkt.result_ready and pkt.parsing_success:
                logger.info(f"입력 상태 읽기 성공: 응답={pkt.response}")
                logger.info(f"결과 값 in packet_state={pkt.parsed['soft_input_device_state']}")
                logger.info(f"결과 값 in client={self._folding_servo.input_device_state}")
                self._folding_servo.set_input_device_control_data_for_recv_value()  # 디바이스에 설정되어있는 값에서부터 수정하여 제어하려고
                logger.info(f"디바이스 제어값 : {self._folding_servo.input_device_control_data}, 디바이스 상태값 : {self._folding_servo.input_device_state}")
            else:
                logger.error(f"입력 상태 읽기 실패: "
                            f"R={pkt.result_ready}, S={pkt.comm_success}, "
                            f"T={pkt.comm_timeout}, C={pkt.comm_error}, E={pkt.comm_exception}")
                return False
        
        # 서보 정지 요청
        logger.info(f"서보 정지 요청")
        self._folding_servo.run_motor_stop()
        result = wait_for_result(self._folding_servo, timeout=2.0)
        if not result:
            logger.error(f"입력 상태 읽기 실패: wait_for_result 타임아웃")
            return False
        else:
            pkt = self._folding_servo.packet_state
            if not pkt.result_ready or not pkt.parsing_success:
                logger.error(f"서보 정지 요청 실패: "
                            f"R={pkt.result_ready}, S={pkt.comm_success}, "
                            f"T={pkt.comm_timeout}, C={pkt.comm_error}, E={pkt.comm_exception}")
                return False
        
        # 서보 CW set 요청
        logger.info(f"서보 CW limit 해제 요청")
        self._folding_servo.cw_limit_on()
        result = wait_for_result(self._folding_servo, timeout=2.0)
        if not result:
            logger.error(f"서보 CW limit 해제 요청 실패: wait_for_result 타임아웃")
            return False
        else:
            pkt = self._folding_servo.packet_state
            if not pkt.result_ready or not pkt.parsing_success:
                logger.error(f"서보 CW limit 해제 요청 실패: "
                            f"R={pkt.result_ready}, S={pkt.comm_success}, "
                            f"T={pkt.comm_timeout}, C={pkt.comm_error}, E={pkt.comm_exception}")
                return False
        
        # 서보 CCW set 요청
        logger.info(f"서보 CCW limit 해제 요청")
        self._folding_servo.ccw_limit_on()
        result = wait_for_result(self._folding_servo, timeout=2.0)
        if not result:
            logger.error(f"서보 CCW limit 해제 요청 실패: wait_for_result 타임아웃")
            return False
        else:
            pkt = self._folding_servo.packet_state
            if not pkt.result_ready or not pkt.parsing_success:
                logger.error(f"서보 CCW limit 해제 요청 실패: "
                            f"R={pkt.result_ready}, S={pkt.comm_success}, "
                            f"T={pkt.comm_timeout}, C={pkt.comm_error}, E={pkt.comm_exception}")
                return False
        
        # 서보 속도 set 요청
        logger.info(f"서보 속도1 set 요청")
        self._folding_servo.select_speed_2()
        result = wait_for_result(self._folding_servo, timeout=2.0)
        if not result:
            logger.error(f"서보 속도1 set 요청 실패: wait_for_result 타임아웃")
            return False
        else:
            pkt = self._folding_servo.packet_state
            if not pkt.result_ready or not pkt.parsing_success:
                logger.error(f"서보 CCW limit 해제 요청 실패: "
                            f"R={pkt.result_ready}, S={pkt.comm_success}, "
                            f"T={pkt.comm_timeout}, C={pkt.comm_error}, E={pkt.comm_exception}")
                return False
        
        # 입력 상태 읽어서 결과 확인
        logger.info(f"입력 상태 읽기 요청")
        self._folding_servo.read_soft_input_device_state()
        result = wait_for_result(self._folding_servo, timeout=2.0)
        if not result:
            logger.error(f"입력 상태 읽기 실패: wait_for_result 타임아웃")
            return False
        else:
            pkt = self._folding_servo.packet_state
            if pkt.result_ready and pkt.parsing_success:
                logger.info(f"입력 상태 읽기 성공: 응답={pkt.response}")
                logger.info(f"결과 값 in packet_state={pkt.parsed['soft_input_device_state']}")
                logger.info(f"결과 값 in client={self._folding_servo.input_device_state}")

                logger.info(f"디바이스 제어값 : {self._folding_servo.input_device_control_data}, 디바이스 상태값 : {self._folding_servo.input_device_state}")
                if self._folding_servo.input_device_control_data != self._folding_servo.input_device_state:
                    logger.error(f"서보 준비 설정 실패")
                    return False

            else:
                logger.error(f"입력 상태 읽기 실패: "
                            f"R={pkt.result_ready}, S={pkt.comm_success}, "
                            f"T={pkt.comm_timeout}, C={pkt.comm_error}, E={pkt.comm_exception}")
                return False

        return True

    def _test_mast_and_pantilt(self) -> bool:
        logger.debug("Test mast and pantilt...")
        return True

    def _test_metal_sensors(self) -> bool:
        logger.debug("Test metal sensors")

        # TODO 통신해서 값 받아오는게 가능한지 확인..
        return True

    def _test_rc(self) -> bool:
        logger.debug("Test rc...")
        return True

    
    def _deinit_folding_servo(self):
        logger.debug("Deinit folding servo...")
        self._folding_servo.disconnect()

    def _deinit_mast_and_pantilt(self):
        logger.debug("Deinit mast and pantilt...")

    def _deinit_metal_sensors(self):
        logger.debug("Deinit metal sensors...")

    def _deinit_rc(self):
        logger.debug("Deinit rc...")
        self._rc.disconnect()


    def _do_fold(self) -> None:
        logger.debug("Folding the telescopic mast...")
        self._folding_servo.run_motor_cw()  # TODO 조심스럽게 확인

    def _do_unfold(self) -> None:
        logger.debug("Unfolding the telescopic mast...")
        self._folding_servo.run_motor_ccw()  # TODO 조심스럽게 확인

    def _do_stop_folding(self) -> None:
        logger.debug("Stopping folding/unfolding action...")
        self._folding_servo.run_motor_stop()  # TODO 조심스럽게 확인

    def _do_select_fold_speed(self, speed_level: int) -> None:
        logger.debug(f"Setting folding speed level to {speed_level}")
        # if speed_level == 1:
        #     self._folding_servo.select_speed_1()
        # elif speed_level == 2:
        #     self._folding_servo.select_speed_2()
        # elif speed_level == 3:
        #     self._folding_servo.select_speed_3()

    def _do_set_folding_ccw_limit(self) -> None:
        logger.debug(f"Setting folding ccw limit...")
        self._folding_servo.ccw_limit_on()

    def _do_reset_folding_ccw_limit(self) -> None:
        logger.debug(f"Resetting folding ccw limit...")
        self._folding_servo.ccw_limit_off()

    def _do_set_folding_cw_limit(self) -> None:
        logger.debug(f"Setting folding cw limit...")
        self._folding_servo.cw_limit_on()

    def _do_reset_folding_cw_limit(self) -> None:
        logger.debug(f"Resetting folding cw limit...")
        self._folding_servo.cw_limit_off()

    def _do_raise_mast(self) -> None:
        logger.debug("Raising the telescopic mast...")

    def _do_lower_mast(self) -> None:
        logger.debug("Lowering the telescopic mast...")

    def _do_stop_mast(self) -> None:
        logger.debug("Stopping mast action...")

    def _do_tilt_up(self) -> None:
        logger.debug("Tilting up...")

    def _do_tilt_down(self) -> None:
        logger.debug("Tilting down...")

    def _do_tilt_stop(self) -> None:
        logger.debug("Stopping tilt action...")

    def _do_pan_ccw(self) -> None:
        logger.debug("Panning counter-clockwise...")

    def _do_pan_cw(self) -> None:
        logger.debug("Panning clockwise...")

    def _do_pan_stop(self) -> None:
        logger.debug("Stopping pan action...")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mrj4-port", required=True, help="MRJ4 시리얼 포트 경로")
    parser.add_argument("--metal-sensor-addr", default="192.168.127.254:502", help="금속감지 센서 IP:PORT")
    parser.add_argument("--rc-port", required=True, help="RC 시리얼 포트 경로")
    args = parser.parse_args()
    
    ftm = FoldableTelescopicMast(args.mrj4_port, args.metal_sensor_addr, args.rc_port)

    def close_ftm():
        ftm.stop()
        ftm.deinit_devices()

    def signal_handler(sig, frame):
        logger.info(f"signal {sig} caught")

        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # kill, systemctl stop
    atexit.register(close_ftm)

    ftm.init_devices()

    if not ftm.test_devivces():  # 스레드 사용하지 않고 테스트
        logger.error("device test 실패")
        sys.exit(0)

    if not ftm._ready_folding_servo():  # 스레드 사용하지 않고 값 설정
        logger.error("서보 준비 세팅 실패")
        sys.exit(0)

    ftm.start()  # 워커 스레드 시작

    # time.sleep(1)
    # ftm.unfold()
    # time.sleep(1)
    # ftm.fold()
    # time.sleep(1)
    # ftm.stop_folding_action()
    # time.sleep(1)

    # time.sleep(1)
    # ftm.raise_mast()
    # time.sleep(1)
    # ftm.lower_mast()
    # time.sleep(1)
    # ftm.stop_mast_action()
    # time.sleep(1)

    while True:
        time.sleep(1)

    