"""PatrolRobotClient: 고객사용 Python Client. (04)

- ZMQ 송수신, 응답 timeout과 1회 재시도, heartbeat와 명령 완료 조회는 Client 내부 background thread
  하나가 처리한다. 고객사 코드에 asyncio를 요구하지 않는다.
- `*_async` 메서드는 `concurrent.futures.Future`를 즉시 반환한다. callback은 Client thread에서 호출될
  수 있으므로 GUI widget을 직접 바꾸지 말고 GUI thread로 전달해야 한다.
- 동기 메서드는 호출한 thread만 제한시간 동안 기다린다.
- 이동 요청의 결과는 실제 완료가 아니라 접수 결과(`CommandReceipt`)다. 완료는 `wait_command_async()`로 확인한다.
- 응답이 없으면 같은 `request_id`로 한 번 다시 보낸다. 서비스는 같은 요청을 다시 실행하지 않는다.
  다시 실패하면 `RequestTimeoutError`이며, 이후 그 요청을 자동으로 다시 보내지 않는다.
"""

from __future__ import annotations

import math
import queue
import threading
import time
import uuid
from concurrent.futures import Future
from dataclasses import dataclass
from typing import Any, Callable, Mapping

import zmq

from patrol_robot_client.errors import (
    CommandWaitTimeoutError,
    PatrolRobotClientError,
    PatrolRobotServiceUnavailableError,
    ProtocolVersionError,
    RequestTimeoutError,
    ResponseFormatError,
    ServiceRequestError,
    UnknownCommandError,
)
from patrol_robot_common import (
    CommandReceipt,
    CommandResult,
    ControlMode,
    DeviceId,
    EmergencyStopResult,
)
from patrol_robot_common.protocol import (
    DEFAULT_ENDPOINT,
    ApiErrorCode,
    MessageFormatError,
    Method,
    decode_response,
    encode_request,
)

POLL_MS = 10
COMMAND_POLL_INTERVAL_SEC = 0.1

# Client 수명 상태. CLOSING부터는 공개 요청을 받지 않고 내부 제어권 반환만 처리한다.
_OPEN, _CLOSING, _CLOSED = "OPEN", "CLOSING", "CLOSED"


@dataclass
class _Pending:
    method: Method
    params: dict[str, Any]
    request_id: str
    lease_token: str | None
    future: Future
    convert: Callable[[Any], Any]
    timeout: float
    deadline: float = 0.0
    attempts: int = 0
    frame: bytes = b""


@dataclass
class _Waiter:
    command_id: str
    future: Future
    deadline: float
    next_poll: float
    in_flight: bool = False
    last: CommandResult | None = None


class PatrolRobotClient:
    def __init__(
        self,
        endpoint: str = DEFAULT_ENDPOINT,
        *,
        acquire_control: bool = False,
        request_timeout: float = 2.0,
        heartbeat_interval: float = 1.0,
        connect_timeout: float = 3.0,
        context: zmq.Context | None = None,
    ) -> None:
        if request_timeout <= 0 or heartbeat_interval <= 0 or connect_timeout <= 0:
            raise ValueError("timeouts must be positive")
        self.endpoint = endpoint
        self.client_instance_id = uuid.uuid4().hex  # 객체마다 새 값, 재사용하지 않는다
        self._request_timeout = request_timeout
        self._heartbeat_interval = heartbeat_interval
        self._ctx = context or zmq.Context.instance()
        self._outbox: queue.Queue = queue.Queue()
        self._lock = threading.Lock()
        self._lease_token: str | None = None
        self._service_instance_id: str | None = None
        self._state = _OPEN
        self._stop = threading.Event()
        self._control_lost_callbacks: list[Callable[[str], None]] = []
        self._thread = threading.Thread(target=self._run, name="patrol-robot-client", daemon=True)
        self._sock: zmq.Socket | None = None
        self._thread.start()

        self.ptu = PtuApi(self)
        self.mast = MastApi(self)
        self.erector = ErectorApi(self)

        try:
            hello = self._call(Method.HELLO, {}, lambda r: r, timeout=connect_timeout,
                               wait=connect_timeout * 2 + 1)
        except RequestTimeoutError as exc:
            self._shutdown_thread()
            raise PatrolRobotServiceUnavailableError(
                f"Patrol Robot Service did not respond at {endpoint}") from exc
        except Exception:
            self._shutdown_thread()
            raise
        self._service_instance_id = hello["service_instance_id"]
        if acquire_control:
            try:
                result = self.acquire_control()
            except BaseException:
                # 예외 종류는 그대로 전달하고(02 §2.1), 생성 실패로 남는 내부 thread·socket만 정리한다.
                self.close()
                raise
            if not result["granted"]:
                self.close()
                raise PatrolRobotClientError(f"control not acquired: {result['reason']}")

    # ================================================================ 공개 API

    def __enter__(self) -> PatrolRobotClient:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def close(self, timeout: float = 3.0) -> None:
        """종료: 공개 요청 접수를 즉시 멈추고, 제어권이 있으면 반환을 한 번 시도한 뒤 thread를 정리한다.

        - 일반 thread에서 호출하면 반환 결과를 최대 timeout 동안 기다린 뒤 정리한다.
        - Client thread의 callback에서 호출하면 자기 자신을 기다리지 않는다. 반환 요청이 끝나는 대로
          (응답 또는 요청 timeout) Client thread가 스스로 정리된다.
        반환에 실패해도 서비스의 heartbeat 만료가 같은 절차를 수행한다.
        """
        with self._lock:
            if self._state != _OPEN:
                return
            # 이 시점 이후 공개 요청은 등록되지 않는다(이동·모드·상태·wait 모두).
            self._state = _CLOSING
            release = self._enqueue_release_locked() if self._lease_token is not None else None
        on_client_thread = threading.current_thread() is self._thread
        if release is None:
            self._finish_close()
        elif on_client_thread:
            release.add_done_callback(lambda _f: self._finish_close())
        else:
            try:
                release.result(timeout=timeout)
            except Exception:
                pass
            self._finish_close()

    def _finish_close(self) -> None:
        with self._lock:
            if self._state == _CLOSED:
                return
            self._state = _CLOSED
        self._shutdown_thread()

    # --- 상태 ---------------------------------------------------------------------

    def get_status_async(self) -> Future:
        return self._submit(Method.GET_STATUS, {}, lambda r: r)

    def get_status(self, timeout: float | None = None) -> dict[str, Any]:
        return self.get_status_async().result(timeout)

    def get_command_async(self, command_id: str) -> Future:
        _require_str("command_id", command_id)
        return self._submit(Method.GET_COMMAND, {"command_id": command_id}, _to_command_result)

    # --- 제어권 --------------------------------------------------------------------

    def acquire_control_async(self) -> Future:
        def convert(result):
            if result["granted"]:
                with self._lock:
                    self._lease_token = result["lease_token"]
            return result

        return self._submit(Method.ACQUIRE_CONTROL, {}, convert)

    def acquire_control(self, timeout: float | None = None) -> dict[str, Any]:
        return self.acquire_control_async().result(timeout)

    def release_control_async(self) -> Future:
        with self._lock:
            self._require_open_locked()
            return self._enqueue_release_locked()

    def _enqueue_release_locked(self) -> Future:
        """`_lock` 안에서 호출. 현재 lease의 반환 요청을 등록한다(close 내부 경로 포함)."""
        token = self._lease_token

        def convert(result):
            with self._lock:
                if self._lease_token == token:
                    self._lease_token = None
            return result

        future: Future = Future()
        self._outbox.put(("req", _Pending(Method.RELEASE_CONTROL, {}, uuid.uuid4().hex, token,
                                          future, convert, self._request_timeout)))
        return future

    def _require_open_locked(self) -> None:
        if self._state != _OPEN:
            raise PatrolRobotClientError(f"client is {self._state.lower()}")

    def release_control(self, timeout: float | None = None) -> dict[str, Any]:
        return self.release_control_async().result(timeout)

    def has_control(self) -> bool:
        with self._lock:
            return self._lease_token is not None

    def get_control_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "has_control": self._lease_token is not None,
                "client_instance_id": self.client_instance_id,
                "service_instance_id": self._service_instance_id,
            }

    def on_control_lost(self, callback: Callable[[str], None]) -> None:
        """heartbeat 거부나 서비스 재시작으로 제어권을 잃으면 사유 문자열과 함께 호출된다(Client thread)."""
        self._control_lost_callbacks.append(callback)

    def set_control_mode_async(self, mode: ControlMode) -> Future:
        if not isinstance(mode, ControlMode):
            raise TypeError("mode must be ControlMode")
        return self._submit(Method.SET_CONTROL_MODE, {"mode": mode.value}, lambda r: r)

    def set_control_mode(self, mode: ControlMode, timeout: float | None = None) -> dict[str, Any]:
        return self.set_control_mode_async(mode).result(timeout)

    # --- 비상정지 -------------------------------------------------------------------

    def emergency_stop_async(self) -> Future:
        """결과는 `EmergencyStopResult`(04 §7). 제어권·모드와 무관하게 호출할 수 있다."""
        return self._submit(Method.EMERGENCY_STOP, {}, EmergencyStopResult.from_dict)

    def emergency_stop(self, timeout: float | None = None) -> EmergencyStopResult:
        return self.emergency_stop_async().result(timeout)

    def release_emergency_stop_async(self) -> Future:
        return self._submit(Method.RELEASE_EMERGENCY_STOP, {}, lambda r: r)

    def release_emergency_stop(self, timeout: float | None = None) -> dict[str, Any]:
        return self.release_emergency_stop_async().result(timeout)

    # --- 장치 명령 ------------------------------------------------------------------

    def submit_command_async(self, device: DeviceId, action: str,
                             params: Mapping[str, Any] | None = None) -> Future:
        """장치 명령 접수. 결과는 `CommandReceipt`(실제 완료 아님)."""
        if not isinstance(device, DeviceId):
            raise TypeError("device must be DeviceId")
        _require_str("action", action)
        return self._submit(
            Method.SUBMIT_COMMAND,
            {"device": device.value, "action": action, "params": dict(params or {})},
            CommandReceipt.from_dict,
        )

    def reset_async(self) -> Future:
        """RESET(Mast Home → PTU Home → Erector Stow). 결과는 `CommandReceipt`이며 완료는 wait_command로 확인한다.

        이미 실행 중인 RESET이 있으면 같은 command_id를 돌려준다.
        """
        return self._submit(Method.RESET, {}, CommandReceipt.from_dict)

    def reset(self, timeout: float | None = None) -> CommandReceipt:
        return self.reset_async().result(timeout)

    def wait_command_async(self, command_id: str, timeout: float) -> Future:
        """명령이 terminal 상태가 되면 `CommandResult`로 완료되는 Future.

        timeout까지 끝나지 않으면 `CommandWaitTimeoutError`(마지막 상태 포함), 서비스가 모르는 명령이면
        `UnknownCommandError`로 실패한다.
        """
        _require_str("command_id", command_id)
        _require_number("timeout", timeout, positive=True)
        future: Future = Future()
        now = time.monotonic()
        with self._lock:
            # close()와 경쟁해도 CLOSING 이후에는 등록되지 않으므로 Future가 끝나지 않는 일이 없다.
            self._require_open_locked()
            self._outbox.put(("wait", _Waiter(command_id, future, now + timeout, now)))
        return future

    def wait_command(self, command_id: str, timeout: float) -> CommandResult:
        return self.wait_command_async(command_id, timeout).result(timeout + self._request_timeout * 3)

    # ================================================================ 내부: 요청

    def _submit(self, method: Method, params: dict[str, Any], convert: Callable[[Any], Any],
                *, lease_token: str | None = None, timeout: float | None = None) -> Future:
        future: Future = Future()
        with self._lock:
            self._require_open_locked()
            token = lease_token if lease_token is not None else self._lease_token
            pending = _Pending(method, params, uuid.uuid4().hex, token, future, convert,
                               timeout or self._request_timeout)
            # 종료 플래그 확인과 등록을 같은 lock 안에서 해 close 이후 등록을 막는다.
            self._outbox.put(("req", pending))
        return future

    def _call(self, method: Method, params: dict[str, Any], convert, *, timeout: float, wait: float):
        future: Future = Future()
        self._outbox.put(("req", _Pending(method, params, uuid.uuid4().hex, None, future, convert,
                                          timeout)))
        return future.result(wait)

    # ================================================================ 내부: background thread

    def _new_socket(self) -> zmq.Socket:
        sock = self._ctx.socket(zmq.DEALER)
        sock.setsockopt(zmq.LINGER, 0)
        # 연결되지 않은 peer로 메시지를 쌓아 두지 않는다(서비스 재시작 후 과거 요청 전달 방지).
        sock.setsockopt(zmq.IMMEDIATE, 1)
        sock.setsockopt(zmq.RECONNECT_IVL, 100)
        sock.setsockopt(zmq.RECONNECT_IVL_MAX, 1000)
        sock.connect(self.endpoint)
        return sock

    def _run(self) -> None:
        self._sock = self._new_socket()
        pending: dict[str, _Pending] = {}
        waiters: list[_Waiter] = []
        next_heartbeat = time.monotonic() + self._heartbeat_interval
        heartbeat_in_flight = [False]
        try:
            while not self._stop.is_set():
                self._receive(pending)
                self._drain_outbox(pending, waiters)
                now = time.monotonic()
                self._check_timeouts(pending, now)
                self._poll_waiters(waiters, now)
                if now >= next_heartbeat:
                    next_heartbeat = now + self._heartbeat_interval
                    self._send_heartbeat(heartbeat_in_flight)
        finally:
            for p in pending.values():
                if not p.future.done():
                    p.future.set_exception(PatrolRobotClientError("client closed"))
            for w in waiters:
                if not w.future.done():
                    w.future.set_exception(PatrolRobotClientError("client closed"))
            while True:
                try:
                    kind, item = self._outbox.get_nowait()
                except queue.Empty:
                    break
                fut = item.future
                if not fut.done():
                    fut.set_exception(PatrolRobotClientError("client closed"))
            self._sock.close(linger=0)

    def _receive(self, pending: dict[str, _Pending]) -> None:
        if not self._sock.poll(POLL_MS, zmq.POLLIN):
            return
        while True:
            try:
                frame = self._sock.recv(zmq.NOBLOCK)
            except zmq.Again:
                return
            self._on_response(frame, pending)

    def _on_response(self, frame: bytes, pending: dict[str, _Pending]) -> None:
        try:
            data = decode_response(frame)
        except MessageFormatError:
            # 어떤 요청의 응답인지 알 수 없으면 버린다. 해당 요청은 timeout으로 끝난다.
            return
        p = pending.pop(data["request_id"], None)
        if p is None or p.future.done():
            return  # 재시도로 중복 도착한 응답
        self._note_service_instance(data["service_instance_id"])
        if not data["ok"]:
            code, message = data["error"]["code"], data["error"]["message"]
            p.future.set_exception(_error_from_code(code, message))
            return
        try:
            p.future.set_result(p.convert(data.get("result")))
        except PatrolRobotClientError as exc:
            p.future.set_exception(exc)
        except Exception as exc:
            p.future.set_exception(ResponseFormatError(f"cannot interpret {p.method.value} result: {exc}"))

    def _note_service_instance(self, service_instance_id: str) -> None:
        previous = self._service_instance_id
        if previous is not None and previous != service_instance_id:
            self._service_instance_id = service_instance_id
            # 서비스 재시작: 이전 제어권은 승계되지 않는다(05 §5).
            self._lose_control("service_restarted")
        elif previous is None:
            self._service_instance_id = service_instance_id

    def _lose_control(self, reason: str) -> None:
        with self._lock:
            had = self._lease_token is not None
            self._lease_token = None
        if had:
            for callback in list(self._control_lost_callbacks):
                try:
                    callback(reason)
                except Exception:
                    pass

    def _drain_outbox(self, pending: dict[str, _Pending], waiters: list[_Waiter]) -> None:
        while True:
            try:
                kind, item = self._outbox.get_nowait()
            except queue.Empty:
                return
            if kind == "wait":
                waiters.append(item)
                continue
            item.frame = encode_request(
                item.method, item.params, request_id=item.request_id,
                client_instance_id=self.client_instance_id, lease_token=item.lease_token,
                # 응답 대기와 1회 재시도가 모두 끝나면 서비스도 이 요청을 실행하지 않는다.
                expires_at=time.time() + item.timeout * 2,
            )
            pending[item.request_id] = item
            self._send(item)

    def _send(self, p: _Pending) -> None:
        p.attempts += 1
        p.deadline = time.monotonic() + p.timeout
        try:
            self._sock.send(p.frame, zmq.NOBLOCK)
        except zmq.Again:
            pass  # 연결되지 않음: deadline에 재시도 또는 timeout 처리

    def _check_timeouts(self, pending: dict[str, _Pending], now: float) -> None:
        expired = [p for p in pending.values() if now >= p.deadline]
        gave_up = False
        for p in expired:
            if p.future.done():
                pending.pop(p.request_id, None)
            elif p.attempts < 2:
                self._send(p)  # 같은 request_id로 1회 재요청
            else:
                pending.pop(p.request_id, None)
                gave_up = True
                p.future.set_exception(RequestTimeoutError(
                    f"{p.method.value} timed out after {p.attempts} attempts"))
        if gave_up:
            # 최종 timeout 후 소켓을 새로 만들어 남은 송신 대기열을 버린다(포기한 요청의 자동 재전송 방지).
            self._sock.close(linger=0)
            self._sock = self._new_socket()
            # 아직 진행 중인 요청은 응답을 새 소켓으로 받도록 같은 request_id로 다시 보낸다.
            # 서비스는 같은 요청을 다시 실행하지 않으므로 재시도 횟수에 넣지 않는다.
            for p in pending.values():
                try:
                    self._sock.send(p.frame, zmq.NOBLOCK)
                except zmq.Again:
                    pass

    def _send_heartbeat(self, in_flight: list[bool]) -> None:
        with self._lock:
            token = self._lease_token
        if token is None or in_flight[0]:
            return
        in_flight[0] = True
        future: Future = Future()

        def done(f: Future) -> None:
            in_flight[0] = False
            try:
                extended = f.result()["extended"]
            except Exception:
                return  # 통신 오류: 다음 heartbeat에서 다시 시도(서비스가 만료를 판단)
            if not extended:
                with self._lock:
                    still_same = self._lease_token == token
                if still_same:
                    self._lose_control("lease_expired")

        future.add_done_callback(done)
        p = _Pending(Method.HEARTBEAT, {}, uuid.uuid4().hex, token, future, lambda r: r,
                     self._heartbeat_interval)
        self._outbox.put(("req", p))

    def _poll_waiters(self, waiters: list[_Waiter], now: float) -> None:
        for w in list(waiters):
            if w.future.done():
                waiters.remove(w)
                continue
            if now >= w.deadline and not w.in_flight:
                waiters.remove(w)
                w.future.set_exception(CommandWaitTimeoutError(w.command_id, w.last))
                continue
            if w.in_flight or now < w.next_poll:
                continue
            w.in_flight = True
            future: Future = Future()

            def done(f: Future, w=w) -> None:
                w.in_flight = False
                w.next_poll = time.monotonic() + COMMAND_POLL_INTERVAL_SEC
                try:
                    result = f.result()
                except Exception:
                    return  # 일시 오류는 다음 조회에서 다시 시도
                if result is None:
                    w.future.set_exception(UnknownCommandError(w.command_id))
                    return
                w.last = result
                if result.state.is_terminal:
                    w.future.set_result(result)

            future.add_done_callback(done)
            p = _Pending(Method.GET_COMMAND, {"command_id": w.command_id}, uuid.uuid4().hex,
                         None, future, _to_command_result, self._request_timeout)
            self._outbox.put(("req", p))

    def _shutdown_thread(self) -> None:
        self._stop.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(3)


# ================================================================ 장치별 API


class _DeviceApi:
    device: DeviceId

    def __init__(self, client: PatrolRobotClient) -> None:
        self._client = client

    def _send(self, action: str, params: Mapping[str, Any] | None = None) -> Future:
        return self._client.submit_command_async(self.device, action, params)

    @staticmethod
    def _wait(future: Future, timeout: float | None) -> CommandReceipt:
        return future.result(timeout)


_PTU_DIRECTIONS = ("UP", "DOWN", "LEFT", "RIGHT")


class PtuApi(_DeviceApi):
    device = DeviceId.PTU

    def move_async(self, direction: str) -> Future:
        if direction not in _PTU_DIRECTIONS:
            raise ValueError(f"direction must be one of {_PTU_DIRECTIONS}")
        return self._send("ptu.jog", {"direction": direction})

    def up_async(self) -> Future:
        return self.move_async("UP")

    def down_async(self) -> Future:
        return self.move_async("DOWN")

    def left_async(self) -> Future:
        return self.move_async("LEFT")

    def right_async(self) -> Future:
        return self.move_async("RIGHT")

    def stop_async(self) -> Future:
        return self._send("ptu.stop")

    def home_async(self) -> Future:
        return self._send("ptu.home")

    def move_to_angle_async(self, pan_deg: float, tilt_deg: float) -> Future:
        _require_number("pan_deg", pan_deg)
        _require_number("tilt_deg", tilt_deg)
        return self._send("ptu.move_to", {"pan_deg": float(pan_deg), "tilt_deg": float(tilt_deg)})

    def set_speed_async(self, speed_code: int) -> Future:
        """연속 이동 속도 코드 0x00~0x3F(0x00 정지, 0xFF 터보 미지원)."""
        if not isinstance(speed_code, int) or isinstance(speed_code, bool):
            raise TypeError("speed_code must be int")
        if not 0x00 <= speed_code <= 0x3F:
            raise ValueError("speed_code must be 0x00~0x3F")
        return self._send("ptu.set_speed", {"speed_code": speed_code})

    def up(self, timeout=None): return self._wait(self.up_async(), timeout)
    def down(self, timeout=None): return self._wait(self.down_async(), timeout)
    def left(self, timeout=None): return self._wait(self.left_async(), timeout)
    def right(self, timeout=None): return self._wait(self.right_async(), timeout)
    def stop(self, timeout=None): return self._wait(self.stop_async(), timeout)
    def home(self, timeout=None): return self._wait(self.home_async(), timeout)

    def move_to_angle(self, pan_deg, tilt_deg, timeout=None):
        return self._wait(self.move_to_angle_async(pan_deg, tilt_deg), timeout)

    def set_speed(self, speed_code, timeout=None):
        return self._wait(self.set_speed_async(speed_code), timeout)


class MastApi(_DeviceApi):
    device = DeviceId.MAST

    def extend_async(self) -> Future:
        return self._send("mast.extend")

    def retract_async(self) -> Future:
        return self._send("mast.retract")

    def stop_async(self) -> Future:
        return self._send("mast.stop")

    def home_async(self) -> Future:
        return self._send("mast.home")

    def extend(self, timeout=None): return self._wait(self.extend_async(), timeout)
    def retract(self, timeout=None): return self._wait(self.retract_async(), timeout)
    def stop(self, timeout=None): return self._wait(self.stop_async(), timeout)
    def home(self, timeout=None): return self._wait(self.home_async(), timeout)


class ErectorApi(_DeviceApi):
    device = DeviceId.ERECTOR

    def fold_async(self) -> Future:
        return self._send("erector.fold")

    def unfold_async(self) -> Future:
        return self._send("erector.unfold")

    def erect_async(self) -> Future:
        return self._send("erector.erect")

    def stow_async(self) -> Future:
        return self._send("erector.stow")

    def stop_async(self) -> Future:
        return self._send("erector.stop")

    def move_to_async(self, position_mm: float) -> Future:
        _require_number("position_mm", position_mm)
        return self._send("erector.move_to", {"position_mm": float(position_mm)})

    def move_by_async(self, delta_mm: float) -> Future:
        _require_number("delta_mm", delta_mm)
        return self._send("erector.move_by", {"delta_mm": float(delta_mm)})

    def set_speed_async(self, speed_code: int) -> Future:
        """Erector 펌웨어 속도 코드 정수 0~255."""
        if not isinstance(speed_code, int) or isinstance(speed_code, bool):
            raise TypeError("speed_code must be int")
        if not 0 <= speed_code <= 255:
            raise ValueError("speed_code must be 0~255")
        return self._send("erector.set_speed", {"speed_code": speed_code})

    def fold(self, timeout=None): return self._wait(self.fold_async(), timeout)
    def unfold(self, timeout=None): return self._wait(self.unfold_async(), timeout)
    def erect(self, timeout=None): return self._wait(self.erect_async(), timeout)
    def stow(self, timeout=None): return self._wait(self.stow_async(), timeout)
    def stop(self, timeout=None): return self._wait(self.stop_async(), timeout)

    def move_to(self, position_mm, timeout=None):
        return self._wait(self.move_to_async(position_mm), timeout)

    def move_by(self, delta_mm, timeout=None):
        return self._wait(self.move_by_async(delta_mm), timeout)

    def set_speed(self, speed_code, timeout=None):
        return self._wait(self.set_speed_async(speed_code), timeout)


# ================================================================ 보조


def _require_str(name: str, value: Any) -> None:
    if not isinstance(value, str):
        raise TypeError(f"{name} must be str")
    if not value:
        raise ValueError(f"{name} must not be empty")


def _require_number(name: str, value: Any, positive: bool = False) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise TypeError(f"{name} must be a number")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if positive and value <= 0:
        raise ValueError(f"{name} must be > 0")


def _to_command_result(result: Any) -> CommandResult | None:
    return None if result is None else CommandResult.from_dict(result)


def _error_from_code(code: str, message: str) -> Exception:
    if code == ApiErrorCode.PROTOCOL_VERSION_MISMATCH.value:
        return ProtocolVersionError(message)
    if code == ApiErrorCode.INVALID_PARAMS.value:
        return ValueError(message)
    if code == ApiErrorCode.EXPIRED.value:
        return RequestTimeoutError(f"request expired at service: {message}")
    return ServiceRequestError(code, message)

