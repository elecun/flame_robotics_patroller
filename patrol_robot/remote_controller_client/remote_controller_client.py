from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional, Protocol, Dict, Any, List
import threading
import time
import logging

logger = logging.getLogger("rc.client")
logger.setLevel(logging.ERROR)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.DEBUG)
console_handler.setFormatter(logging.Formatter("[%(name)s] %(levelname)s: %(message)s"))

if not logger.hasHandlers():  # 중복 방지
    logger.addHandler(console_handler)

# ========== 공통 타입 ==========

ByteStr = bytes
Callback = Callable[..., None]
Timestamp = float

class RCError(Exception):
    pass

class ConnectState(Enum):
    DISCONNECTED = auto()
    CONNECTING   = auto()
    CONNECTED    = auto()
    DISCONNECTING= auto()

class Event(Enum):
    CONNECTED     = auto()
    DISCONNECTED  = auto()
    FRAME_RX      = auto()  # 원시 바이트 수신
    PACKET_RX     = auto()  # 파싱 성공 패킷 수신
    ERROR         = auto()

class Transport(Protocol):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def write(self, data: ByteStr) -> int: ...
    def read(self, n: int) -> ByteStr: ...
    def in_waiting(self) -> int: ...

class Parser(Protocol):
    def feed(self, chunk: ByteStr) -> List["Packet"]: ...

# ========== 이벤트 버스 ==========
class EventBus:
    def __init__(self) -> None:
        self._handlers: Dict[Event, List[Callback]] = {e: [] for e in Event}

    def on(self, event: Event, cb: Callback) -> None:
        self._handlers[event].append(cb)

    def emit(self, event: Event, /, **kwargs: Any) -> None:
        for cb in list(self._handlers[event]):
            cb(**kwargs)  # 예외는 raise

# ========== 패킷(논리 오브젝트) ==========
@dataclass
class Packet:
    payload: ByteStr
    timestamp: Timestamp = field(default_factory=time.time)

# ========== 클라이언트 ==========
class RCClient:
    """
    최소 동작 버전:
    - SerialTransport로 연결/해제
    - PacketBuilder로 프레임 생성 후 write
    - StreamParser로 프레임 파싱
    - 이벤트 콜백: CONNECTED / DISCONNECTED / FRAME_RX / PACKET_RX / ERROR
    """

    def __init__(self, transport: Transport, parser: Parser, builder) -> None:
        self.transport = transport
        self.parser = parser
        self.builder = builder
        self.events = EventBus()
        self.state = ConnectState.DISCONNECTED
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = threading.Event()
        self._tx_lock = threading.Lock()

    # sugar
    def on_connected(self, cb: Callback) -> None: self.events.on(Event.CONNECTED, cb)
    def on_disconnected(self, cb: Callback) -> None: self.events.on(Event.DISCONNECTED, cb)
    def on_frame(self, cb: Callback) -> None: self.events.on(Event.FRAME_RX, cb)
    def on_packet(self, cb: Callback) -> None: self.events.on(Event.PACKET_RX, cb)
    def on_error(self, cb: Callback) -> None: self.events.on(Event.ERROR, cb)

    # 연결
    def connect(self) -> None:
        if self.state != ConnectState.DISCONNECTED:
            return
        self.state = ConnectState.CONNECTING
        try:
            self.transport.open()
            self._start_rx_loop()
            self.state = ConnectState.CONNECTED
            self.events.emit(Event.CONNECTED, when=time.time())
        except Exception as e:
            self.state = ConnectState.DISCONNECTED
            self.events.emit(Event.ERROR, when=time.time(), error=e)
            raise

    def disconnect(self) -> None:
        if self.state == ConnectState.DISCONNECTED:
            return
        self.state = ConnectState.DISCONNECTING
        self._stop_rx_loop()
        try:
            self.transport.close()
        finally:
            self.state = ConnectState.DISCONNECTED
            self.events.emit(Event.DISCONNECTED, when=time.time())

    # 송신
    def send_packet(self, frame: ByteStr) -> None:
        """완성된 프레임(bytes)을 그대로 전송 (Thread-safe)"""
        logger.debug(f"send_packet {len(frame)}B: {frame.hex(' ').upper()}")
        with self._tx_lock:
            self.transport.write(frame)

    def send_remote_controller(
        self,
        addr: int,
        cmd1: int,
        cmd2: int,
        data1: int = 0x00,
        data2: int = 0x00,
    ) -> None:
        """RemoteController 빌더로 Pelco-D 패킷 생성 후 전송"""
        frame = self.builder.build(
            addr=addr,
            cmd1=cmd1,
            cmd2=cmd2,
            data1=data1,
            data2=data2,
        )
        self.send_packet(frame)

    # 수신 루프
    def _start_rx_loop(self) -> None:
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_worker, name="rc-rx", daemon=True)
        self._rx_thread.start()

    def _stop_rx_loop(self) -> None:
        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        self._rx_thread = None

    def _rx_worker(self) -> None:
        while not self._rx_stop.is_set():
            try:
                n = self.transport.in_waiting()
                if n <= 0:
                    time.sleep(0.005)
                    continue
                chunk = self.transport.read(n)
                if not chunk:
                    continue
                self.events.emit(Event.FRAME_RX, data=chunk, when=time.time())
                for pk in self.parser.feed(chunk):
                    self.events.emit(Event.PACKET_RX, packet=pk, when=time.time())
            except Exception as e:
                self.events.emit(Event.ERROR, when=time.time(), error=e)
                time.sleep(0.05)
