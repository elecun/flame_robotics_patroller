"""설계 문서에 정의된 상태·결과 enum.

값 문자열은 ZMQ 메시지와 로그에 그대로 사용하므로 임의로 바꾸지 않는다.
각 enum 위 주석은 근거 문서 위치다.
"""

from __future__ import annotations

from enum import Enum


class DeviceId(str, Enum):
    """구동 장치. 전체 STOP 대상은 PTU, MAST, ERECTOR다."""

    PTU = "PTU"
    MAST = "MAST"
    ERECTOR = "ERECTOR"


MOTION_DEVICES = (DeviceId.PTU, DeviceId.MAST, DeviceId.ERECTOR)


class CommandOrigin(str, Enum):
    """명령 출처. 모든 queue 경계에서 보존한다. (02 §3, AGENTS.md)"""

    PROGRAM_LEASE = "PROGRAM_LEASE"
    REMOTE = "REMOTE"
    SAFETY = "SAFETY"
    SYSTEM = "SYSTEM"


# 05 §2 상태 축


class ControlMode(str, Enum):
    PROGRAM = "PROGRAM"
    MANUAL = "MANUAL"


class ProgramControl(str, Enum):
    NONE = "NONE"
    ACQUIRED = "ACQUIRED"


class NportLink(str, Enum):
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"


class RemoteActivity(str, Enum):
    RECENT = "RECENT"
    STALE = "STALE"
    UNKNOWN = "UNKNOWN"


class ManualAuthorization(str, Enum):
    LOCKED = "LOCKED"
    ARMING = "ARMING"
    READY = "READY"


class Health(str, Enum):
    NORMAL = "NORMAL"
    DEGRADED = "DEGRADED"
    FAULT = "FAULT"
    EMERGENCY_STOPPED = "EMERGENCY_STOPPED"


class TransportState(str, Enum):
    """통신 경로 상태. 연결만 성공한 상태는 제어 가능 상태가 아니다. (03 §6)"""

    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    INITIALIZING = "INITIALIZING"
    READY = "READY"


class DeviceConnection(str, Enum):
    """개별 장치의 응답 상태. (02 §6 DEGRADED/OFFLINE, 01 §5 PTU INITIALIZING)"""

    UNKNOWN = "UNKNOWN"
    INITIALIZING = "INITIALIZING"
    ONLINE = "ONLINE"
    DEGRADED = "DEGRADED"
    OFFLINE = "OFFLINE"


class MotionState(str, Enum):
    """장치 동작 상태 최소 집합. (01 §5 공통 장치 상태)"""

    INITIALIZING = "INITIALIZING"
    MOVING = "MOVING"
    STOPPED = "STOPPED"
    UNKNOWN = "UNKNOWN"


class MotionStateSource(str, Enum):
    """동작 상태의 근거. (01 §5)"""

    DEVICE_REPORTED = "DEVICE_REPORTED"
    POSITION_INFERRED = "POSITION_INFERRED"
    COMMAND_ASSUMED = "COMMAND_ASSUMED"


class TxResult(str, Enum):
    """로컬 전송 결과. TX_SUCCESS는 write()+flush() 완료만 뜻한다. (03 §8)"""

    TX_SUCCESS = "TX_SUCCESS"
    TX_FAILED = "TX_FAILED"


class RejectReason(str, Enum):
    """명령 거부 사유. (04 §5)

    04 §5 목록 뒤의 세 값은 다른 설계 문서에 정의된 결과를 담기 위해 추가했다.
    """

    WRONG_MODE = "wrong_mode"
    CONTROL_NOT_ACQUIRED = "control_not_acquired"
    CONTROL_ALREADY_ACQUIRED = "control_already_acquired"
    EMERGENCY_STOP_ACTIVE = "emergency_stop_active"
    DEVICE_OFFLINE = "device_offline"
    DEVICE_INITIALIZING = "device_initializing"
    NPORT_LINK_UNAVAILABLE = "nport_link_unavailable"
    LIMIT_REACHED = "limit_reached"
    OUT_OF_RANGE = "out_of_range"
    MANUAL_LOCKED = "manual_locked"
    CAPABILITY_UNAVAILABLE = "capability_unavailable"
    # 05 §5: lease 무효화 후 도착한 요청과 취소된 pending 요청
    CONTROL_LOST = "control_lost"
    # 03 §6: READY가 아닌 경로로 들어온 이동 명령
    NOT_READY = "not_ready"
    # 02 §3: 명령 큐가 가득 참
    QUEUE_FULL = "queue_full"


class CommandState(str, Enum):
    """장시간 명령 상태. (04 §5)"""

    ACCEPTED = "accepted"
    RUNNING = "running"
    COMPLETED = "completed"
    STOPPED = "stopped"
    LIMIT_REACHED = "limit_reached"
    CANCELLED = "cancelled"
    FAILED = "failed"

    @property
    def is_terminal(self) -> bool:
        return self not in (CommandState.ACCEPTED, CommandState.RUNNING)
