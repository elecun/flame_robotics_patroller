"""Client 통신 예외. (04 §2)

잘못된 인자는 표준 `TypeError`/`ValueError`를 사용하고, 장치·정책에 따른 거부는 예외가 아니라
`CommandReceipt` 등 구조화된 결과로 반환한다.
"""


class PatrolRobotClientError(Exception):
    """Client 통신 예외의 기반 클래스."""


class PatrolRobotServiceUnavailableError(PatrolRobotClientError):
    """서비스 미실행 또는 연결 실패."""


class RequestTimeoutError(PatrolRobotClientError):
    """제한시간 내 응답 없음(같은 request_id로 1회 재시도 후)."""


class ProtocolVersionError(PatrolRobotClientError):
    """Client와 서비스 API 버전 불일치."""


class ResponseFormatError(PatrolRobotClientError):
    """응답 해석 실패."""


class ServiceRequestError(PatrolRobotClientError):
    """서비스가 요청을 처리하지 못함(BAD_REQUEST, UNKNOWN_METHOD, INTERNAL_ERROR 등)."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


class UnknownCommandError(PatrolRobotClientError):
    """서비스가 command_id를 알지 못함(서비스 재시작 등)."""

    def __init__(self, command_id: str) -> None:
        super().__init__(f"unknown command_id: {command_id}")
        self.command_id = command_id


class CommandWaitTimeoutError(PatrolRobotClientError, TimeoutError):
    """wait_command 제한시간 안에 명령이 끝나지 않음. 마지막으로 조회한 상태를 담는다."""

    def __init__(self, command_id: str, last_result=None) -> None:
        super().__init__(f"command {command_id} did not finish in time")
        self.command_id = command_id
        self.last_result = last_result
