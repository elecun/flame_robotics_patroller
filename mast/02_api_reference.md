# Python Client 상세 API 명세

- 이 문서는 고객사 앱 개발 기준입니다. 실행 가능한 패키지는 차주 방문하여 검증 후 제공 예정입니다.
- 대상 패키지: `patrol_robot_client`(Client), `patrol_robot_common`(결과 형식·열거형). Python 3.10 이상.
- 상태 응답(`get_status()`)의 형식은 [`03_status_contract.md`](03_status_contract.md)에서 설명한다.

## 1. 공통 규칙

### 1.1 동기 메서드와 비동기 메서드
- 대부분의 메서드는 두 형태로 제공한다.
  - `xxx_async(...)`: 요청을 등록하고 즉시 `concurrent.futures.Future`를 반환한다.
  - `xxx(..., timeout=None)`: 내부에서 `xxx_async(...).result(timeout)`을 호출한다.
- **동기 메서드의 `timeout`은 호출한 스레드가 기다리는 한도**다(초, `None`이면 결과가 날 때까지 기다림).
  - 이 한도를 넘으면 `concurrent.futures.TimeoutError`가 난다.
  - 요청 자체는 취소되지 않고 계속 처리될 수 있다. 결과를 확인하려면 상태를 조회한다.
  - Python 3.10에서 `concurrent.futures.TimeoutError`는 내장 `TimeoutError`와 다른 클래스다. `except TimeoutError`로는 잡히지 않는다.
- Future callback(`add_done_callback`)이 실행되는 스레드는 정해져 있지 않다.
  - Future가 Client 내부 응답 처리 중에 완료되면 주로 Client 내부 스레드에서 실행된다.
  - callback을 등록할 때 Future가 이미 완료돼 있으면 **등록한 스레드에서 즉시** 실행될 수 있다.
  - 따라서 callback 안에서는 오래 걸리는 작업을 하지 않고, GUI 갱신은 **항상** GUI 스레드로 전달한다(queue와 GUI 타이머 등, 개발자 가이드 8절).

### 1.2 요청 응답 대기와 재시도
- 요청마다 응답 대기 한도는 생성자 인자 `request_timeout`(기본 2.0초)이다.
- 이 시간 안에 응답이 없으면 **같은 `request_id`로 한 번 다시 보낸다.** 서비스는 같은 `request_id`의 요청을 다시 실행하지 않고 처음 결과를 돌려준다.
- 재전송 뒤에도 `request_timeout` 안에 응답이 없으면 Future가 `RequestTimeoutError`로 끝난다. 이후 그 요청을 자동으로 다시 보내지 않는다.
- 서비스는 요청을 보낸 뒤 약 `2 × request_timeout`이 지난 요청을 실행하지 않는다(만료 시 `RequestTimeoutError`).
- `RequestTimeoutError`가 나도 요청이 서비스에서 실행됐는지는 알 수 없다. 상태를 조회해 판단하고, **이동 메서드를 다시 호출하지 않는다.** 다시 호출하면 새 명령이다.

### 1.3 인자 검증
Client가 호출 즉시 검사하는 인자와 범위는 아래와 같다. 검사에 걸리면 요청을 보내지 않고 예외가 난다.

| 인자 | 적용 메서드 | 검사 |
|---|---|---|
| `pan_deg`, `tilt_deg`, `height_mm`, `position_mm`, `delta_mm` | `ptu.move_to_angle*`, `mast.move_to_height*`, `erector.move_to*`, `erector.move_by*` | `int`/`float`(bool 제외)가 아니면 `TypeError`, NaN·무한대면 `ValueError`. 운용 범위는 서비스가 검사(`out_of_range` 등) |
| `timeout` | `wait_command`, `wait_command_async` | 위 숫자 검사 + 0 이하이면 `ValueError` |
| `speed_code` | `ptu.set_speed*`(0x00~0x3F), `erector.set_speed*`(0~255) | `int`(bool 제외)가 아니면 `TypeError`, 범위 밖이면 `ValueError` |
| `direction` | `ptu.move_async` | `"UP"`, `"DOWN"`, `"LEFT"`, `"RIGHT"`가 아니면 `ValueError` |
| `command_id` | `get_command_async`, `wait_command*` | str이 아니면 `TypeError`, 빈 문자열이면 `ValueError` |
| `mode` | `set_control_mode*` | `ControlMode`가 아니면 `TypeError` |
| `device`, `action` | `submit_command_async` | `DeviceId`가 아니면 `TypeError` / `action`은 `command_id`와 같은 문자열 검사 |
| `request_timeout`, `heartbeat_interval`, `connect_timeout` | 생성자 | 0 이하이면 `ValueError`. 숫자 타입·유한성 검사는 하지 않으므로 양의 유한한 숫자를 넘긴다 |

- 위 표에 없는 인자는 Client가 검사하지 않는다. 특히 **동기 메서드의 `timeout`과 `close(timeout)`** 은 `Future.result(timeout)` 등에 그대로 전달되므로 `None` 또는 양의 숫자를 넘긴다.
- 서비스가 인자를 해석할 수 없으면 `ValueError`(서비스 오류 코드 `INVALID_PARAMS`)가 난다.
- 장치·정책 조건으로 접수할 수 없는 명령은 예외가 아니라 `CommandReceipt(accepted=False, reason=...)`로 반환한다.

### 1.4 권한과 모드
- 운전 명령(장치 메서드, `reset`)은 **프로그램 모드 + 유효한 제어권 + 비상정지 미고정**에서만 접수된다. 아니면 `wrong_mode`, `control_not_acquired`, `control_lost`, `emergency_stop_active`로 거부된다.
- 제어권이 필요 없는 메서드: `get_status`, `get_command_async`, `wait_command`, `acquire_control`, `emergency_stop`, `release_emergency_stop`, `has_control`, `get_control_status`

### 1.5 Client 종료 이후
- `close()`가 시작된 뒤 **서비스로 요청을 보내는 메서드**(`*_async`와 그 동기 형태, `wait_command*`, `release_control*`, 장치 메서드)를 호출하면 `PatrolRobotClientError("client is closing")` 또는 `("client is closed")`가 난다.
  - 1.3절 인자 검사는 이 검사보다 먼저 한다. 잘못된 인자면 종료 뒤에도 `TypeError`/`ValueError`가 난다.
- 로컬 메서드 `has_control()`, `get_control_status()`, `on_control_lost()`는 종료 뒤에도 예외 없이 동작한다. 종료 뒤의 값은 판단에 쓰지 않는다.
- `close()`는 이미 종료 중이거나 종료된 뒤에 호출하면 아무것도 하지 않고 반환한다.
- 처리 중이던 Future는 `PatrolRobotClientError("client closed")`로 끝난다.

## 2. PatrolRobotClient

`robot`(PatrolRobotClient) 메서드의 정확한 시그니처와 반환형:

```python
# 동기                                                     # 비동기(즉시 Future 반환)
get_status(timeout=None) -> dict                          get_status_async() -> Future[dict]
#   (동기 get_command 없음)                                get_command_async(command_id: str) -> Future[CommandResult | None]
wait_command(command_id: str, timeout: float) -> CommandResult
                                                          wait_command_async(command_id: str, timeout: float) -> Future[CommandResult]
acquire_control(timeout=None) -> dict                     acquire_control_async() -> Future[dict]
release_control(timeout=None) -> dict                     release_control_async() -> Future[dict]
set_control_mode(mode: ControlMode, timeout=None) -> dict set_control_mode_async(mode: ControlMode) -> Future[dict]
emergency_stop(timeout=None) -> EmergencyStopResult       emergency_stop_async() -> Future[EmergencyStopResult]
release_emergency_stop(timeout=None) -> dict              release_emergency_stop_async() -> Future[dict]
reset(timeout=None) -> CommandReceipt                     reset_async() -> Future[CommandReceipt]
#   (동기 submit_command 없음)                             submit_command_async(device: DeviceId, action: str,
#                                                                              params: Mapping | None = None) -> Future[CommandReceipt]
has_control() -> bool                                     # 로컬, 비동기 형태 없음
get_control_status() -> dict                              # 로컬, 비동기 형태 없음
on_control_lost(callback: Callable[[str], None]) -> None  # 로컬, callback 등록
close(timeout: float = 3.0) -> None
# 컨텍스트 관리자: with PatrolRobotClient(...) as robot:  (블록 종료 시 close())
```

- `timeout=None`은 동기 메서드의 호출 스레드 대기 한도(1.1절)다. `wait_command`의 `timeout`은 의미가 다르다(4.3절).
- 반환 dict의 키는 각 절의 표에 있다.

### 2.1 생성자

```python
PatrolRobotClient(
    endpoint: str = "tcp://127.0.0.1:5590",
    *,
    acquire_control: bool = False,
    request_timeout: float = 2.0,
    heartbeat_interval: float = 1.0,
    connect_timeout: float = 3.0,
    context: zmq.Context | None = None,
)
```

| 인자 | 의미 |
|---|---|
| endpoint | 서비스 API 주소 |
| acquire_control | True이면 생성 직후 제어권을 획득한다. 거부되면 Client를 닫고 `PatrolRobotClientError("control not acquired: <reason>")`를 낸다 |
| request_timeout | 요청별 응답 대기 한도(초, 1.2절) |
| heartbeat_interval | 제어권 보유 중 heartbeat 전송 간격(초). 서비스의 제어권 만료 시간(현재 설정 3초)보다 짧아야 한다 |
| connect_timeout | 생성 시 연결 확인 요청 **한 번**의 응답 대기 한도(초). 재전송을 포함한 실제 대기는 아래 참고 |
| context | 공유할 ZMQ context. 보통 지정하지 않는다 |

- 세 시간 인자 중 하나라도 0 이하이면 `ValueError`가 난다(1.3절).
- 생성자는 반환 전에 아래 순서로 서비스와 통신하며, 그동안 호출 스레드를 막는다. **GUI에서는 화면 스레드 밖에서 생성한다.**
  1. **연결 확인(HELLO)**
     - `connect_timeout`은 HELLO 요청 한 번의 응답 대기 한도다. 응답이 없으면 같은 요청을 한 번 다시 보낸다(1.2절과 같은 규칙).
     - 두 번 모두 응답이 없으면 약 `2 × connect_timeout`(기본 약 6초) 뒤 `PatrolRobotServiceUnavailableError`가 난다(내부 `RequestTimeoutError`를 바꿔 낸다).
     - 생성자는 HELLO 결과를 최대 `2 × connect_timeout + 1`초(기본 7초) 기다린다. 내부 처리가 지연돼 이 한도를 넘으면 `concurrent.futures.TimeoutError`가 그대로 난다. 이 한도도 실시간 보장은 아니다.
     - 서비스와 프로토콜 버전이 맞지 않으면 `ProtocolVersionError`가 난다.
  2. **제어권 획득(`acquire_control=True`일 때만)**
     - 연결 확인 뒤 `acquire_control()`을 한 번 호출하고 결과를 기다린다. 이 대기는 1.2절 요청 규칙을 따르므로 응답이 없으면 약 `2 × request_timeout`(기본 약 4초) 뒤 `RequestTimeoutError`가 그대로 난다.
     - 거부되면 Client를 닫고 `PatrolRobotClientError("control not acquired: <reason>")`를 낸다.
  - 따라서 기본값에서 생성자는 서비스 무응답 시 약 6~7초, `acquire_control=True`이고 획득 요청이 무응답이면 연결 확인 시간에 약 4초가 더해질 수 있다.
- 생성자에서 예외가 나면 Client 객체를 얻지 못한다. 원인을 해결한 뒤 새 Client를 생성한다.
- `with PatrolRobotClient() as robot:` 형태로 쓸 수 있다. 블록을 나가면 `close()`가 호출된다.
- 속성 `robot.ptu`, `robot.mast`, `robot.erector`로 장치별 API(3절)를 쓴다.

### 2.2 상태·명령 조회

| 메서드 | 반환 | 설명 |
|---|---|---|
| `get_status_async()` / `get_status(timeout=None)` | `dict` | 상태 전체(03_status_contract.md). 제어권 불필요 |
| `get_command_async(command_id: str)` | Future → `CommandResult` 또는 `None` | 명령 처리 상태 조회. 서비스가 모르는 ID면 `None`. 동기 형태는 없다 |
| `wait_command_async(command_id: str, timeout: float)` | Future → `CommandResult` | 4.3절 |
| `wait_command(command_id: str, timeout: float)` | `CommandResult` | 4.3절 |

- `command_id`가 str이 아니면 `TypeError`, 빈 문자열이면 `ValueError`가 난다.
- `timeout`은 양수여야 하며, 아니면 `ValueError`가 난다.

### 2.3 제어권

| 메서드 | 반환 |
|---|---|
| `acquire_control_async()` / `acquire_control(timeout=None)` | `{"granted": bool, "lease_token": str \| None, "reason": str \| None, "message": str}` |
| `release_control_async()` / `release_control(timeout=None)` | `{"released": bool, "reason": str \| None, "cancelled_command_ids": [str], "stop": {장치: StopAttempt dict} \| None, "control_mode": str \| None}` |
| `has_control()` | `bool`: Client가 제어권 토큰을 가지고 있는가(서비스 상태와 다를 수 있음) |
| `get_control_status()` | `{"has_control": bool, "client_instance_id": str, "service_instance_id": str \| None}` |
| `on_control_lost(callback)` | 제어권 상실 시 `callback(reason: str)` 호출 |

- `acquire_control`
  - 다른 Client가 제어권을 가지고 있으면 `granted=False`, `reason="control_already_acquired"`이다.
  - 성공하면 Client가 heartbeat를 자동으로 보낸다.
  - 제어권만으로 프로그램 모드가 되지는 않는다(`set_control_mode`).
- `release_control`
  - 그 제어권으로 접수된 대기 중 요청을 모두 취소한다(`cancelled_command_ids`).
  - 프로그램 모드였으면 서비스가 세 장치에 STOP을 시도하고(`stop`), `MANUAL` 잠금 상태로 전환한다.
  - 수동 모드였으면 수동 상태와 리모콘 동작을 그대로 두고 `stop`은 `None`이다.
  - 응답은 STOP의 로컬 전송 결과가 나오거나 전송 대기 상한이 지난 뒤 반환된다. 실제 정지까지 기다리지 않는다.
- `on_control_lost`
  - reason은 `"lease_expired"`(heartbeat가 거부됨) 또는 `"service_restarted"`(서비스 재시작 감지)다.
  - callback은 Client 내부 스레드에서 실행된다. callback 안의 예외는 무시된다. GUI 갱신은 GUI 스레드로 전달한다.

### 2.4 모드

| 메서드 | 반환 |
|---|---|
| `set_control_mode_async(mode: ControlMode)` / `set_control_mode(mode, timeout=None)` | `{"accepted": bool, "control_mode": str, "reason": str \| None, "message": str, "stop": {장치: StopAttempt dict} \| None}` |

- `mode`가 `ControlMode`가 아니면 `TypeError`가 난다.
- 유효한 제어권이 필요하다. 없으면 `accepted=False`이고 상태를 바꾸지 않는다.
- 거부 사유
  - `emergency_stop_active`: 비상정지 고정 중
  - `nport_link_unavailable`: NPort가 끊긴 상태에서 수동 전환
  - `not_ready`: 서비스 종료 중
- 이미 요청한 모드이면 `accepted=True`, `message="already in requested mode"`, `stop=None`이다.
- 모드가 바뀌면 이전 모드의 진행 중인 동작을 끝내고 세 장치에 STOP을 시도한다(`stop`). 수동으로 전환하면 리모콘 활성화 상태가 `LOCKED`로 초기화된다.

### 2.5 비상정지

| 메서드 | 반환 |
|---|---|
| `emergency_stop_async()` / `emergency_stop(timeout=None)` | `EmergencyStopResult` |
| `release_emergency_stop_async()` / `release_emergency_stop(timeout=None)` | `{"released": bool, "control_mode": str, "was_latched": bool}` |

- 두 메서드 모두 제어 모드와 제어권에 관계없이 호출할 수 있다. 반복 호출해도 되지만 결과가 매번 같지는 않다.
  - `emergency_stop`을 다시 호출하면 고정은 그대로 유지(`latched=True`)되고, **호출할 때마다 세 장치에 STOP을 다시 시도**한다. 그래서 호출마다 `tx_result` 등 StopAttempt 내용이 달라질 수 있다.
  - `release_emergency_stop`을 반복 호출하면 첫 해제 뒤에는 `was_latched=False`가 반환된다. `was_latched`는 그 호출 시점에 고정이 있었는지를 나타낸다.
- `emergency_stop`
  - 새 동작을 막는 고정(latch)을 걸고, 대기 중인 이동을 취소하고, 진행 중인 작업을 `stopped`로 끝내고, 세 장치에 STOP을 각각 시도한다.
  - 응답은 모든 장치의 로컬 전송 결과가 나오거나 전송 대기 상한(현재 설정 1.0초)이 지난 뒤 반환된다. 실제 정지를 기다리지 않는다.
- `release_emergency_stop`
  - 고정만 푼다. 이전 동작을 다시 시작하지 않는다.
  - 고정 전에 프로그램 모드였고 제어권이 유효하면 프로그램 모드로, 그렇지 않으면 수동 잠금으로 돌아간다.
  - 고정이 없을 때 호출하면 `released=True`, `was_latched=False`다.

### 2.6 RESET

| 메서드 | 반환 |
|---|---|
| `reset_async()` / `reset(timeout=None)` | `CommandReceipt` |

- 마스트 Home → PTU Home → Erector Stow를 차례로 실행하는 작업 명령이다. 이미 Home인 단계는 건너뛴다.
- 한 단계가 실패하면 이후 단계로 진행하지 않고 그 결과로 끝난다.
- 실행 중에 다시 요청하면 새 명령을 만들지 않고 같은 `command_id`를 `message="already running"`과 함께 돌려준다.
- 단순한 오류 해제 기능이 아니다. 여러 장치가 움직인다.

### 2.7 범용 명령 접수

| 메서드 | 반환 |
|---|---|
| `submit_command_async(device: DeviceId, action: str, params: Mapping \| None = None)` | Future → `CommandReceipt` |

장치별 메서드(3절)가 내부에서 쓰는 저수준 메서드다. 앱은 3절 메서드를 사용한다. `device`가 `DeviceId`가 아니면 `TypeError`가 난다.

### 2.8 종료

| 메서드 | 설명 |
|---|---|
| `close(timeout: float = 3.0)` | 새 요청을 막고, 제어권이 있으면 반환을 한 번 시도한 뒤 내부 스레드를 정리한다 |

- 일반 스레드에서 호출하면 반환 결과를 최대 `timeout` 동안 기다린 뒤 정리한다.
- Client 내부 스레드에서 실행 중인 callback(예: `on_control_lost`, 응답 처리 중 완료된 Future의 callback) 안에서 호출하면 기다리지 않고, 반환 요청이 끝나는 대로 스스로 정리된다. 다른 스레드에서 실행된 callback이면 일반 스레드와 같다.
- 반환에 실패해도 서비스의 heartbeat 만료가 같은 절차(프로그램 모드면 STOP과 수동 잠금)를 수행한다.
- 두 번 이상 호출해도 된다.

## 3. 장치별 API

모든 운전 메서드는 1.4절 권한·모드가 필요하다. 반환은 `CommandReceipt`이며 **접수 결과일 뿐 실제 완료가 아니다.**

- 동기 메서드는 모두 마지막 인자로 `timeout: float | None = None`(1.1절 호출 스레드 대기 한도)을 받는다.
- `_async` 메서드는 `timeout`을 받지 않고 즉시 `Future[CommandReceipt]`를 반환한다.
- 각 절 첫머리의 코드 블록이 정확한 시그니처다. 표는 동작 설명이다.

명령 종류:
- **단순 명령**: 연속 이동, STOP, 속도 설정
- **작업 명령**: Home, 목표 위치 이동, Erect/Stow, RESET

결과 의미는 4절에서 설명한다.

### 3.1 PTU(`robot.ptu`)

```python
# 동기: CommandReceipt 반환              # 비동기: Future[CommandReceipt] 반환
up(timeout=None)                         up_async()
down(timeout=None)                       down_async()
left(timeout=None)                       left_async()
right(timeout=None)                      right_async()
                                         move_async(direction: str)   # 동기 move()는 없다
stop(timeout=None)                       stop_async()
home(timeout=None)                       home_async()
move_to_angle(pan_deg, tilt_deg, timeout=None)
                                         move_to_angle_async(pan_deg: float, tilt_deg: float)
set_speed(speed_code, timeout=None)      set_speed_async(speed_code: int)
```

- `move_async(direction)`은 방향을 인자로 받는 범용 비동기 메서드다. **동기 `move()`는 없다.** 동기로 쓰려면 `up()` 등을 쓰거나 `move_async(d).result(timeout)`을 호출한다.

| 메서드 | 인자 | 종류 | 동작 |
|---|---|---|---|
| `up()`, `down()`, `left()`, `right()` | - | 단순 | 해당 방향으로 연속 이동. 다른 축은 멈춘다. STOP 또는 다른 명령까지 계속 |
| `move_async(direction)` | `"UP"`, `"DOWN"`, `"LEFT"`, `"RIGHT"` | 단순 | 위와 같음. 다른 값이면 `ValueError` |
| `stop()` | - | 단순 | 정지 명령 전송. 통신 경로가 READY가 아니어도 시도한다 |
| `set_speed(speed_code)` | int 0x00~0x3F | 단순 | 이후 연속 이동에 쓸 속도. **장치로 바로 보내지 않고** 다음 방향 이동 명령부터 적용된다. 0x00은 정지 속도, 0xFF 터보는 지원하지 않는다. int가 아니면 `TypeError`, 범위 밖이면 `ValueError` |
| `home()` | - | 작업 | Pan 0°, Tilt 0°로 이동하고 도착을 확인한다 |
| `move_to_angle(pan_deg, tilt_deg)` | 각도(°) | 작업 | 절대 각도 이동. Pan은 360°로 나눈 나머지로 처리한다. Tilt는 −60.36° ~ 59.57°(현재 설정) 밖이면 `out_of_range` |

- 운전 준비: 상태의 `devices.ptu.readiness.motion`/`speed`/`stop`
- Pan에는 확인된 하드웨어 이동 제한이 없다.

### 3.2 마스트(`robot.mast`)

```python
# 동기: CommandReceipt 반환              # 비동기: Future[CommandReceipt] 반환
extend(timeout=None)                     extend_async()
retract(timeout=None)                    retract_async()
stop(timeout=None)                       stop_async()
home(timeout=None)                       home_async()
move_to_height(height_mm, timeout=None)  move_to_height_async(height_mm: float)
```

| 메서드 | 인자 | 종류 | 동작 |
|---|---|---|---|
| `extend()`, `retract()` | - | 단순 | 연속 상승·하강. STOP 또는 다른 명령까지 계속. **높이 상·하한과 카운터 상태를 검사하지 않는다**(자동 리밋 보호 아님) |
| `stop()` | - | 단순 | 정지 명령 전송 |
| `home()` | - | 작업 | 운용 하한까지 하강하고 도착을 확인한다 |
| `move_to_height(height_mm)` | 높이(mm) | 작업 | 목표 높이 이동. 운용 범위(현재 2900~9100mm) 밖이면 `out_of_range` |

- 목표 이동·Home에는 디지털카운터 높이가 유효·최신이어야 하고, 운용 하한·상한·허용오차 설정이 필요하다(readiness `target_motion`).
- 운전 준비: `devices.mast.readiness.manual_motion`/`target_motion`/`stop`

### 3.3 Erector(`robot.erector`)

```python
# 동기: CommandReceipt 반환              # 비동기: Future[CommandReceipt] 반환
fold(timeout=None)                       fold_async()
unfold(timeout=None)                     unfold_async()
erect(timeout=None)                      erect_async()
stow(timeout=None)                       stow_async()
stop(timeout=None)                       stop_async()
move_to(position_mm, timeout=None)       move_to_async(position_mm: float)
move_by(delta_mm, timeout=None)          move_by_async(delta_mm: float)
set_speed(speed_code, timeout=None)      set_speed_async(speed_code: int)
```

| 메서드 | 인자 | 종류 | 동작 |
|---|---|---|---|
| `fold()`, `unfold()` | - | 단순 | 연속 접기·펴기. STOP 또는 펌웨어 리밋까지 |
| `stop()` | - | 단순 | 정지 명령 전송 |
| `set_speed(speed_code)` | int 0~255 | 단순 | 펌웨어 속도 설정 명령을 장치로 보낸다. 0이면 구동 출력 0. int가 아니면 `TypeError`, 범위 밖이면 `ValueError` |
| `erect()`, `stow()` | - | 작업 | 펌웨어에 설정된 Erect·Stow 위치로 이동하고 도착 보고를 확인한다 |
| `move_to(position_mm)` | 위치(mm) | 작업 | 절대 위치 이동. 펌웨어 소프트 리밋 밖이면 리밋에서 멈추고 `limit_reached`로 끝난다. 표현할 수 없는 값(음수 등)은 `out_of_range` |
| `move_by(delta_mm)` | 이동량(mm) | 작업 | 접수 시점 위치 + delta를 절대 목표로 바꿔 이동한다. 접수 시 위치가 유효·최신이어야 한다 |

- 운전 준비: `devices.erector.readiness.motion`/`speed`/`stop`

## 4. 결과 형식과 명령 상태

### 4.1 CommandReceipt(접수 결과)

| 필드 | 타입 | 의미 |
|---|---|---|
| accepted | bool | 접수 여부 |
| command_id | str \| None | 접수된 명령 ID(거부 시 None) |
| reason | RejectReason \| None | 거부 사유(접수 시 None) |
| message | str | 보조 설명(표시용) |

### 4.2 CommandResult와 CommandState

| 필드 | 타입 | 의미 |
|---|---|---|
| command_id | str | 명령 ID |
| state | CommandState | 처리 상태 |
| reason | RejectReason \| None | 종료 사유(있을 때) |
| message | str | 보조 설명(표시용) |
| updated_at | str | 마지막 갱신 시각(UTC ISO 8601) |

`CommandState`는 **서비스의 명령 처리 진행 상태**다. 장치가 실제로 움직이거나 멈췄다는 보장이 아니다.

| 값 | 의미 | 종료 여부 |
|---|---|---|
| `accepted` | 접수됨 | 진행 중 |
| `running` | 처리 중 | 진행 중 |
| `completed` | 그 명령의 처리 완료 조건 충족. **성공은 이것뿐** | 종료 |
| `stopped` | STOP·비상정지·모드 전환으로 종료 | 종료 |
| `limit_reached` | 리밋 조건으로 종료 | 종료 |
| `cancelled` | 실행 전 취소(제어권 반환·만료, 다른 명령으로 대체 등) | 종료 |
| `failed` | 송신 실패·장치 오류 등으로 실패 | 종료 |

**명령 종류별 `completed` 조건**

| 종류 | completed 조건 | 보장하지 않는 것 |
|---|---|---|
| 단순 명령(연속 이동, STOP) | 서비스가 장치로 명령 송신 처리를 끝냄 | 장치 수신, 실제 이동·정지 |
| PTU `set_speed` | 서비스가 속도값을 반영함(다음 방향 이동부터 적용) | 장치 송신(이 명령은 장치로 보내지 않음) |
| Erector `set_speed` | 서비스가 설정 명령 송신 처리를 끝냄 | 펌웨어 적용 확인 |
| 마스트 `move_to_height` | 목표에 가까워지면 STOP을 보내고, 정지 후 높이가 목표 허용오차 안. 시작할 때 이미 허용오차 안이면 바로 completed. 남은 거리가 선행 STOP 거리(현재 50mm) 이하면 접수 거부(`out_of_range`) | - |
| 마스트 `home` | 하한 + 허용오차까지 하강 후 STOP, 정지 확인 | - |
| PTU `home`, `move_to_angle` | 정지 후 두 축이 목표 허용오차 안 | - |
| Erector `erect`, `stow` | 펌웨어가 정지와 Erect/Home 도착을 보고 | - |
| Erector `move_to`, `move_by` | 펌웨어 정지 보고 후 위치가 목표 허용오차 안. stall이면 failed, 목표 밖 소프트 리밋 정지는 limit_reached | - |
| RESET | 모든 단계가 completed(이미 Home인 단계는 건너뜀) | - |

**목표 허용오차**(완료 판정 기준)는 서비스 설치 설정값이다. 고객사 앱이 바꾸는 값이 아니며, 현장 설정은 납품 시 함께 전달한다.

| 장치 | 설정 항목 | 의미 | 현재 값 |
|---|---|---|---|
| PTU | `devices.ptu.position_tolerance_deg` | 정지 후 Pan·Tilt 각각의 목표와 실제 각도 차이 허용값(°). Home(0°, 0°)에도 같이 적용 | 0.02°(초기값, 실측 최대 오차 0.010°) |
| 마스트 | `devices.mast.target_tolerance_mm` | 정지 후 목표 높이와 실제 높이 차이 허용값(mm). Home은 운용 하한 + 이 값 이하 | 실물 시험 후 확정. 설정 전에는 목표 이동·Home이 `target_motion` 미준비로 거부된다 |
| Erector | `devices.erector.target_tolerance_mm` | `move_to`/`move_by` 정지 후 위치 차이 허용값(mm) | ±5mm(임시값, 실물 검증 예정) |

- 마스트 운용 범위 2900~9100mm(`devices.mast.lower_limit_mm`/`upper_limit_mm`)와 선행 STOP 거리 50mm는 확정값이다. 위 표의 "초기값"·"임시값"은 실물 시험으로 바뀔 수 있으며, 바뀌면 설정값과 함께 알린다.
- 단순 명령은 처리 완료 후 바로 `completed`가 된다. 이후 STOP이 와도 그 기록은 바뀌지 않는다. 실제 동작·정지는 상태의 `motion`과 위치로 확인한다.
- 동일한 장치의 동일한 Home·Erect·Stow 명령 또는 RESET을 진행 중에 다시 요청하면 새 작업을 시작하지 않고 기존 작업의 `command_id`를 반환한다.
- 서비스는 명령 기록을 일정 개수만 보관한다. 오래된 종료 기록은 정리될 수 있고, 그 ID를 조회하면 `None`이 반환된다(`wait_command`는 `UnknownCommandError`).

### 4.3 wait_command

```python
wait_command_async(command_id: str, timeout: float) -> Future[CommandResult]
wait_command(command_id: str, timeout: float) -> CommandResult
```

동작:
1. 등록 즉시, 이후 약 0.1초 간격으로 명령 상태를 조회한다(`get_command`).
2. 종료 상태(4.2절 표의 "종료")가 오면 그 `CommandResult`로 끝난다. **성공 여부는 `state == completed`로 판단한다.**
3. 서비스가 모르는 ID면 `UnknownCommandError`로 끝난다.
   - 상태 조회 요청 하나가 실패하면(`RequestTimeoutError`, `ServiceRequestError` 등) **그 오류는 `wait_command` 결과로 반환되지 않는다.** 내부에서 무시하고 약 0.1초 뒤 다시 조회한다.
   - 즉 재시도는 두 단계다. (가) 조회 요청 하나는 1.2절 규칙대로 같은 `request_id`로 1회 재전송된다. (나) `wait_command`는 마감까지 새 조회 요청을 반복한다.
   - 통신이 계속 실패하면 마감에 `CommandWaitTimeoutError`로 끝나고, `last_result`는 마지막으로 **성공한** 조회 결과(없으면 None)다.
4. 대기 마감(`등록 시각 + timeout`)이 지나면 `CommandWaitTimeoutError`로 끝난다. `last_result`에 마지막으로 받은 결과(없으면 None)가 들어 있다.
   - 마감 시점에 진행 중인 조회 요청이 있으면 그 요청이 끝날 때(응답 도착 또는 재전송 후 응답 없음)까지 기다린 뒤 판정한다. 그 요청이 실패로 끝나면 위 규칙대로 오류를 반환하지 않고 `CommandWaitTimeoutError`로 끝난다.
   - 그래서 실제 종료는 마감보다 늦을 수 있다. 정상 진행 시 대략 `timeout + 2 × request_timeout`까지다.
   - 그 응답이 종료 상태면 마감이 지났어도 정상 결과를 반환한다.
   - 이 늦어짐에 실시간 상한은 없다. Client 스레드가 callback에 붙잡히거나 시스템이 바쁘면 더 늦어질 수 있다.
5. 동기 `wait_command`는 위 Future를 `timeout + 3 × request_timeout`초 동안 기다린다.
   - 이 3배는 재시도 횟수가 아니라, 진행 중 조회의 응답 대기(최대 약 2배)에 여유를 더한 호출 스레드 대기 한도다. 보장된 상한이 아니다.
   - 그 안에 Future가 끝나지 않으면 `concurrent.futures.TimeoutError`가 난다(`CommandWaitTimeoutError`가 아님). 이때 내부 대기는 남아 있다가 나중에 끝난다.
6. **대기가 만료돼도 서비스는 명령을 자동으로 STOP하거나 취소하지 않는다.** 장치를 멈추려면 `stop()` 또는 `emergency_stop()`을 호출한다.

| 끝나는 방식 | 형태 |
|---|---|
| 종료 상태 도착 | `CommandResult` 반환 |
| 대기 마감 경과(정상 경로) | `CommandWaitTimeoutError`(내장 `TimeoutError` 하위 클래스) |
| 동기 호출 스레드 대기 한도 경과 | `concurrent.futures.TimeoutError` |
| 서비스가 모르는 ID | `UnknownCommandError` |
| Client 종료 | `PatrolRobotClientError("client closed")` |

### 4.4 EmergencyStopResult와 StopAttempt

```
EmergencyStopResult = {latched: bool, ptu: StopAttempt, mast: StopAttempt, erector: StopAttempt}
StopAttempt        = {device: DeviceId, tx_result: TxResult, reason: RejectReason | None,
                      error_code: str | None, error_message: str | None}
```
(dict 형태에서 `error`는 `{"code": str, "message": str}` 또는 `null`이다.)

- `latched=True`는 서비스가 새 동작을 막았다는 뜻이다. 장치가 실제로 멈췄다는 뜻이 아니다.
- `tx_result`: `TX_SUCCESS`(로컬 포트 송신 성공) / `TX_FAILED`. **`TX_SUCCESS`는 장치 수신이나 정지 확인이 아니다.** 실제 정지는 상태 조회의 장치 동작 상태로 확인한다.

| 경우 | tx_result | reason | error |
|---|---|---|---|
| 송신 성공 | TX_SUCCESS | null | null |
| 통신 경로 비활성·주소 미설정 | TX_FAILED | capability_unavailable | null |
| STOP 접수 거부 | TX_FAILED | 거부 사유 | null |
| 대기 상한 안에 송신 결과 없음 | TX_FAILED | null | code `STOP_RESULT_TIMEOUT` |
| 송신 실패(포트 오류 등) | TX_FAILED | 실패 사유 | 오류 코드·메시지 |

`release_control()`과 `set_control_mode()` 응답의 `stop`은 `{"PTU": StopAttempt dict, "MAST": ..., "ERECTOR": ...}`이다.

## 5. 열거형(`patrol_robot_common`)

| 이름 | 값 |
|---|---|
| ControlMode | `PROGRAM`, `MANUAL` |
| DeviceId | `PTU`, `MAST`, `ERECTOR` |
| CommandState | `accepted`, `running`, `completed`, `stopped`, `limit_reached`, `cancelled`, `failed` |
| TxResult | `TX_SUCCESS`, `TX_FAILED` |
| RejectReason | `wrong_mode`, `control_not_acquired`, `control_already_acquired`, `control_lost`, `emergency_stop_active`, `device_offline`, `device_initializing`, `not_ready`, `nport_link_unavailable`, `manual_locked`, `limit_reached`, `out_of_range`, `capability_unavailable`, `queue_full` |

| RejectReason | 뜻 | 앱 대응 |
|---|---|---|
| wrong_mode | 프로그램 모드가 아님 | 모드 확인·전환 |
| control_not_acquired / control_lost | 제어권 없음·상실 | 제어권 다시 획득 |
| control_already_acquired | 다른 Client가 보유 | 보유 Client 확인 |
| emergency_stop_active | 비상정지 고정 중 | 원인 확인 후 해제 |
| device_offline / device_initializing / not_ready / capability_unavailable | 통신·초기화·설정·피드백 조건 불충족 | readiness의 reasons와 같은 문자열. 상태 확인 |
| out_of_range / limit_reached | 목표값·운용 제한 위반 | 목표값 확인 |
| nport_link_unavailable / manual_locked | NPort 연결·수동 활성화 조건 | 현장 상태 확인 |
| queue_full | 요청 과다 | 간격을 두고 재시도 |

## 6. 예외(`patrol_robot_client`)

| 예외 | 발생 | 앱 대응 |
|---|---|---|
| `PatrolRobotClientError` | 모든 Client 예외의 기반. Client 종료 후 호출, 생성 시 제어권 획득 실패 등 | - |
| `PatrolRobotServiceUnavailableError` | 생성 시 서비스 무응답 | 서비스 실행·주소 확인 |
| `RequestTimeoutError` | 요청 응답 없음(재시도 후), 서비스에서 요청 만료 | 실행 여부 불명. 상태 조회 후 판단, 이동 재호출 금지 |
| `CommandWaitTimeoutError` | `wait_command` 대기 마감(4.3절). `command_id`, `last_result` 속성 | 명령은 계속 진행 중일 수 있음 |
| `UnknownCommandError` | 서비스가 모르는 command_id. `command_id` 속성 | 서비스 재시작·기록 정리 여부 확인 |
| `ProtocolVersionError` | Client와 서비스 프로토콜 버전 불일치 | 같은 배포본의 Client·서비스 사용 |
| `ResponseFormatError` | 서비스 응답을 해석할 수 없음 | 오류 정보 보존, 버전 확인 |
| `ServiceRequestError` | 서비스가 요청을 처리하지 못함. `code`, `message` 속성 | 오류 정보 보존 |
| `TypeError` / `ValueError` | 잘못된 인자(호출 즉시 또는 서비스 `INVALID_PARAMS`) | 인자 수정 |
| `concurrent.futures.TimeoutError` | 동기 메서드의 호출 스레드 대기 한도 초과 | 요청은 계속 처리될 수 있음. 상태 확인 |
