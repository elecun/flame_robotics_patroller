# [Task] 모바일 로봇 제어 인터페이스

## 1. Objective
- Kavser CAN miniPCIe 를 통해 모바일 로봇의 주행 제어를 하기위한 컴포넌트를 구현

## 2. Context & Constraints
- **Environment:** Ubuntu 22.04 LTS, C++17
- **Dependencies:** canlib


## 3. Reference Files & Scope
- **Read & Analyze:**
  - `flame/include` (컴포넌트를 만들기위해 참조해야 하는 include 파일들)
- **Modify / Create:**
  - `mobility.drive.control.hpp` (신규 작성 또는 수정)
  - `mobility.drive.contro.cc` (신규 작성 또는 수정)
  - `bin/x86_64/patroller/mobility_drive_control.json` (신규 작성 또는 수정)
  - 리펙토링에 의해서 모듈화 될 수 있으면, 추가적인 파일 생성 허용함

## 4. Technical Requirements
- **Functions:**
  - s1_driver.hpp 는 모바일 로봇의 구동 주행을 위한 CAN 프로토콜을 생성해주는 함수들이 정의되어 있다.
  - mmobility.drive.control 컴포넌트는 CAN을 통해 모바일 로봇의 주행을 제어한다.
  - 모바일 로봇은 ackermann steering 방식으로 구동되는데, 모바일 로봇은 폭 1000mm, 길이 2055mm, 높이 640mm이다.
  - 모바일 로봇은 4개의 바퀴로 구성되어 있는데, 앞바퀴와 뒷바퀴간 거리는 1150mm이다.
  - 최소 회전 반경은 2.5m이고, 최대 조향각은 24.7도이다.
  - 로봇의 최대 이동속도는 원래 20km/h이지만, 최대 이동속도 한계는 5km/h로 제한하도록 하고, 사용자가 해당 컴포넌트 프로파일(mobility_drive_control.json)에서 변경할 수 있도록 한다.
  - 모바일 로봇의 고유 파라메터는 mobility_drive_control.json 파일에 parameters 에 정의되어야 하고, 사용자는 그 값을 바꿀 수 있도록 한다.
  - 컴포넌트 외부에서 전달되는 메세지는 onData 함수로 진입되는데, 이 메세지를 parse하여 제어될 수 있다.
  - drive 메세지는 조향각(angle)과 주행속도(velocity)를 메세지로 전달받아, s1_driver.hpp에서 제공하는 실제 구동 명령으로 전달한다.
  - drive 메세지는 onLoop를 통해 주기적으로 모바일 로봇에 보내지도록 한다.
  - onLoop를 통해 주기적으로 전송되는 조향각과 주행속도는 onData에서 수신받는 메세지와 연결되기때문에, 조향각과 주행속도는 별도의 멤버변수로 두고, 그 값을 onLoop 함수와 onData에서 공유되도록 하며, 동시에 값을 접근하여 사용하는것에 문제가 없도록 한다.
  - stop 메세지는 조향각을 0, 주행속도를 0으로 만들도록 한다.
  - forward는 전진 기어, backward는 후진 기어, neutral는 중립 기어로 변경하도록 한다.



## 5. Verification
- 에이전트는 코드를 작성한 후 Antigravity 터미널 환경에서 직접 빌드를 수행하여 컴파일 에러가 없음을 확인해야 한다.
- 빌드는 프로젝트 root 디렉토리에서 make mobility_drive_control.comp로 빌드할 수있다.
- 빌드가 정상적으로 되면 bin/x86_64/patroller 디렉토리에 .comp 확장자의 파일이 생성된다.
- 전체 기능이 구현되고 정상 빌드가 되면, 리팩토링을 1회 수행한다.
