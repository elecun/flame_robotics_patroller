"""고객사용 Python Client 패키지. (04)

```python
from patrol_robot_client import PatrolRobotClient

with PatrolRobotClient(acquire_control=True) as robot:
    robot.set_control_mode(ControlMode.PROGRAM)
    receipt = robot.mast.move_to_height(5000)
    if receipt.accepted:
        result = robot.wait_command(receipt.command_id, timeout=30)
```
"""

from patrol_robot_client.client import ErectorApi, MastApi, PatrolRobotClient, PtuApi
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

__all__ = [
    "CommandWaitTimeoutError",
    "ErectorApi",
    "MastApi",
    "PatrolRobotClient",
    "PatrolRobotClientError",
    "PatrolRobotServiceUnavailableError",
    "ProtocolVersionError",
    "PtuApi",
    "RequestTimeoutError",
    "ResponseFormatError",
    "ServiceRequestError",
    "UnknownCommandError",
]
