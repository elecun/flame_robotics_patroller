"""
Base interface for robot device components with ZPipe integration support.
"""
from abc import ABC, abstractmethod
from typing import Optional, Any

class BaseDevice(ABC):
    def __init__(self, name: str, enable: bool = True, status_monitor: Optional[Any] = None):
        self.name = name
        self.is_connected = False
        self.enable = enable
        self.zpipe_context: Optional[Any] = None
        if isinstance(status_monitor, str):
            self.status_monitor = [s.strip() for s in status_monitor.split(",") if s.strip()]
        elif isinstance(status_monitor, (list, tuple)):
            self.status_monitor = [str(s).strip() for s in status_monitor if str(s).strip()]
        else:
            self.status_monitor = []

    def set_zpipe_context(self, zpipe_ctx: Any):
        """Set the ZPipe context instance for inter-module communication."""
        self.zpipe_context = zpipe_ctx

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self) -> bool:
        pass

    @abstractmethod
    def get_status(self) -> dict:
        pass
