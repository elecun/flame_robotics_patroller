"""
Base interface for robot device components.
"""
from abc import ABC, abstractmethod

class BaseDevice(ABC):
    def __init__(self, name: str):
        self.name = name
        self.is_connected = False

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self) -> bool:
        pass

    @abstractmethod
    def get_status(self) -> dict:
        pass
