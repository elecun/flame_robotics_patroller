"""
APROS Plugin Base class for modular algorithm components with ZPipe integration support.
"""
from abc import ABC, abstractmethod
from typing import Optional, Any

class BasePlugin(ABC):
    def __init__(self, name: str):
        self.name = name
        self.zpipe_context: Optional[Any] = None

    def set_zpipe_context(self, zpipe_ctx: Any):
        """Set the ZPipe context instance for inter-module communication."""
        self.zpipe_context = zpipe_ctx

    @abstractmethod
    def initialize(self, config: dict) -> bool:
        pass

    @abstractmethod
    def process(self, data: dict) -> dict:
        pass
