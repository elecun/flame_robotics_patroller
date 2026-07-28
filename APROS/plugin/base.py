"""
APROS Plugin Base class for modular algorithm components.
"""
from abc import ABC, abstractmethod

class BasePlugin(ABC):
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def initialize(self, config: dict) -> bool:
        pass

    @abstractmethod
    def process(self, data: dict) -> dict:
        pass
