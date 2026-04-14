from PyQt6.QtCore import QObject
from util.logger.console import ConsoleLogger

class Tab3DScan(QObject):
    def __init__(self, main_ui):
        super().__init__()
        self.__console = ConsoleLogger.get_logger()
        self.main_ui = main_ui
        self.__console.debug("Tab3DScan initialized")
