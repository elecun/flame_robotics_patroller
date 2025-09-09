'''
Colorful logging on Console
@author Byunghun Hwang<bh.hwang@iae.re.kr>
'''


import logging
import colorlog


# logger class printing on console with singleton
class ConsoleLogger:
    _logger = None
    
    @classmethod
    def get_logger(cls, level:str = "DEBUG"):
        if cls._logger is None:
            cls._logger = logging.getLogger(__name__)
            cls._logger.setLevel(logging._nameToLevel.get(level.upper(), logging.DEBUG))
            
            formatter = colorlog.ColoredFormatter(
                '[%(asctime)s] %(log_color)s%(levelname)-8s%(reset)s %(white)s%(message)s',
                log_colors={
                    'DEBUG':    'cyan',
                    'INFO':     'green',
                    'WARNING':  'yellow',
                    'ERROR':    'red',
                    'CRITICAL': 'bold_red,bg_white'
                }
            )
            
            ch = logging.StreamHandler()
            ch.setLevel(logging._nameToLevel.get(level.upper(), logging.DEBUG))
            ch.setFormatter(formatter)
            
            cls._logger.addHandler(ch)
        return cls._logger