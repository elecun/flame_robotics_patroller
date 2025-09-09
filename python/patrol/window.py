"""
Patroller Operation/Monitoring Window
@author Byunghun Hwang<bh.hwang@iae.re.kr>
"""

try:
    # using PyQt6
    from PyQt6.QtGui import QImage, QPixmap, QCloseEvent, QStandardItem, QStandardItemModel
    from PyQt6.QtWidgets import QApplication, QFrame, QMainWindow, QLabel, QPushButton, QCheckBox, QComboBox, QDialog
    from PyQt6.QtWidgets import QMessageBox, QProgressBar, QFileDialog, QComboBox, QLineEdit, QSlider, QVBoxLayout
    from PyQt6.uic import loadUi
    from PyQt6.QtCore import QObject, Qt, QTimer, QThread, pyqtSignal, QRegularExpression
except ImportError:
    print("PyQt6 is required to run this application.")

import zmq
import os, sys
import pathlib
import json
import threading
import re
import numpy as np
import math
from functools import partial

from util.logger.console import ConsoleLogger
from common.zpipe import AsyncZSocket, ZPipe

class PatrolWindow(QMainWindow):
    def __init__(self, config:dict, zpipe:ZPipe):
        """ initialization """
        super().__init__()
        # initialize
        self.__config = config
        self.__console = ConsoleLogger.get_logger()

        # open ui file
        try:
            if "gui" in config:
                ui_path = pathlib.Path(config["app_path"]) / config["gui"]
                if os.path.isfile(ui_path):
                    loadUi(ui_path, self)
                    self.setWindowTitle(config.get("window_title", "Patrol Robot Operation/Monitoring"))

                    if config.get("fullscreen", False):
                        self.showFullScreen()

                    # create & join asynczsocket
                    self.__socket = AsyncZSocket(f"{self.__class__.__name__}", "subscribe")
                    if self.__socket.create(pipeline=zpipe):
                        transport = config.get("transport", "tcp")
                        port = config.get("port", 9001)
                        host = config.get("host", "localhost")
                        if self.__socket.join(transport, host, port):
                            self.__socket.subscribe("call")
                            self.__socket.set_message_callback(self.__on_data_received)
                            self.__console.debug(f"Socket created and joined: {transport}://{host}:{port}")
                        else:
                            self.__console.error("Failed to join socket")
                    else:
                        self.__console.error("Failed to create socket")

                else:
                    raise Exception(f"Cannot found UI file : {ui_path}")
                
        except Exception as e:
            self.__console.error(f"{e}")

    def __on_data_received(self, multipart_data):
        """Callback function for zpipe data reception"""
        if len(multipart_data) < 2:
            self.__console.error(f"({self.__class__.__name__}) Invalid multipart data received")
            return

        topic = multipart_data[0]
        msg = multipart_data[1]

        if topic.decode() == "call":
            msg_decoded = json.loads(msg.decode('utf8').replace("'", '"'))
            try:
                function_name = msg_decoded["function"]
                function = getattr(super(), function_name)
                kwargs = msg_decoded["kwargs"]
                # function(self._scene, **kwargs)

            except json.JSONDecodeError as e:
                self.__console.error(f"({self.__class__.__name__}) {e}")
            except Exception as e:
                self.__console.error(f"({self.__class__.__name__}) {e}")

    def closeEvent(self, event:QCloseEvent) -> None:
        try:            
            # Clear all geometry in viewer3d before closing
            self.__console.info("Terminating System")
            # self.__call(socket=self.__socket, function="API_system_termination", kwargs={})


            # Clean up subscriber socket first
            if hasattr(self, '_PatrolWindow__socket') and self.__socket:
                self.__socket.destroy_socket()
                self.__console.debug(f"({self.__class__.__name__}) Destroyed socket")

        except Exception as e:
            self.__console.error(f"Error during window close: {e}")
        finally:
            self.__console.info("Successfully Closed")
            return super().closeEvent(event)
        
    def set_label_status(self, widget_name:str, status:int):
        widget = self.findChild(QLabel, widget_name)
        if widget:
            if status == 1: # on | warning
                widget.setStyleSheet("background-color: yellow;color: black;border: 1px solid #555555;")
            elif status == 2: # on | normal | good
                widget.setStyleSheet("background-color: green;color: black;border: 1px solid #555555;")
            else: # off | critical
                widget.setStyleSheet("background-color: red;color: white;border: 1px solid #555555;")

