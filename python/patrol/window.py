"""
Patroller Operation/Monitoring Window
@author Byunghun Hwang<bh.hwang@iae.re.kr>
"""

try:
    # using PyQt6
    from PyQt6.QtGui import QImage, QPixmap, QCloseEvent, QStandardItem, QStandardItemModel, QShortcut, QKeySequence
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

                    # Global shortcuts for Exit
                    QShortcut(QKeySequence("Esc"), self, self.close)
                    QShortcut(QKeySequence("Q"), self, self.close)

                    # Connect button events
                    try:
                        self.btn_snap.clicked.connect(self.on_btn_snap_clicked)
                        self.btn_3dscan.clicked.connect(self.on_btn_3dscan_clicked)
                        self.btn_setup.clicked.connect(self.on_btn_setup_clicked)
                        self.btn_camera.clicked.connect(self.on_btn_camera_clicked)
                        self.btn_location.clicked.connect(self.on_btn_location_clicked)
                        self.btn_acoustic.clicked.connect(self.on_btn_acoustic_clicked)
                        self.btn_data.clicked.connect(self.on_btn_data_clicked)
                        self.btn_sequence.clicked.connect(self.on_btn_sequence_clicked)
                    except AttributeError as e:
                        self.__console.warning(f"Button connection failed: {e}")

                    # device instances (dynamic loading based on config)
                    self.active_modules = {}
                    use_modules = config.get("use_module", [])
                    if isinstance(use_modules, list):
                        import importlib
                        for mod_name in use_modules:
                            try:
                                mod = importlib.import_module(f"module.{mod_name}")
                                
                                kwargs = {}
                                cfg_path = pathlib.Path(__file__).parent.parent / "module" / f"{mod_name}.cfg"
                                if cfg_path.is_file():
                                    with open(cfg_path, 'r', encoding='utf-8') as f:
                                        kwargs = json.load(f)
                                else:
                                    self.__console.warning(f"Configuration file not found: {cfg_path.name}")
                                    
                                module = mod.component(**kwargs)
                                
                                if hasattr(module, 'packet_received'):
                                    module.packet_received.connect(self.on_packet_received)
                                
                                self.active_modules[mod_name] = module
                                
                                # Start thread
                                self.active_modules[mod_name].start()
                                self.__console.info(f"Loaded and started module: {mod_name}")
                            except Exception as e:
                                self.__console.error(f"Failed to load module {mod_name}: {e}")


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

    def on_packet_received(self, packet):
        if isinstance(packet, tuple): # (idx, packet)
            packet = packet[1]
            print(f"received pacekt : {packet}")

    def start_scanner_streaming(self):
        """ start scanner streaming """
        ouster = self.active_modules.get("ouster_lidar")
        if ouster and not ouster.isRunning():
            ouster.start()
            print("LiDAR streaming started")

    def stop_scanner_streaming(self):
        """ stop scanner streaming """
        ouster = self.active_modules.get("ouster_lidar")
        if ouster and ouster.isRunning():
            ouster.stop()
            print("LiDAR streaming stopped")


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
            self.__console.info("Terminating System")
            # self.__call(socket=self.__socket, function="API_system_termination", kwargs={})


            # Stop all active modules
            if hasattr(self, 'active_modules'):
                for name, module in self.active_modules.items():
                    if module.isRunning():
                        if hasattr(module, 'stop'):
                            module.stop()
                        module.wait()
                        self.__console.debug(f"Stopped module: {name}")

            # Clean up subscriber socket first
            if hasattr(self, '_PatrolWindow__socket') and self.__socket:
                self.__socket.destroy_socket()
                self.__console.debug(f"({self.__class__.__name__}) Destroyed socket")

        except Exception as e:
            self.__console.error(f"Error during window close: {e}")
        finally:
            self.__console.info("Successfully Closed")
            event.accept()
        
    def set_label_status(self, widget_name:str, status:int):
        widget = self.findChild(QLabel, widget_name)
        if widget:
            if status == 1: # on | warning
                widget.setStyleSheet("background-color: yellow;color: black;border: 1px solid #555555;")
            elif status == 2: # on | normal | good
                widget.setStyleSheet("background-color: green;color: black;border: 1px solid #555555;")
            else: # off | critical
                widget.setStyleSheet("background-color: red;color: white;border: 1px solid #555555;")

    # Button Event Handlers ----------------------------------------------------
    def __switch_tab(self, target: str):
        if not hasattr(self, 'tabview'):
            return
            
        for i in range(self.tabview.count()):
            widget = self.tabview.widget(i)
            # Match by title or objectName
            if (self.tabview.tabText(i).lower() == target.lower() or 
                widget.objectName() == target or 
                widget.objectName() == f"tab_{target}" or 
                widget.objectName() == f"btn_{target}"):
                self.tabview.setCurrentIndex(i)
                return
        self.__console.warning(f"Tab matching '{target}' not found!")

    def on_btn_snap_clicked(self):
        self.__switch_tab("snap")

    def on_btn_3dscan_clicked(self):
        self.__switch_tab("3dscan")

    def on_btn_setup_clicked(self):
        self.__switch_tab("setup")

    def on_btn_camera_clicked(self):
        self.__switch_tab("camera")

    def on_btn_location_clicked(self):
        self.__switch_tab("location")

    def on_btn_acoustic_clicked(self):
        self.__switch_tab("acoustic")

    def on_btn_data_clicked(self):
        self.__switch_tab("data")

    def on_btn_sequence_clicked(self):
        self.__switch_tab("sequence")
