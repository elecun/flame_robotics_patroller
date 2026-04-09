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
from pyvistaqt import QtInteractor

from util.logger.console import ConsoleLogger
from common.zpipe import AsyncZSocket, ZPipe
from .geometry import geometry

# for device
from module.ouster_lidar import ouster_os0_128
from module.pylon_camera import pylonCamera
from module.rtk_gnss import smc2000_rtk
from module.velodyne_lidar import velodyne_vlp16


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

                    # device instances
                    self.ouster_os0_thread = ouster_os0_128(hostname="192.168.0.10")
                    self.ouster_os0_thread.packet_received.connect(self.on_packet_received)


                    # UI component event proc.
                    self.btn_record.clicked.connect(self.on_btn_record)

                    # lazy loading for 3D viewer
                    QTimer.singleShot(10, self.__init_3d_viewer)

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

    def on_btn_record(self):
        """ start record """
        if self.ouster_os0_thread.isRunning():
            filepath = "/"
            self.ouster_os0_thread.start_record(filepath)
        else:
            print(f"streaming is not active")

    def on_packet_received(self, packet):
        if isinstance(packet, tuple): # (idx, packet)
            packet = packet[1]
            print(f"received pacekt : {packet}")

    def start_scanner_streaming(self):
        """ start scanner streaming """
        if not self.ouster_os0_thread.isRunning():
            self.ouster_os0_thread.start()
            print("LiDAR streaming started")

    def stop_scanner_streaming(self):
        """ stop scanner streaming """
        if self.ouster_os0_thread.isRunning():
            self.ouster_os0_thread.stop()
            print("LiDAR streaming stopped")


    def __init_3d_viewer(self):
        """initialize 3D viewer"""
        try:
            # using pyvistaqt
            self.plotter = QtInteractor(self.widget_3d_frame)
            self.plotter.background_color = self.__config.get("background-color", [0.2, 0.2, 0.2])
            layout = self.widget_3d_frame.layout()
            if layout is None:
                layout = QVBoxLayout(self.widget_3d_frame)
                layout.setContentsMargins(0, 0, 0, 0) # zero margin
                layout.setSpacing(0)  # zero spacing
                self.widget_3d_frame.setLayout(layout)
            else:
                layout.setContentsMargins(0, 0, 0, 0)
                layout.setSpacing(0)
            layout.addWidget(self.plotter.interactor)

            # render geometry
            self.geometry_api = geometry()
            self.geometry_api.API_add_coord_frame(self.plotter, "origin", pos=[0,0,0], ori=[0,0,0], size=0.1)
            self.__console.debug("3D Viewer is initialized")
        except Exception as e:
            self.__console.error(f"Failed to initialize 3D viewer: {e}")

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

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Q or event.key() == Qt.Key.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(event)

