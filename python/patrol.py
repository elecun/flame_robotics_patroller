'''
DRT 3D Window Controller with Qt GUI
@auhtor Byunghun Hwang<bh.hwnag@iae.re.kr>
'''

import sys, os

# -----------------------------------------------------------------------
# QtWebEngine 환경변수는 반드시 Qt/QApplication 초기화 전에 설정해야 함
# Ubuntu 22.04에서 black screen 방지를 위한 필수 설정
# -----------------------------------------------------------------------
if sys.platform in ("linux", "linux2"):
    os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
    os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--disable-gpu --disable-software-rasterizer")

# VTK(vedo)와 QtWebEngine이 동시에 사용될 때 OpenGL 컨텍스트 충돌 방지
# QApplication 생성 전에 반드시 설정해야 함
from PyQt6.QtWidgets import QApplication as _QApp
from PyQt6.QtCore import Qt as _Qt
_QApp.setAttribute(_Qt.ApplicationAttribute.AA_ShareOpenGLContexts)

try:
    from PyQt6.QtGui import QImage, QPixmap, QCloseEvent, QFontDatabase, QFont
    from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QMessageBox
    from PyQt6.uic import loadUi
    from PyQt6.QtCore import QObject, Qt, QTimer, QThread, pyqtSignal
    from PyQt6.QtWebEngineWidgets import QWebEngineView
except ImportError:
    print("PyQt6 and PyQt6-WebEngine are required to run this application.")

import pathlib
import json
from common.zpipe import zpipe_create_pipe, zpipe_destroy_pipe
from common.zpipe import ZPipe

# root directory registration on system environment
ROOT_PATH = pathlib.Path(__file__).parent.parent
APP_NAME = pathlib.Path(__file__).stem
sys.path.append(ROOT_PATH.as_posix())

import argparse
import multiprocessing
import zmq
from multiprocessing import Process
from util.logger.console import ConsoleLogger
from patrol.window import PatrolWindow


if __name__ == "__main__":
    # Fix for multiprocessing on Linux
    if sys.platform == "linux" or sys.platform == "linux2":
        multiprocessing.set_start_method('spawn', force=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', nargs='?', required=False, help="Configuration File(*.cfg)", default=f"{APP_NAME}.cfg")
    parser.add_argument('--verbose_level', nargs='?', required=False, help="Set Verbose Level", default="DEBUG")
    args = parser.parse_args()

    console = ConsoleLogger.get_logger(level="DEBUG")

    app = None
    try:
        with open(args.config, "r") as cfile:
            configure = json.load(cfile)

            configure["root_path"] = ROOT_PATH
            configure["app_path"] = (pathlib.Path(__file__).parent / APP_NAME)
            configure["verbose_level"] = args.verbose_level.upper()

            if configure["verbose_level"] == "DEBUG":
                console.debug(f"Root Path : {configure['root_path']}")
                console.debug(f"Application Path : {configure['app_path']}")
                console.debug(f"Verbose Level : {configure['verbose_level']}")

            # zmq pipeline
            # create zpipe context
            n_ctx_value = configure.get("n_io_context", 10)
            zpipe_instance = zpipe_create_pipe(io_threads=n_ctx_value)

            app = QApplication(sys.argv)  # AA_ShareOpenGLContexts는 이미 상단에서 설정됨
            font_id = QFontDatabase.addApplicationFont((ROOT_PATH / configure['font_path']).as_posix())
            font_family = QFontDatabase.applicationFontFamilies(font_id)[0]
            app.setFont(QFont(font_family, 12))
            window = PatrolWindow(config=configure, zpipe=zpipe_instance)
            window.show()

            exit_cdoe = app.exec()

            # terminate pipeline
            zpipe_destroy_pipe()
            console.info(f"Successfully terminated")
            sys.exit(exit_cdoe)

    except json.JSONDecodeError as e:
        console.critical(f"Configuration File Parse Exception : {e}")
    except Exception as e:
        console.critical(f"General Exception : {e}")
