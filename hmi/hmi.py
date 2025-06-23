'''
Patrol Robot Operation GUI Panel
@auhtor Byunghun Hwang<bh.hwnag@iae.re.kr>
'''

import sys, os
import pathlib
import json

try:
    # using PyQt5
    from PyQt5.QtWidgets import QApplication
except ImportError:
    # using PyQt6
    from PyQt6.QtGui import QImage, QPixmap, QCloseEvent, QFontDatabase, QFont
    from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QMessageBox
    from PyQt6.uic import loadUi
    from PyQt6.QtCore import QObject, Qt, QTimer, QThread, pyqtSignal

# root directory registration on system environment
ROOT_PATH = pathlib.Path(__file__).parent.parent
APP_NAME = pathlib.Path(__file__).stem
sys.path.append(ROOT_PATH.as_posix())

import argparse
from gui.window import AppWindow
from util.logger.console import ConsoleLogger


if __name__ == "__main__":
    
    console = ConsoleLogger.get_logger()

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', nargs='?', required=False, help="Configuration File(*.cfg)", default="default.cfg")
    parser.add_argument('--verbose', nargs='?', required=False, help="Enable/Disable verbose", default=True)
    args = parser.parse_args()

    app = None
    try:
        with open(args.config, "r") as cfile:
            configure = json.load(cfile)

            configure["root_path"] = ROOT_PATH
            configure["app_path"] = (pathlib.Path(__file__).parent / APP_NAME)
            configure["verbose"] = args.verbose

            if args.verbose:
                console.info(f"+ Root Path : {configure['root_path']}")
                console.info(f"+ Application Path : {configure['app_path']}")
                console.info(f"+ Verbose Level : {configure['verbose']}")

            app = QApplication(sys.argv)
            font_id = QFontDatabase.addApplicationFont((ROOT_PATH / configure['font_path']).as_posix())
            font_family = QFontDatabase.applicationFontFamilies(font_id)[0]
            app.setFont(QFont(font_family, 12))
            app_window = AppWindow(config=configure)
            
            if "app_window_title" in configure:
                app_window.setWindowTitle(configure["app_window_title"])
            app_window.show()
            sys.exit(app.exec())

    except json.JSONDecodeError as e:
        console.critical(f"Configuration File Load Error : {e}")
    except Exception as e:
        console.critical(f"{e}")
        
    
        
    