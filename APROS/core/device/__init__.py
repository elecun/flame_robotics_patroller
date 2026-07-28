"""
core/device package initialization.
"""
from core.device.base import BaseDevice
from core.device.robot_controller import RobotController
from core.device.mobile_drive_s1 import MobileDriveS1
import importlib

# Dynamic module imports to handle hyphenated filenames
vlp16_mod = importlib.import_module("APROS.core.device.vlp-16" if __name__.startswith("APROS") else "core.device.vlp-16")
ouster_mod = importlib.import_module("APROS.core.device.ouster-sr-128" if __name__.startswith("APROS") else "core.device.ouster-sr-128")

VLP16 = vlp16_mod.VLP16
OusterSR128 = ouster_mod.OusterSR128

__all__ = ["BaseDevice", "RobotController", "MobileDriveS1", "VLP16", "OusterSR128"]

