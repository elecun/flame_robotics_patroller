"""
IAE Patrol Robot Platform Configuration (iae_patrol_v1.py)
Dynamically loads configured device and plugin modules from apros.cfg and injects ZPipe context.
"""

import importlib
from typing import Dict, Any, Optional
from core.device.base import BaseDevice
from core.plugin.base import BasePlugin


class IAEPatrolV1:
    """
    IAE Patrol V1 Robot Platform Instance.
    Dynamically loads and configures devices (e.g. mobile_drive_s1, vlp-16, ouster-sr-128)
    and plugins specified in apros.cfg.
    """

    # Device module mapping: module_name -> (file_name, class_name)
    DEVICE_MAP = {
        "mobile_drive_s1": ("mobile_drive_s1", "MobileDriveS1"),
        "vlp-16": ("vlp-16", "VLP16"),
        "ouster-sr-128": ("ouster-sr-128", "OusterSR128"),
        "robot_controller": ("robot_controller", "RobotController"),
    }

    def __init__(self, config: Optional[Any] = None, zpipe_ctx: Optional[Any] = None):
        self.config = config
        self.zpipe_ctx = zpipe_ctx

        self.devices: Dict[str, BaseDevice] = {}
        self.plugins: Dict[str, BasePlugin] = {}
        self.drive_base: Optional[BaseDevice] = None

        # Parse platform device list from config
        device_names = []
        if self.config and self.config.has_section("PLATFORM"):
            dev_str = self.config.get("PLATFORM", "devices", fallback="")
            if dev_str:
                device_names = [d.strip() for d in dev_str.split(",") if d.strip()]

        # Fallback to default devices if config is empty
        if not device_names:
            device_names = ["mobile_drive_s1", "vlp-16", "ouster-sr-128"]

        # Dynamically instantiate devices with section parameters from config
        for dev_name in device_names:
            device_obj = self._instantiate_device(dev_name)
            if device_obj:
                self.devices[dev_name] = device_obj
                if dev_name == "mobile_drive_s1":
                    self.drive_base = device_obj

        # If drive_base is not set among loaded devices, fall back to first device or MobileDriveS1
        if self.drive_base is None and "mobile_drive_s1" in self.devices:
            self.drive_base = self.devices["mobile_drive_s1"]

        # Inject ZPipe context if provided
        if self.zpipe_ctx:
            self.set_zpipe_context(self.zpipe_ctx)

    def _instantiate_device(self, dev_name: str) -> Optional[BaseDevice]:
        """Dynamically import device class and pass arguments parsed from section [dev_name]."""
        if dev_name not in self.DEVICE_MAP:
            print(f"[IAEPatrolV1] Warning: Unknown device '{dev_name}' specified in config.")
            return None

        file_name, class_name = self.DEVICE_MAP[dev_name]
        module_path = f"APROS.core.device.{file_name}" if __name__.startswith("APROS") else f"core.device.{file_name}"

        try:
            mod = importlib.import_module(module_path)
            cls = getattr(mod, class_name)
        except Exception as e:
            print(f"[IAEPatrolV1] Error importing device module '{file_name}': {e}")
            return None

        # Read specific device section settings from config if available
        kwargs = {}
        if self.config and self.config.has_section(dev_name):
            section = self.config[dev_name]
            for key, val in section.items():
                # Cast numeric strings to int/float appropriately
                try:
                    if "." in val:
                        kwargs[key] = float(val)
                    else:
                        kwargs[key] = int(val)
                except ValueError:
                    kwargs[key] = val

        try:
            instance = cls(name=dev_name, **kwargs)
            print(f"[IAEPatrolV1] Loaded device '{dev_name}' ({class_name}) with parameters: {kwargs}")
            return instance
        except Exception as e:
            print(f"[IAEPatrolV1] Error initializing device '{dev_name}': {e}")
            return None

    def set_zpipe_context(self, zpipe_ctx: Any):
        """Pass ZPipe context to all registered device and plugin components."""
        self.zpipe_ctx = zpipe_ctx
        for dev in self.devices.values():
            dev.set_zpipe_context(self.zpipe_ctx)
        for plug in self.plugins.values():
            plug.set_zpipe_context(self.zpipe_ctx)

    def connect(self) -> bool:
        """Connect all configured hardware devices."""
        success = True
        print("[IAEPatrolV1] Connecting platform devices...")
        for name, device in self.devices.items():
            res = device.connect()
            print(f"  - Device '{name}': {'Connected' if res else 'Connection failed / Standby'}")
            if not res and name == "mobile_drive_s1":
                success = False
        return success

    def disconnect(self) -> bool:
        """Disconnect all platform devices."""
        print("[IAEPatrolV1] Disconnecting platform devices...")
        for name, device in self.devices.items():
            device.disconnect()
        return True

    def get_status(self) -> Dict[str, Any]:
        """Aggregate status across all components for dashboard & telemetry."""
        status = {}
        if self.drive_base:
            status = self.drive_base.get_status()
        for name, dev in self.devices.items():
            if name != "mobile_drive_s1":
                status[f"{name}_status"] = dev.get_status()
        return status

    # Delegated properties and methods for backward compatibility with Viser UI / controllers
    @property
    def speed(self) -> float:
        return self.drive_base.speed if self.drive_base and hasattr(self.drive_base, 'speed') else 0.0

    @speed.setter
    def speed(self, val: float):
        if self.drive_base and hasattr(self.drive_base, 'speed'):
            self.drive_base.speed = val

    @property
    def steer_angle(self) -> float:
        return self.drive_base.steer_angle if self.drive_base and hasattr(self.drive_base, 'steer_angle') else 0.0

    @steer_angle.setter
    def steer_angle(self, val: float):
        if self.drive_base and hasattr(self.drive_base, 'steer_angle'):
            self.drive_base.steer_angle = val

    @property
    def gear(self) -> str:
        return self.drive_base.gear if self.drive_base and hasattr(self.drive_base, 'gear') else "P"

    @gear.setter
    def gear(self, val: str):
        if self.drive_base and hasattr(self.drive_base, 'gear'):
            self.drive_base.gear = val

    @property
    def drive_mode(self) -> str:
        return self.drive_base.drive_mode if self.drive_base and hasattr(self.drive_base, 'drive_mode') else "Manual"

    @drive_mode.setter
    def drive_mode(self, val: str):
        if self.drive_base and hasattr(self.drive_base, 'drive_mode'):
            self.drive_base.drive_mode = val

    @property
    def is_connected(self) -> bool:
        return self.drive_base.is_connected if self.drive_base else False

    def set_steering_angle(self, angle_deg: float):
        if self.drive_base and hasattr(self.drive_base, 'set_steering_angle'):
            self.drive_base.set_steering_angle(angle_deg)

    def update_simulation_step(self, dt: float = 0.05):
        if self.drive_base and hasattr(self.drive_base, 'update_simulation_step'):
            self.drive_base.update_simulation_step(dt=dt)
