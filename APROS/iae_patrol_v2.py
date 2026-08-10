"""
IAE Patrol Robot Platform Configuration V2 (iae_patrol_v2.py)
Dynamically loads configured device and plugin modules from apros.cfg and injects ZPipe context.
Excludes baumer_incline device while maintaining compatibility with IAEPatrolV1 structure.
"""

import importlib
import inspect
from typing import Dict, Any, Optional
from core.device.base import BaseDevice
from core.plugin.base import BasePlugin
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class IAEPatrolV2:
    """
    IAE Patrol V2 Robot Platform Instance.
    Dynamically loads and configures devices (e.g. mobile_drive_s1, vlp-16, ouster-sr-128)
    and plugins specified in apros.cfg (excluding baumer_incline).
    """

    # Device module mapping: module_name -> (file_name, class_name)
    DEVICE_MAP = {
        "mobile_drive_s1": ("mobile_drive_s1", "MobileDriveS1"),
        "vlp-16": ("vlp-16", "VLP16"),
        "ouster-sr-128": ("ouster-sr-128", "OusterSR128"),
        "telescopic_mast": ("telescopic_mast", "TelescopicMast"),
        "synerex_rtk": ("synerex_rtk", "SynerexRTK"),
        "robot_controller": ("robot_controller", "RobotController"),
    }

    def __init__(self, config: Optional[Any] = None, zpipe_ctx: Optional[Any] = None):
        self.config = config
        self.zpipe_ctx = zpipe_ctx

        self.devices: Dict[str, BaseDevice] = {}
        self.plugins: Dict[str, BasePlugin] = {}
        self.drive_base: Optional[BaseDevice] = None
        self.vlp16_connector: Optional[Any] = None
        self.last_vlp16_points: Optional[Any] = None
        self.synerex_rtk_connector: Optional[Any] = None
        self.last_rtk_data: Optional[Dict[str, Any]] = None

        # Parse platform device list from config
        device_names = []
        if self.config and self.config.has_section("PLATFORM"):
            dev_str = self.config.get("PLATFORM", "devices", fallback="")
            if dev_str:
                device_names = [d.strip() for d in dev_str.split(",") if d.strip() and d.strip() != "baumer_incline"]

        # Fallback to default devices if config is empty (baumer_incline is omitted in V2)
        if not device_names:
            device_names = ["mobile_drive_s1", "vlp-16", "ouster-sr-128", "telescopic_mast", "synerex_rtk"]

        # Dynamically instantiate devices with section parameters from config
        for dev_name in device_names:
            if dev_name == "baumer_incline":
                continue
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

        # Dynamically load Mast Executor Plugin
        self.mast_executor = None
        try:
            mod_me = importlib.import_module("APROS.core.plugin.mast_executor" if __name__.startswith("APROS") else "core.plugin.mast_executor")
            MastExecutor = getattr(mod_me, "MastExecutor")
            stop_trig_b = 15.0
            if self.config:
                if self.config.has_section("telescopic_mast") and "stop_trig_bound" in self.config["telescopic_mast"]:
                    stop_trig_b = float(self.config.get("telescopic_mast", "stop_trig_bound", fallback=15.0))
                elif self.config.has_section("mast_executor") and "stop_trig_bound" in self.config["mast_executor"]:
                    stop_trig_b = float(self.config.get("mast_executor", "stop_trig_bound", fallback=15.0))

            self.mast_executor = MastExecutor(robot=self, stop_trig_bound=stop_trig_b)
            self.plugins["mast_executor"] = self.mast_executor
            logger.info(f"[IAEPatrolV2] Mast Executor plugin initialized (stop_trig_bound={stop_trig_b}mm).")
        except Exception as e:
            logger.error(f"[IAEPatrolV2] Error initializing Mast Executor: {e}")

    def _instantiate_device(self, dev_name: str) -> Optional[BaseDevice]:
        """Dynamically import device class and pass arguments parsed from section [dev_name]."""
        if dev_name not in self.DEVICE_MAP:
            logger.warning(f"[IAEPatrolV2] Unknown device '{dev_name}' specified in config.")
            return None

        file_name, class_name = self.DEVICE_MAP[dev_name]
        module_path = f"APROS.core.device.{file_name}" if __name__.startswith("APROS") else f"core.device.{file_name}"

        try:
            mod = importlib.import_module(module_path)
            cls = getattr(mod, class_name)
        except Exception as e:
            logger.error(f"[IAEPatrolV2] Error importing device module '{file_name}': {e}")
            return None

        # Read specific device section settings from config if available
        kwargs = {}
        if self.config and self.config.has_section("PLATFORM"):
            plat_sec = self.config["PLATFORM"]
            if dev_name in ["vlp-16", "ouster-sr-128"]:
                robot_model = plat_sec.get("robot_model", fallback="iae_patrol_v2")
                kwargs["robot_model"] = robot_model
            for key in ("default_lat", "default_lon", "default_alt"):
                if key in plat_sec:
                    try:
                        kwargs[key] = float(plat_sec[key])
                    except ValueError:
                        kwargs[key] = plat_sec[key]

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

        # Check if ground_removal algorithm module section (e.g. [patchworkpp]) exists in config
        ground_removal_module = kwargs.get("ground_removal")
        if ground_removal_module and isinstance(ground_removal_module, str) and self.config:
            mod_sec_name = ground_removal_module.lower()
            if self.config.has_section(mod_sec_name):
                gr_params = {}
                for key, val in self.config[mod_sec_name].items():
                    if val.lower() in ("true", "yes", "1"):
                        gr_params[key] = True
                    elif val.lower() in ("false", "no", "0"):
                        gr_params[key] = False
                    else:
                        try:
                            if "." in val:
                                gr_params[key] = float(val)
                            else:
                                gr_params[key] = int(val)
                        except ValueError:
                            gr_params[key] = val
                kwargs["ground_removal_params"] = gr_params

        try:
            sig = inspect.signature(cls)
            has_kwargs = any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values())
            if not has_kwargs:
                init_kwargs = {k: v for k, v in kwargs.items() if k in sig.parameters}
            else:
                init_kwargs = kwargs

            instance = cls(name=dev_name, **init_kwargs)
            logger.info(f"[IAEPatrolV2] Loaded device '{dev_name}' ({class_name}) with parameters: {init_kwargs}")
            return instance
        except Exception as e:
            logger.error(f"[IAEPatrolV2] Error initializing device '{dev_name}': {e}")
            return None

    def set_zpipe_context(self, zpipe_ctx: Any):
        """Pass ZPipe context to all registered device and plugin components."""
        self.zpipe_ctx = zpipe_ctx
        for dev in self.devices.values():
            dev.set_zpipe_context(self.zpipe_ctx)
        for plug in self.plugins.values():
            plug.set_zpipe_context(self.zpipe_ctx)

    def _on_vlp16_data_received(self, points: Any):
        """Callback invoked when VLP16_Connector receives point cloud data over ZPipe IPC."""
        self.last_vlp16_points = points

    def _on_rtk_data_received(self, data: Dict[str, Any]):
        """Callback invoked when SynerexRTK_Connector receives GNSS position data over ZPipe IPC."""
        self.last_rtk_data = data

    def connect(self) -> bool:
        """Connect all configured hardware devices and start IPC connectors."""
        success = True
        logger.info("[IAEPatrolV2] Connecting platform devices...")
        for name, device in self.devices.items():
            res = device.connect()
            logger.info(f"  - Device '{name}': {'Connected' if res else 'Connection failed / Standby'}")
            if not res and name == "mobile_drive_s1":
                success = False

        # Initialize VLP16_Connector for IPC reception
        if "vlp-16" in self.devices:
            try:
                mod = importlib.import_module("APROS.core.device.vlp-16" if __name__.startswith("APROS") else "core.device.vlp-16")
                VLP16_Connector = getattr(mod, "VLP16_Connector")
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v2") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v2"
                self.vlp16_connector = VLP16_Connector(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_data_received=self._on_vlp16_data_received
                )
                self.vlp16_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV2] Error initializing VLP16_Connector: {e}")

        # Initialize SynerexRTK_Connector for IPC reception
        if "synerex_rtk" in self.devices:
            try:
                mod = importlib.import_module("APROS.core.device.synerex_rtk" if __name__.startswith("APROS") else "core.device.synerex_rtk")
                SynerexRTK_Connector = getattr(mod, "SynerexRTK_Connector")
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v2") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v2"
                self.synerex_rtk_connector = SynerexRTK_Connector(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_data_received=self._on_rtk_data_received
                )
                self.synerex_rtk_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV2] Error initializing SynerexRTK_Connector: {e}")

        return success

    def disconnect(self) -> bool:
        """Disconnect all platform devices and stop connectors."""
        logger.info("[IAEPatrolV2] Disconnecting platform devices...")
        if self.vlp16_connector:
            try:
                self.vlp16_connector.stop()
            except Exception:
                pass
            self.vlp16_connector = None
        if self.synerex_rtk_connector:
            try:
                self.synerex_rtk_connector.stop()
            except Exception:
                pass
            self.synerex_rtk_connector = None
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
        if self.drive_base:
            if hasattr(self.drive_base, 'set_speed'):
                self.drive_base.set_speed(val)
            elif hasattr(self.drive_base, 'speed'):
                self.drive_base.speed = val

    @property
    def steer_angle(self) -> float:
        return self.drive_base.steer_angle if self.drive_base and hasattr(self.drive_base, 'steer_angle') else 0.0

    @steer_angle.setter
    def steer_angle(self, val: float):
        if self.drive_base:
            if hasattr(self.drive_base, 'set_steering_angle'):
                self.drive_base.set_steering_angle(val)
            elif hasattr(self.drive_base, 'steer_angle'):
                self.drive_base.steer_angle = val

    @property
    def gear(self) -> str:
        return self.drive_base.gear if self.drive_base and hasattr(self.drive_base, 'gear') else "P"

    @gear.setter
    def gear(self, val: str):
        if self.drive_base:
            if hasattr(self.drive_base, 'target_gear'):
                self.drive_base.target_gear = val
            if hasattr(self.drive_base, 'gear'):
                self.drive_base.gear = val

    @property
    def drive_mode(self) -> str:
        return self.drive_base.drive_mode if self.drive_base and hasattr(self.drive_base, 'drive_mode') else "Manual"

    @drive_mode.setter
    def drive_mode(self, val: str):
        if self.drive_base and hasattr(self.drive_mode, 'drive_mode'):
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
