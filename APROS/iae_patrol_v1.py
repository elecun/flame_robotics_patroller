"""
IAE Patrol Robot Platform Configuration (iae_patrol_v1.py)
Dynamically loads configured device and plugin modules from apros.cfg and injects ZPipe context.
"""

import os
import inspect
import importlib
from typing import Dict, Any, Optional
from core.device.base import BaseDevice
from core.plugin.base import BasePlugin
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()



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
        "baumer_incline": ("baumer_incline", "BaumerIncline"),
        "telescopic_mast": ("telescopic_mast", "TelescopicMast"),
        "synerex_rtk": ("synerex_rtk", "SynerexRTK"),
        "basler_gige_camera": ("basler_gige_camera", "BaslerGigECamera"),
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
        self.baumer_connector: Optional[Any] = None
        self.last_baumer_status: Optional[Dict[str, Any]] = None
        self.synerex_rtk_connector: Optional[Any] = None
        self.last_rtk_data: Optional[Dict[str, Any]] = None
        self.telescopic_mast_connector: Optional[Any] = None
        self.last_mast_data: Optional[Dict[str, Any]] = None
        self.basler_camera_connector: Optional[Any] = None
        self.last_camera_header: Optional[Dict[str, Any]] = None
        self.last_camera_frame: Optional[bytes] = None
        self.ouster_connector: Optional[Any] = None
        self.last_ouster_points: Optional[Any] = None

        # Parse platform device list from config
        device_names = []
        if self.config and self.config.has_section("PLATFORM"):
            dev_str = self.config.get("PLATFORM", "devices", fallback="")
            if dev_str:
                device_names = [d.strip() for d in dev_str.split(",") if d.strip()]

        if not device_names:
            logger.info("[IAEPatrolV1] No devices specified in config [PLATFORM] devices setting or setting is commented out. No device modules will be loaded.")

        # Dynamically instantiate devices with section parameters from config
        for dev_name in device_names:
            device_obj = self._instantiate_device(dev_name)
            if device_obj:
                self.devices[dev_name] = device_obj
                if dev_name == "mobile_drive_s1":
                    self.drive_base = device_obj
            else:
                logger.error(f"[IAEPatrolV1] Failed to dynamically load device module '{dev_name}'.")
        # Parse path planner modules & drive_executor plugin configuration
        self.global_planner = None
        self.local_planner = None
        self.drive_executor = None

        self._init_path_planners_and_executor()

        # Inject ZPipe context if provided
        if self.zpipe_ctx:
            self.set_zpipe_context(self.zpipe_ctx)

    def _init_path_planners_and_executor(self):
        """Dynamically load RobotConfig, Global Planner, Local Planner, and Drive Executor."""
        urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdf", "iae_patrol_v1.urdf")
        try:
            mod_cfg = importlib.import_module("APROS.core.plugin.path_planner.robot_config" if __name__.startswith("APROS") else "core.plugin.path_planner.robot_config")
            RobotConfig = getattr(mod_cfg, "RobotConfig")
            self.robot_config = RobotConfig.from_urdf(urdf_path)
            # Override max_velocity, min_velocity, lookahead_distance and predict_time from apros.cfg [mobile_drive_s1] section
            if self.config and self.config.has_section("mobile_drive_s1"):
                max_v_kmh = float(self.config.get("mobile_drive_s1", "max_velocity", fallback=3.0))
                min_v_kmh = float(self.config.get("mobile_drive_s1", "min_velocity", fallback=0.0))
                lookahead_dist = float(self.config.get("mobile_drive_s1", "lookahead_distance", fallback=3.0))
                pred_time = float(self.config.get("mobile_drive_s1", "predict_time", fallback=4.0))
                self.robot_config.max_velocity = max_v_kmh / 3.6
                self.robot_config.min_velocity = min_v_kmh / 3.6
                self.robot_config.lookahead_distance = lookahead_dist
                self.robot_config.predict_time = pred_time
                logger.info(f"[IAEPatrolV1] Overrode RobotConfig max_velocity: {max_v_kmh:.1f} km/h ({self.robot_config.max_velocity:.3f} m/s), lookahead_distance: {self.robot_config.lookahead_distance:.1f} m, predict_time: {self.robot_config.predict_time:.1f} s")
        except Exception as e:
            logger.error(f"[IAEPatrolV1] Error loading RobotConfig: {e}")
            return

        gp_mod_name = "cubic_spline_global_planner"
        lp_mod_name = "ackermann_dwa_local_planner"

        if self.config and self.config.has_section("iae_patrol_v1"):
            gp_mod_name = self.config.get("iae_patrol_v1", "global_planner", fallback="cubic_spline_global_planner").strip()
            lp_mod_name = self.config.get("iae_patrol_v1", "local_planner", fallback="ackermann_dwa_local_planner").strip()

        # 1. Dynamically load Global Planner
        try:
            gp_module_path = f"APROS.core.plugin.path_planner.{gp_mod_name}" if __name__.startswith("APROS") else f"core.plugin.path_planner.{gp_mod_name}"
            mod_gp = importlib.import_module(gp_module_path)
            # Find Planner Class in module
            gp_cls = None
            for attr_name in dir(mod_gp):
                if attr_name.lower().endswith("globalplanner") and attr_name != "BaseGlobalPlanner":
                    gp_cls = getattr(mod_gp, attr_name)
                    break
            if gp_cls:
                self.global_planner = gp_cls(self.robot_config)
                logger.info(f"[IAEPatrolV1] Dynamically loaded Global Planner '{gp_mod_name}' ({gp_cls.__name__}).")
        except Exception as e:
            logger.error(f"[IAEPatrolV1] Failed to load Global Planner '{gp_mod_name}': {e}")

        # 2. Dynamically load Local Planner
        try:
            lp_module_path = f"APROS.core.plugin.path_planner.{lp_mod_name}" if __name__.startswith("APROS") else f"core.plugin.path_planner.{lp_mod_name}"
            mod_lp = importlib.import_module(lp_module_path)
            lp_cls = None
            for attr_name in dir(mod_lp):
                if attr_name.lower().endswith("localplanner") and attr_name != "BaseLocalPlanner":
                    lp_cls = getattr(mod_lp, attr_name)
                    break
            if lp_cls:
                self.local_planner = lp_cls(self.robot_config)
                logger.info(f"[IAEPatrolV1] Dynamically loaded Local Planner '{lp_mod_name}' ({lp_cls.__name__}).")
        except Exception as e:
            logger.error(f"[IAEPatrolV1] Failed to load Local Planner '{lp_mod_name}': {e}")

        # 3. Read control_freq from mobile_drive_s1 section if specified
        control_freq = 10.0
        if self.config and self.config.has_section("mobile_drive_s1"):
            control_freq = float(self.config.get("mobile_drive_s1", "control_freq", fallback=10.0))

        # 4. Instantiate Drive Executor Plugin
        try:
            mod_executor = importlib.import_module("APROS.core.plugin.drive_executor" if __name__.startswith("APROS") else "core.plugin.drive_executor")
            DriveExecutor = getattr(mod_executor, "DriveExecutor")
            self.drive_executor = DriveExecutor(
                robot=self,
                global_planner=self.global_planner,
                local_planner=self.local_planner,
                control_freq=control_freq
            )
            self.plugins["drive_executor"] = self.drive_executor
            logger.info(f"[IAEPatrolV1] Drive Executor plugin initialized (control_freq={control_freq}Hz).")
        except Exception as e:
            logger.error(f"[IAEPatrolV1] Error initializing Drive Executor: {e}")

    def _instantiate_device(self, dev_name: str) -> Optional[BaseDevice]:
        """Dynamically import device class and pass arguments parsed from section [dev_name]."""
        if dev_name not in self.DEVICE_MAP:
            logger.warning(f"[IAEPatrolV1] Unknown device '{dev_name}' specified in config.")
            return None

        file_name, class_name = self.DEVICE_MAP[dev_name]
        module_path = f"APROS.core.device.{file_name}" if __name__.startswith("APROS") else f"core.device.{file_name}"

        try:
            mod = importlib.import_module(module_path)
            cls = getattr(mod, class_name)
        except Exception as e:
            logger.error(f"[IAEPatrolV1] Error importing device module '{file_name}': {e}")
            return None

        # Read specific device section settings from config if available
        kwargs = {}
        if self.config and self.config.has_section("PLATFORM"):
            plat_sec = self.config["PLATFORM"]
            if dev_name in ["vlp-16", "ouster-sr-128", "baumer_incline"]:
                robot_model = plat_sec.get("robot_model", fallback="iae_patrol_v1")
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
                if val.lower() in ("true", "yes", "1"):
                    kwargs[key] = True
                elif val.lower() in ("false", "no", "0"):
                    kwargs[key] = False
                else:
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
            logger.info(f"[IAEPatrolV1] Loaded device '{dev_name}' ({class_name}) with parameters: {init_kwargs}")
            return instance
        except Exception as e:
            logger.error(f"[IAEPatrolV1] Error initializing device '{dev_name}': {e}")
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

    def _on_baumer_data_received(self, status: Dict[str, Any]):
        """Callback invoked when BaumerIncline_Connector receives tilt data over ZPipe IPC."""
        self.last_baumer_status = status
        tx = status.get('tilt_x', 0.0)
        tz = status.get('tilt_z', 0.0)
        temp = status.get('temperature', 0)
        # Pass tilt_x to VLP-16 device for ground removal calculation if active
        if "vlp-16" in self.devices and hasattr(self.devices["vlp-16"], "set_vehicle_tilt_x"):
            self.devices["vlp-16"].set_vehicle_tilt_x(tx)
        # Update drive base or platform telemetry status
        if self.drive_base and hasattr(self.drive_base, 'parsed_can_status'):
            self.drive_base.parsed_can_status["Baumer Incline Tilt X (deg)"] = f"{tx:.2f}"
            self.drive_base.parsed_can_status["Baumer Incline Tilt Z (deg)"] = f"{tz:.2f}"
            self.drive_base.parsed_can_status["Baumer Incline Temp (℃)"] = f"{temp}"

    def _on_rtk_data_received(self, data: Dict[str, Any]):
        """Callback invoked when SynerexRTK_Connector receives GNSS position data over ZPipe IPC."""
        self.last_rtk_data = data

    def _on_mast_data_received(self, data: Dict[str, Any]):
        """Callback invoked when TelescopicMast_Connector receives mast extension telemetry over ZPipe IPC."""
        self.last_mast_data = data

    def _on_basler_camera_received(self, header: Dict[str, Any], jpeg_data: bytes):
        """Callback invoked when BaslerGigECamera_Connector receives frame over ZPipe IPC."""
        self.last_camera_header = header
        self.last_camera_frame = jpeg_data

    def _on_ouster_data_received(self, points: Any):
        """Callback invoked when OusterSR128_Connector receives point cloud data over ZPipe IPC."""
        self.last_ouster_points = points

    def connect(self) -> bool:
        """Connect all configured hardware devices and start IPC connectors."""
        success = True
        logger.info("[IAEPatrolV1] Connecting platform devices...")
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
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v1") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v1"
                self.vlp16_connector = VLP16_Connector(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_data_received=self._on_vlp16_data_received
                )
                self.vlp16_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV1] Error initializing VLP16_Connector: {e}")

        # Initialize BaumerIncline_Connector for IPC reception
        if "baumer_incline" in self.devices:
            try:
                mod = importlib.import_module("APROS.core.device.baumer_incline" if __name__.startswith("APROS") else "core.device.baumer_incline")
                BaumerIncline_Connector = getattr(mod, "BaumerIncline_Connector")
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v1") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v1"
                self.baumer_connector = BaumerIncline_Connector(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_data_received=self._on_baumer_data_received
                )
                self.baumer_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV1] Error initializing BaumerIncline_Connector: {e}")

        # Initialize SynerexRTK_Connector for IPC reception
        if "synerex_rtk" in self.devices:
            try:
                mod = importlib.import_module("APROS.core.device.synerex_rtk" if __name__.startswith("APROS") else "core.device.synerex_rtk")
                SynerexRTK_Connector = getattr(mod, "SynerexRTK_Connector")
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v1") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v1"
                self.synerex_rtk_connector = SynerexRTK_Connector(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_data_received=self._on_rtk_data_received
                )
                self.synerex_rtk_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV1] Error initializing SynerexRTK_Connector: {e}")

        # Initialize TelescopicMast_Connector for IPC reception
        if "telescopic_mast" in self.devices:
            try:
                mod = importlib.import_module("APROS.core.device.telescopic_mast" if __name__.startswith("APROS") else "core.device.telescopic_mast")
                TelescopicMast_Connector = getattr(mod, "TelescopicMast_Connector")
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v1") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v1"
                self.telescopic_mast_connector = TelescopicMast_Connector(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_data_received=self._on_mast_data_received
                )
                self.telescopic_mast_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV1] Error initializing TelescopicMast_Connector: {e}")

        # Initialize OusterSR128_Connector for IPC reception
        if "ouster-sr-128" in self.devices:
            try:
                mod = importlib.import_module("APROS.core.device.ouster-sr-128" if __name__.startswith("APROS") else "core.device.ouster-sr-128")
                OusterSR128_Connector = getattr(mod, "OusterSR128_Connector")
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v1") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v1"
                self.ouster_connector = OusterSR128_Connector(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_data_received=self._on_ouster_data_received
                )
                self.ouster_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV1] Error initializing OusterSR128_Connector: {e}")

        # Initialize BaslerGigECamera_Connector for IPC reception
        if "basler_gige_camera" in self.devices:
            try:
                mod = importlib.import_module("APROS.core.device.basler_gige_camera" if __name__.startswith("APROS") else "core.device.basler_gige_camera")
                BaslerGigECamera_Connector = getattr(mod, "BaslerGigECamera_Connector")
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v1") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v1"
                self.basler_camera_connector = BaslerGigECamera_Connector(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_frame_received=self._on_basler_camera_received
                )
                self.basler_camera_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV1] Error initializing BaslerGigECamera_Connector: {e}")

        return success

    def disconnect(self) -> bool:
        """Disconnect all platform devices and stop connectors."""
        logger.info("[IAEPatrolV1] Disconnecting platform devices...")
        if self.vlp16_connector:
            try:
                self.vlp16_connector.stop()
            except Exception:
                pass
            self.vlp16_connector = None
        if self.baumer_connector:
            try:
                self.baumer_connector.stop()
            except Exception:
                pass
            self.baumer_connector = None
        if self.synerex_rtk_connector:
            try:
                self.synerex_rtk_connector.stop()
            except Exception:
                pass
            self.synerex_rtk_connector = None
        if self.telescopic_mast_connector:
            try:
                self.telescopic_mast_connector.stop()
            except Exception:
                pass
            self.telescopic_mast_connector = None
        if self.ouster_connector:
            try:
                self.ouster_connector.stop()
            except Exception:
                pass
            self.ouster_connector = None
        if self.basler_camera_connector:
            try:
                self.basler_camera_connector.stop()
            except Exception:
                pass
            self.basler_camera_connector = None
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
