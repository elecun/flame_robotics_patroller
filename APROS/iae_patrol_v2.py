"""
IAE Patrol Robot Platform Configuration V2 (iae_patrol_v2.py)
Dynamically loads configured device and plugin modules from apros.cfg and injects ZPipe context.
Excludes baumer_incline device while maintaining compatibility with IAEPatrolV1 structure.
"""

import importlib
import inspect
import math
import os
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
        "synerex_syn2000": ("synerex_syn2000", "SynerexSyn2000"),
        "synerex_syn3100": ("synerex_syn3100", "SynerexSyn3100"),
        "fluke_sv600": ("fluke_sv600", "FlukeSV600"),
        "xsens_mti_630r": ("xsens_mti_630r", "XsensMTi630R"),
        "tabos_bmu": ("tabos_bmu", "TabosBMU"),
        "basler_gige_camera": ("basler_gige_camera", "BaslerGigECamera"),
        "robot_controller": ("robot_controller", "RobotController"),
        "des_control": ("des_control", "DesControl"),
    }

    def __init__(self, config: Optional[Any] = None, zpipe_ctx: Optional[Any] = None, simulation_mode: bool = False):
        self.config = config
        self.zpipe_ctx = zpipe_ctx
        self.simulation_mode = simulation_mode

        self.devices: Dict[str, BaseDevice] = {}
        self.plugins: Dict[str, BasePlugin] = {}
        self.drive_base: Optional[BaseDevice] = None
        self.vlp16_connector: Optional[Any] = None
        self.last_vlp16_points: Optional[Any] = None
        self.ouster_connector: Optional[Any] = None
        self.last_ouster_points: Optional[Any] = None
        self.synerex_rtk_connector: Optional[Any] = None
        self.last_rtk_data: Optional[Dict[str, Any]] = None
        self.basler_camera_connector: Optional[Any] = None
        self.last_camera_header: Optional[Dict[str, Any]] = None
        self.last_camera_frame: Optional[bytes] = None
        self.global_planner = None
        self.local_planner = None
        self.drive_executor = None
        self.simulated_x = 0.0
        self.simulated_y = 0.0
        self.simulated_heading = 0.0

        # Parse platform device list from config
        device_names = []
        if self.config and self.config.has_section("PLATFORM"):
            dev_str = self.config.get("PLATFORM", "devices", fallback="")
            if dev_str:
                device_names = [d.strip() for d in dev_str.split(",") if d.strip() and d.strip() != "baumer_incline"]

        # Fallback to default devices if config is empty (baumer_incline is omitted in V2)
        if not device_names:
            device_names = ["mobile_drive_s1", "vlp-16", "ouster-sr-128", "telescopic_mast", "synerex_syn2000"]

        # Dynamically instantiate devices with section parameters from config
        for dev_name in device_names:
            if dev_name == "baumer_incline":
                continue
            if self.simulation_mode:
                if dev_name != "mobile_drive_s1":
                    logger.info(f"[IAEPatrolV2] Simulation mode: skipping hardware device '{dev_name}'.")
                    continue
                device_obj = self._instantiate_simulated_drive()
            else:
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

        self._init_navigation_planners()

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

        # Dynamically load Mission Manager Plugin
        self.mission_manager = None
        try:
            mod_mm = importlib.import_module("APROS.core.plugin.mission_manager" if __name__.startswith("APROS") else "core.plugin.mission_manager")
            MissionManager = getattr(mod_mm, "MissionManager")
            self.mission_manager = MissionManager(robot=self, poi_reach_threshold=0.5)
            self.plugins["mission_manager"] = self.mission_manager
            logger.info(f"[IAEPatrolV2] Mission Manager plugin initialized (poi_reach_threshold=0.5m).")
        except Exception as e:
            logger.error(f"[IAEPatrolV2] Error initializing Mission Manager: {e}")

        if self.zpipe_ctx:
            self.set_zpipe_context(self.zpipe_ctx)

    def _init_navigation_planners(self):
        """Load V2 global/local planners and connect them to DriveExecutor."""
        try:
            mod_cfg = importlib.import_module("APROS.core.plugin.path_planner.robot_config" if __name__.startswith("APROS") else "core.plugin.path_planner.robot_config")
            RobotConfig = getattr(mod_cfg, "RobotConfig")
            model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v2").strip().lower() if self.config else "iae_patrol_v2"
            urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdf", f"{model}.urdf")
            self.robot_config = RobotConfig.from_urdf(urdf_path)
            if self.config:
                self.robot_config.update_from_config(self.config)

            section = self.config["iae_patrol_v2"] if self.config and self.config.has_section("iae_patrol_v2") else {}
            gp_name = section.get("global_planner", "cubic_spline_global_planner").strip()
            lp_name = section.get("local_planner", "mppi_local_planner").strip()
            pkg = "APROS.core.plugin.path_planner" if __name__.startswith("APROS") else "core.plugin.path_planner"

            gp_mod = importlib.import_module(f"{pkg}.{gp_name}")
            gp_cls = next((getattr(gp_mod, n) for n in dir(gp_mod)
                           if n.lower().endswith("globalplanner") and n != "BaseGlobalPlanner"), None)
            if gp_cls:
                self.global_planner = gp_cls(self.robot_config)

            lp_mod = importlib.import_module(f"{pkg}.{lp_name}")
            lp_cls = next((getattr(lp_mod, n) for n in dir(lp_mod)
                           if n.lower().endswith("localplanner") and n != "BaseLocalPlanner"), None)
            if lp_cls:
                mppi = self.config["mppi_local_planner"] if self.config and self.config.has_section("mppi_local_planner") else {}
                self.local_planner = lp_cls(
                    self.robot_config,
                    samples=int(mppi.get("samples", 96)),
                    horizon=int(mppi.get("horizon", 36)),
                    iterations=int(mppi.get("iterations", 2)),
                    temperature=float(mppi.get("temperature", 1.0)),
                    steering_noise=float(mppi.get("steering_noise", 0.16)),
                    velocity_noise=float(mppi.get("velocity_noise", 0.22)),
                )

            drive_cfg = self.config["mobile_drive_s1"] if self.config and self.config.has_section("mobile_drive_s1") else {}
            mod_exec = importlib.import_module("APROS.core.plugin.drive_executor" if __name__.startswith("APROS") else "core.plugin.drive_executor")
            DriveExecutor = getattr(mod_exec, "DriveExecutor")
            self.drive_executor = DriveExecutor(
                robot=self, global_planner=self.global_planner, local_planner=self.local_planner,
                control_freq=float(drive_cfg.get("control_freq", 10.0)))
            self.plugins["drive_executor"] = self.drive_executor
            logger.info(f"[IAEPatrolV2] Navigation ready: global={gp_name}, local={lp_name}.")
        except Exception as e:
            logger.error(f"[IAEPatrolV2] Failed to initialize V2 navigation planners: {e}")

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
            if dev_name in ["vlp-16", "ouster-sr-128", "synerex_syn2000", "synerex_syn3100"]:
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
                if key == "status":
                    continue
                if val.lower() in ("true", "yes", "1"):
                    kwargs[key] = True
                    continue
                if val.lower() in ("false", "no", "0"):
                    kwargs[key] = False
                    continue
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

    def _instantiate_simulated_drive(self) -> Optional[BaseDevice]:
        """Build a pure software drive model without importing MobileDriveS1/CANlib."""
        try:
            mod = importlib.import_module("APROS.core.device.robot_controller" if __name__.startswith("APROS") else "core.device.robot_controller")
            cls = getattr(mod, "RobotController")
            instance = cls(name="mobile_drive_s1")
            logger.info("[IAEPatrolV2] Using RobotController simulation drive; MobileDriveS1/CANlib was not loaded.")
            return instance
        except Exception as e:
            logger.error(f"[IAEPatrolV2] Could not initialize simulated drive: {e}")
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

    def _on_ouster_data_received(self, points: Any):
        """Store Ouster points for the Viser scene update loop."""
        self.last_ouster_points = points

    def _on_rtk_data_received(self, data: Dict[str, Any]):
        """Callback invoked when SynerexRTK_Connector receives GNSS position data over ZPipe IPC."""
        self.last_rtk_data = data

    def _on_basler_camera_received(self, header: Dict[str, Any], jpeg_data: bytes):
        self.last_camera_header = header
        self.last_camera_frame = jpeg_data

    def connect(self) -> bool:
        """Connect all configured hardware devices and start IPC connectors."""
        if getattr(self, "simulation_mode", False):
            logger.info("[IAEPatrolV2] Simulation mode: skipping all device and IPC connections.")
            if self.drive_base:
                self.drive_base.drive_mode = "Auto"
                self.drive_base.is_connected = True
            return True
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

        if "ouster-sr-128" in self.devices:
            try:
                mod = importlib.import_module("APROS.core.device.ouster-sr-128" if __name__.startswith("APROS") else "core.device.ouster-sr-128")
                connector_class = getattr(mod, "OusterSR128_Connector")
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v2") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v2"
                self.ouster_connector = connector_class(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_data_received=self._on_ouster_data_received,
                )
                self.ouster_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV2] Error initializing OusterSR128_Connector: {e}")

        # Initialize SynerexRTK_Connector for IPC reception
        synerex_name = next((name for name in ("synerex_syn2000", "synerex_syn3100", "synerex_rtk") if name in self.devices), None)
        if synerex_name:
            try:
                mod_name = "synerex_rtk" if synerex_name == "synerex_rtk" else synerex_name
                mod = importlib.import_module(f"APROS.core.device.{mod_name}" if __name__.startswith("APROS") else f"core.device.{mod_name}")
                connector_class = "SynerexRTK_Connector" if synerex_name == "synerex_rtk" else ("SynerexSyn2000_Connector" if synerex_name == "synerex_syn2000" else "SynerexSyn3100_Connector")
                SynerexConnector = getattr(mod, connector_class)
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v2") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v2"
                self.synerex_rtk_connector = SynerexConnector(
                    robot_model=robot_model,
                    zpipe_ctx=self.zpipe_ctx,
                    on_data_received=self._on_rtk_data_received
                )
                self.synerex_rtk_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV2] Error initializing SynerexRTK_Connector: {e}")

        if "basler_gige_camera" in self.devices:
            try:
                mod = importlib.import_module("APROS.core.device.basler_gige_camera" if __name__.startswith("APROS") else "core.device.basler_gige_camera")
                CameraConnector = getattr(mod, "BaslerGigECamera_Connector")
                robot_model = self.config.get("PLATFORM", "robot_model", fallback="iae_patrol_v2") if self.config and self.config.has_section("PLATFORM") else "iae_patrol_v2"
                self.basler_camera_connector = CameraConnector(robot_model=robot_model, zpipe_ctx=self.zpipe_ctx,
                                                               on_frame_received=self._on_basler_camera_received)
                self.basler_camera_connector.start()
            except Exception as e:
                logger.error(f"[IAEPatrolV2] Error initializing BaslerGigECamera_Connector: {e}")

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
        if self.ouster_connector:
            try:
                self.ouster_connector.stop()
            except Exception:
                pass
            self.ouster_connector = None
        if self.synerex_rtk_connector:
            try:
                self.synerex_rtk_connector.stop()
            except Exception:
                pass
            self.synerex_rtk_connector = None
        if self.basler_camera_connector:
            try:
                self.basler_camera_connector.stop()
            except Exception:
                pass
            self.basler_camera_connector = None
        success = True
        for name, device in self.devices.items():
            try:
                success = bool(device.disconnect()) and success
            except Exception as exc:
                success = False
                logger.error(f"[IAEPatrolV2] Device '{name}' disconnect failed: {exc}")
        return success

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
            if self.simulation_mode:
                speed_m_s = float(getattr(self.drive_base, "speed", 0.0)) / 3.6
                heading = float(getattr(self.drive_base, "simulated_heading", self.simulated_heading))
                self.simulated_x += speed_m_s * math.cos(heading) * dt
                self.simulated_y += speed_m_s * math.sin(heading) * dt
                self.simulated_heading = heading
