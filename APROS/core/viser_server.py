"""
APROS Viser Visualization Server & UI Module.
Handles 3D visualization, robot box model (W:1000, L:2055, H:640 mm), CAN connection status display, and MobileDriveS1 control integration.
"""
import os
import time
import threading
import numpy as np
import viser
import viser.transforms as tf
from typing import Any, Optional
from resource.tile_server import TileServerManager
from core.data_logger import DataLogger
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class ViserServerManager:
    def __init__(self, host: str = "0.0.0.0", port: int = 8080, robot: Any = None):
        self.host = host
        self.port = port
        self.robot = robot
        if self.robot is not None and not getattr(self.robot, "is_connected", True):
            self.robot.connect()

        # Viser server
        self.server = viser.ViserServer(host=self.host, port=self.port)

        # Data Logger
        self._data_logger = DataLogger(robot=self.robot)

        # Title configuration & Theme setup (control_layout="fixed" docks panel to the right side)
        self.server.gui.configure_theme(
            titlebar_content=None,
            control_layout="fixed",
            control_width="large",
            dark_mode=True,
            show_logo=False,
            brand_color=(30, 144, 255)
        )
        self.server.gui.set_panel_label("APROS Control Center")

        # Tile Server for Leaflet JS/CSS and maptiles
        self.tile_server = TileServerManager(host=self.host, port=8082)
        self.tile_server.start()

        # Read platform IP from apros.cfg [PLATFORM] section
        self.platform_ip = "127.0.0.1"
        if hasattr(self.robot, 'config') and self.robot.config and self.robot.config.has_section("PLATFORM"):
            self.platform_ip = self.robot.config.get("PLATFORM", "ip", fallback="127.0.0.1")

        # Custom Top Titlebar Header & Floating Map Panel Window (Left: APROS, Right: Map Window Button)
        titlebar_html = f"""
        <div id="apros-top-titlebar" style="
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 48px;
            z-index: 10000;
            background: rgba(15, 20, 32, 0.95);
            border-bottom: 1px solid rgba(30, 144, 255, 0.3);
            backdrop-filter: blur(10px);
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 0 20px;
            box-sizing: border-box;
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
        ">
            <!-- Left: APROS Brand Title -->
            <div style="display: flex; align-items: center; gap: 10px;">
                <span style="
                    color: #00E676;
                    font-weight: 900;
                    font-size: 20px;
                    letter-spacing: 1.5px;
                    text-shadow: 0 0 10px rgba(0,230,118,0.4);
                ">APROS</span>
                <span style="color: #78909C; font-size: 13px; font-weight: 500;">| Autonomous Patrol Robot Operating System</span>
            </div>

            <!-- Right: Map Window Button -->
            <button id="apros-titlebar-btn" onclick="
                var modal = document.getElementById('apros-custom-modal');
                if(modal) {{
                    modal.style.display = (modal.style.display === 'none' || !modal.style.display) ? 'flex' : 'none';
                }}
            " style="
                background: linear-gradient(135deg, #1E90FF, #00E676);
                border: none;
                color: #FFFFFF;
                font-weight: bold;
                font-size: 13px;
                padding: 7px 16px;
                border-radius: 6px;
                cursor: pointer;
                box-shadow: 0 4px 12px rgba(0, 230, 118, 0.25);
                transition: all 0.2s ease;
            " onmouseover="this.style.opacity='0.9'" onmouseout="this.style.opacity='1.0'">
                🗺️ Map Window
            </button>
        </div>

        <!-- Custom Floating Map Panel (Embedded map.html via Configured Platform IP) -->
        <div id="apros-custom-modal" style="
            position: fixed;
            bottom: 25px;
            left: 25px;
            width: 560px;
            height: 400px;
            z-index: 15000;
            background: rgba(18, 24, 38, 0.96);
            border: 1px solid rgba(0, 230, 118, 0.6);
            border-radius: 12px;
            box-shadow: 0 10px 32px rgba(0, 0, 0, 0.75);
            backdrop-filter: blur(10px);
            display: none;
            flex-direction: column;
            overflow: hidden;
            resize: both;
        ">

            <!-- Window Header (Draggable Handle & Controls) -->
            <div id="apros-modal-header" style="
                padding: 10px 16px;
                background: rgba(255, 255, 255, 0.05);
                border-bottom: 1px solid rgba(255, 255, 255, 0.1);
                display: flex;
                justify-content: space-between;
                align-items: center;
                cursor: move;
                user-select: none;
            ">
                <div style="display: flex; align-items: center; gap: 8px;">
                    <span style="color: #00E676; font-weight: bold; font-size: 14px;">🗺️ Map Panel</span>
                    <span id="rtk-status-badge" style="font-size: 11px; color: #FFD700; background: rgba(255,215,0,0.15); padding: 2px 8px; border-radius: 4px; font-weight: 600;">연결 중 (Connecting)...</span>
                </div>
                <!-- Close Button -->
                <button onclick="document.getElementById('apros-custom-modal').style.display='none'" title="닫기" style="
                    background: transparent;
                    border: none;
                    color: #FF5252;
                    font-weight: bold;
                    font-size: 16px;
                    cursor: pointer;
                    padding: 0 4px;
                    border-radius: 4px;
                    transition: background 0.2s;
                " onmouseover="this.style.background='rgba(255,82,82,0.2)'" onmouseout="this.style.background='transparent'">✕</button>
            </div>

            <!-- Leaflet Map Container (Embedded map.html using configured IP: {self.platform_ip}) -->
            <div style="position: relative; flex: 1; width: 100%; height: 100%;">
                <iframe id="apros-map-iframe" style="width: 100%; height: 100%; border: none; background: #10141f;" src="http://{self.platform_ip}:8082/resource/map.html"></iframe>
            </div>
        </div>

        <script>
        (function() {{
            function relocate() {{
                var tb = document.getElementById('apros-top-titlebar');
                var md = document.getElementById('apros-custom-modal');
                if (tb && tb.parentElement !== document.body) document.body.appendChild(tb);
                if (md && md.parentElement !== document.body) document.body.appendChild(md);

                if (md) {{
                    var header = document.getElementById('apros-modal-header');
                    if (header && !md.dataset.dragInit) {{
                        md.dataset.dragInit = "true";
                        var isDragging = false, startX, startY, initialLeft, initialTop;
                        header.addEventListener('mousedown', function(e) {{
                            // Disable drag when clicked on header buttons
                            if (e.target.tagName === 'BUTTON') return;

                            isDragging = true;
                            startX = e.clientX;
                            startY = e.clientY;
                            var rect = md.getBoundingClientRect();
                            initialLeft = rect.left;
                            initialTop = rect.top;
                            md.style.right = 'auto';
                            md.style.bottom = 'auto';
                            md.style.left = initialLeft + 'px';
                            md.style.top = initialTop + 'px';
                        }});
                        document.addEventListener('mousemove', function(e) {{
                            if (!isDragging) return;
                            md.style.left = (initialLeft + (e.clientX - startX)) + 'px';
                            md.style.top = (initialTop + (e.clientY - startY)) + 'px';
                        }});
                        document.addEventListener('mouseup', function() {{ isDragging = false; }});
                    }}
                }}
            }}

            relocate();
            setTimeout(relocate, 50);
            setTimeout(relocate, 300);
        }})();
        </script>

        <style>
            /* Adjust body & root top padding for custom 48px TitleBar */
            body, #root, main {{
                padding-top: 48px !important;
            }}
            /* Hide default empty header if any */
            header, [class*="mantine-Header-root"] {{
                display: none !important;
            }}
        </style>
        """
        self.server.gui.add_html(titlebar_html)

        # Robot dimensions in meters (default fallback: 1.000m W, 2.055m L, 0.640m H)
        self.robot_width = 1.000
        self.robot_length = 2.055
        self.robot_height = 0.640
        self.lookahead_distance = 3.0  # meters (default 3.0m)

        if hasattr(self.robot, 'config') and self.robot.config:
            if self.robot.config.has_section("PLATFORM"):
                platform_cfg = self.robot.config["PLATFORM"]
                self.robot_width = float(platform_cfg.get("robot_width", self.robot_width))
                self.robot_length = float(platform_cfg.get("robot_length", self.robot_length))
                self.robot_height = float(platform_cfg.get("robot_height", self.robot_height))
                if "lookahead_distance" in platform_cfg:
                    self.lookahead_distance = float(platform_cfg["lookahead_distance"])
            if self.robot.config.has_section("mobile_drive_s1"):
                drive_cfg = self.robot.config["mobile_drive_s1"]
                if "lookahead_distance" in drive_cfg:
                    self.lookahead_distance = float(drive_cfg["lookahead_distance"])

        # Also check instantiated mobile_drive_s1 device if available
        if hasattr(self.robot, 'devices') and "mobile_drive_s1" in self.robot.devices:
            drive_dev = self.robot.devices["mobile_drive_s1"]
            if hasattr(drive_dev, "lookahead_distance"):
                self.lookahead_distance = float(drive_dev.lookahead_distance)

        self._running = False
        self._thread = None

        # Setup initial 3D scene environment
        self._setup_scene()

        # Connect client event callback
        @self.server.on_client_connect
        def _(client: viser.ClientHandle):
            self._setup_client_ui(client)

    def _setup_scene(self):
        """Build environment scene with single grid, pure robot box model, and controllable frame axes."""
        # 1. Single Ground Grid (XY Plane in ROS frame: plane='xy')
        self.server.scene.add_grid(
            name="/ground_grid",
            width=50.0,
            height=50.0,
            plane="xy",
            cell_color=(80, 90, 100),
            section_color=(140, 150, 160)
        )

        # 2. World Axes Frame with X, Y, Z labels (Scene Tree show/hide support)
        # ROS Standard Frame: X=Forward (Red), Y=Left (Green), Z=Up (Blue)
        self.server.scene.add_frame(
            name="/world_axes",
            show_axes=True,
            axes_length=1.5,
            axes_radius=0.03,
            origin_radius=0.04
        )
        
        self.server.scene.add_label(
            name="/world_axes/label_x",
            text="X (Forward)",
            position=(1.7, 0.0, 0.0)
        )
        self.server.scene.add_label(
            name="/world_axes/label_y",
            text="Y (Left)",
            position=(0.0, 1.7, 0.0)
        )
        self.server.scene.add_label(
            name="/world_axes/label_z",
            text="Z (Up)",
            position=(0.0, 0.0, 1.7)
        )

        # 3. Load URDF Robot Model into Viser Scene
        urdf_file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "urdf", "iae_patrol_v1.urdf")
        self.urdf_model = None
        if os.path.exists(urdf_file_path):
            try:
                import pathlib
                from viser.extras import ViserUrdf
                self.urdf_model = ViserUrdf(
                    target=self.server,
                    urdf_or_path=pathlib.Path(urdf_file_path),
                    root_node_name="/robot"
                )
                logger.info(f"[ViserServerManager] Loaded URDF model from: {urdf_file_path}")
            except Exception as e:
                logger.error(f"[ViserServerManager] Failed to load URDF model: {e}")

        # Fallback Box Model if URDF is unavailable
        if self.urdf_model is None:
            self.server.scene.add_box(
                name="/robot/chassis",
                dimensions=(self.robot_length, self.robot_width, self.robot_height),
                color=(30, 144, 255),
                position=(0.0, 0.0, self.robot_height / 2.0)
            )

        # 4. Lookahead Distance Green Circle Outline on Ground (Center of Robot)
        num_segments = 64
        angles = np.linspace(0, 2 * np.pi, num_segments)
        circle_points = np.stack([
            self.lookahead_distance * np.cos(angles),
            self.lookahead_distance * np.sin(angles),
            np.full(num_segments, 0.005)
        ], axis=-1)

        self.server.scene.add_spline_catmull_rom(
            name="/robot/lookahead_circle",
            positions=circle_points,
            color=(0, 255, 0),
            line_width=1.0,
            closed=True
        )

        # 5. VLP-16 Point Cloud visualization attached to /robot/visual/base_link/vlp16_link frame (URDF Link Frame)
        self.vlp16_pc_handle = self.server.scene.add_point_cloud(
            name="/robot/visual/base_link/vlp16_link/points",
            points=np.zeros((1, 3), dtype=np.float32),
            colors=np.array([[255, 0, 0]], dtype=np.uint8),
            point_size=0.015,
            point_shape="circle"
        )

        # 6. Ouster-SR-128 Point Cloud visualization attached to Mission Module ouster_link frame (URDF Link Frame)
        # Tree path: base_link -> mast_stage1_link -> mast_stage2_link -> ... -> mast_stage6_link -> mission_pan_link -> mission_module_link -> ouster_link
        ouster_frame_path = "/robot/visual/base_link/mast_stage1_link/mast_stage2_link/mast_stage3_link/mast_stage4_link/mast_stage5_link/mast_stage6_link/mission_pan_link/mission_module_link/ouster_link/points"
        self.ouster_pc_handle = self.server.scene.add_point_cloud(
            name=ouster_frame_path,
            points=np.zeros((1, 3), dtype=np.float32),
            colors=np.array([[200, 80, 255]], dtype=np.uint8),
            point_size=0.015,
            point_shape="circle"
        )



    def _setup_client_ui(self, client: viser.ClientHandle):
        """Setup UI components for newly connected client."""
        # ROS Z-Up Camera Orientation
        client.camera.up_direction = (0.0, 0.0, 1.0)
        client.camera.position = (-6.0, -3.5, 3.5)
        client.camera.look_at = (3.0, 0.0, 0.0)

        # Create Tab Group for multiple GUI windows
        tabs = client.gui.add_tab_group()

        # Window 1: APROS Control Tab
        with tabs.add_tab("APROS Dashboard", viser.Icon.SETTINGS):
            # 1. Robot Drive Status Folder (Real-time Parsed CAN 0 Data)
            with client.gui.add_folder("🚘 Robot Drive Status"):
                robot_drive_status_md = client.gui.add_markdown(self._format_robot_drive_status_text())

            # 2. Remote Control & Mast GUI Folder (Integrated Remote Control & Mast)
            with client.gui.add_folder("🎮 Robot Remote Control"):
                # CAN Connection Status Display
                can_status_md = client.gui.add_markdown(self._format_can_status_text())

                speed_slider = client.gui.add_slider(
                    label="Target Speed (km/h)",
                    min=0.0,
                    max=20.0,
                    step=0.5,
                    initial_value=self.robot.speed
                )
                
                # Steering Angle Slider: -28 deg ~ +28 deg (Step 0.5 deg)
                steer_slider = client.gui.add_slider(
                    label="Steering Angle (deg)",
                    min=-28.0,
                    max=28.0,
                    step=0.5,
                    initial_value=self.robot.steer_angle
                )

                gear_dropdown = client.gui.add_dropdown(
                    label="Gear",
                    options=["P", "D", "N", "R"],
                    initial_value=self.robot.gear
                )

                # Telescopic Mast Control (Integrated: 1800mm ~ 8000mm)
                mast_dev = self.robot.devices.get("telescopic_mast") if hasattr(self.robot, "devices") else None
                init_mast_height = mast_dev.current_height_mm if mast_dev else 1800.0

                mast_slider = client.gui.add_slider(
                    label="Mast Height (mm)",
                    min=1800.0,
                    max=8000.0,
                    step=50.0,
                    initial_value=init_mast_height
                )

                @mast_slider.on_update
                def _(_):
                    if hasattr(self.robot, "devices") and "telescopic_mast" in self.robot.devices:
                        self.robot.devices["telescopic_mast"].target_height_mm = mast_slider.value



                # Control Mode Group Buttons (Remote, Auto)
                init_is_auto = self.robot.drive_mode.startswith("Auto")
                options = ("🤖 Auto", "🕹️ Remote") if init_is_auto else ("🕹️ Remote", "🤖 Auto")
                control_mode_group = client.gui.add_button_group(
                    label="Control Mode",
                    options=options
                )

                self._current_control_mode = "Auto" if init_is_auto else "Remote"
                _is_processing_mode_change = False

                def set_group_value_silently(val: str):
                    # Update internal value and notify client GUI without calling update_cb (avoids recursion)
                    control_mode_group._impl.value = val
                    client._websock_interface.queue_message(
                        viser._messages.GuiUpdateMessage(control_mode_group._impl.uuid, {"value": val})
                    )

                def apply_control_mode(selected_mode: str):
                    self._current_control_mode = selected_mode
                    print(f"[Control Mode Event] Mode applied: {selected_mode}")
                    mobile_drive_dev = self.robot.devices.get("mobile_drive_s1") if hasattr(self.robot, "devices") else None
                    if mobile_drive_dev and hasattr(mobile_drive_dev, "change_drive_mode"):
                        mobile_drive_dev.change_drive_mode(selected_mode)
                    elif hasattr(self.robot, "set_mode_remote") and selected_mode == "Remote":
                        self.robot.set_mode_remote()
                    elif hasattr(self.robot, "set_mode_auto") and selected_mode == "Auto":
                        self.robot.set_mode_auto()

                    # On mode transition, reset gear selection UI to P Gear as required
                    gear_dropdown.value = "P"
                    speed_slider.disabled = False
                    steer_slider.disabled = False
                    if selected_mode == "Remote":
                        set_group_value_silently("🕹️ Remote")
                    else:
                        set_group_value_silently("🤖 Auto")

                @control_mode_group.on_click
                def _(_):
                    nonlocal _is_processing_mode_change
                    if _is_processing_mode_change:
                        return
                    
                    val = control_mode_group.value
                    selected_mode = "Remote" if "Remote" in val or "Manual" in val else "Auto"
                    print(f"[Control Mode Event] Button clicked: {selected_mode} (Value: '{val}')")

                    if selected_mode == "Auto" and self._current_control_mode == "Remote":
                        _is_processing_mode_change = True
                        # Revert group button visual selection until user confirms
                        set_group_value_silently("🕹️ Remote")
                        modal = client.gui.add_modal("Auto 모드 전환 확인")
                        with modal:
                            client.gui.add_markdown("Auto 모드로 전환합니다.")
                            confirm_btn = client.gui.add_button("확인", color="green")
                            cancel_btn = client.gui.add_button("취소", color="red")

                            @confirm_btn.on_click
                            def _(_):
                                nonlocal _is_processing_mode_change
                                modal.close()
                                apply_control_mode("Auto")
                                _is_processing_mode_change = False

                            @cancel_btn.on_click
                            def _(_):
                                nonlocal _is_processing_mode_change
                                modal.close()
                                print("[Control Mode Event] Auto mode transition cancelled by user.")
                                _is_processing_mode_change = False
                    else:
                        apply_control_mode(selected_mode)

            # Basler GigE Camera Folder in APROS Control tab
            with client.gui.add_folder("📷 Basler GigE Camera Stream", expand_by_default=True):
                # Placeholder 320x240 image
                init_cam_img = np.zeros((240, 320, 3), dtype=np.uint8)
                init_cam_img[::20, :] = [30, 30, 45]
                init_cam_img[:, ::20] = [30, 30, 45]
                
                gui_cam_image = client.gui.add_image(
                    image=init_cam_img,
                    label="Live Camera Stream",
                    format="jpeg"
                )

            estop_button = client.gui.add_button(
                label="🚨 EMERGENCY STOP",
                color="red"
            )

        # Tab 2: Mission Control Tab (Native Viser GUI Window)
        with tabs.add_tab("Mission Control", viser.Icon.TARGET):
            with client.gui.add_folder("📌 Active Mission Overview", expand_by_default=True):
                mission_overview_md = client.gui.add_markdown("""
<div style="background: rgba(0, 176, 255, 0.08); padding: 10px; border-radius: 8px; border-left: 4px solid #00B0FF;">
    <div style="font-size: 11px; text-transform: uppercase; color: #88C0D0; letter-spacing: 0.5px; margin-bottom: 4px;">Active Mission</div>
    <div style="font-size: 14px; font-weight: 700; color: #FFFFFF;">Autonomous Patrol Path A</div>
    <div style="font-size: 12px; color: #FFD700; margin-top: 4px; font-weight: 600;">STATUS: RUNNING</div>
</div>
                """)

            with client.gui.add_folder("📊 Patrol Progress"):
                mission_progress_md = client.gui.add_markdown("""
<div style="background: rgba(255, 255, 255, 0.03); padding: 10px; border-radius: 8px; border: 1px solid rgba(255, 255, 255, 0.08);">
    <div style="font-size: 12px; font-weight: 600; color: #E0E0E0; margin-bottom: 6px;">Patrol Progress</div>
    <div style="width: 100%; height: 8px; background: rgba(255,255,255,0.1); border-radius: 4px; overflow: hidden; margin-bottom: 6px;">
        <div style="width: 35%; height: 100%; background: #00E676; border-radius: 4px;"></div>
    </div>
    <div style="display: flex; justify-content: space-between; font-size: 11px; color: #AAA;">
        <span>Waypoints: 4 / 12</span>
        <span>35% Completed</span>
    </div>
</div>
                """)

            with client.gui.add_folder("⚙️ Controls & Camera", expand_by_default=True):
                camera_btn = client.gui.add_button("📹 Camera Toggle")

            with client.gui.add_folder("📌 Patrol Route & Task Execution"):
                mission_select = client.gui.add_dropdown(
                    label="Mission Select",
                    options=["Autonomous Patrol Path A", "Perimeter Security Loop", "Waypoint Inspection B", "Return to Home Base"],
                    initial_value="Autonomous Patrol Path A"
                )
                start_mission_btn = client.gui.add_button("▶️ Start Mission", color="green")
                pause_mission_btn = client.gui.add_button("⏸️ Pause Mission", color="yellow")
                abort_mission_btn = client.gui.add_button("⏹️ Abort Mission", color="red")

            with client.gui.add_folder("📜 Real-time Mission Log", expand_by_default=True):
                mission_log_md = client.gui.add_markdown("""
<div style="font-size: 11px; font-family: monospace; color: #B0BEC5; line-height: 1.6; max-height: 180px; overflow-y: auto;">
    <div><span style="color: #666;">[09:00:12]</span> <span style="color: #00E676;">[INFO]</span> Mission 'Path A' started</div>
    <div><span style="color: #666;">[09:01:05]</span> <span style="color: #00B0FF;">[NAV]</span> Reached Waypoint #1</div>
    <div><span style="color: #666;">[09:01:42]</span> <span style="color: #00B0FF;">[NAV]</span> Reached Waypoint #2</div>
    <div><span style="color: #666;">[09:02:15]</span> <span style="color: #FFD700;">[WARN]</span> Minor obstacle detected; rerouting</div>
</div>
                """)

            # Data Logger Folder (bottom of Mission Control tab)
            with client.gui.add_folder("💾 Data Logger", expand_by_default=True):
                datalog_status_md = client.gui.add_markdown(
                    "**Status**: ⏹️ Stopped"
                )

                datalog_mode_group = client.gui.add_button_group(
                    "Data Log Control",
                    options=["⏺️ Record", "⏹️ Stop"]
                )

                @datalog_mode_group.on_click
                def _(event: viser.GuiEvent) -> None:
                    selected = datalog_mode_group.value
                    if selected == "⏺️ Record":
                        if not self._data_logger.is_recording:
                            self._data_logger.start_recording()
                    elif selected == "⏹️ Stop":
                        if self._data_logger.is_recording:
                            self._data_logger.stop_recording()

        # 2. Floating/Dockable Camera View Panel
        dummy_cam_img = np.zeros((480, 640, 3), dtype=np.uint8)
        dummy_cam_img[::30, :] = [40, 40, 50]
        dummy_cam_img[:, ::30] = [40, 40, 50]

        camera_html_content = """
            <div id="camera-floating-panel" style="
                position: fixed;
                bottom: 25px;
                left: 25px;
                width: 960px;
                height: 580px;
                z-index: 15000;
                background: rgba(18, 24, 38, 0.96);
                border: 1px solid rgba(0, 230, 118, 0.6);
                border-radius: 12px;
                box-shadow: 0 10px 32px rgba(0, 0, 0, 0.75);
                backdrop-filter: blur(10px);
                display: none;
                flex-direction: column;
                overflow: hidden;
            ">
                <div id="camera-floating-header" style="
                    padding: 8px 14px;
                    background: rgba(255, 255, 255, 0.05);
                    border-bottom: 1px solid rgba(255, 255, 255, 0.1);
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                    cursor: move;
                    user-select: none;
                ">
                    <div style="display: flex; align-items: center; gap: 8px;">
                        <span style="color: #00E676; font-weight: bold; font-size: 14px;">📹 Basler Camera Live Stream</span>
                        <span id="camera-status-badge" style="font-size: 11px; color: #00E676; background: rgba(0,230,118,0.15); padding: 2px 8px; border-radius: 4px; font-weight: 600;">LIVE</span>
                    </div>
                    <button onclick="document.getElementById('camera-floating-panel').style.display='none'" style="
                        background: transparent;
                        border: none;
                        color: #FF5252;
                        font-weight: bold;
                        font-size: 16px;
                        cursor: pointer;
                        padding: 0 4px;
                    ">✕</button>
                </div>
                <div style="position: relative; flex: 1; width: 100%; height: 100%; display: flex; justify-content: center; align-items: center; background: #080b12;">
                    <img id="camera-stream-image" src="" alt="Camera Stream" style="max-width: 100%; max-height: 100%; object-fit: contain; display: none;">
                    <div id="camera-placeholder" style="color: #888; font-size: 14px; text-align: center;">
                        📹 Waiting for Basler GigE Camera Stream...
                    </div>
                </div>
            </div>
            <script>
            (function() {
                var el = document.getElementById('camera-floating-panel');
                if (!el || el.dataset.dragInit) return;
                el.dataset.dragInit = "true";
                var header = el.firstElementChild;
                var isDragging = false, startX, startY, initialLeft, initialTop;
                header.addEventListener('mousedown', function(e) {
                    isDragging = true;
                    startX = e.clientX;
                    startY = e.clientY;
                    var rect = el.getBoundingClientRect();
                    initialLeft = rect.left;
                    initialTop = rect.top;
                    el.style.bottom = 'auto';
                    el.style.left = initialLeft + 'px';
                    el.style.top = initialTop + 'px';
                });
                document.addEventListener('mousemove', function(e) {
                    if (!isDragging) return;
                    var dx = e.clientX - startX;
                    var dy = e.clientY - startY;
                    el.style.left = (initialLeft + dx) + 'px';
                    el.style.top = (initialTop + dy) + 'px';
                });
                document.addEventListener('mouseup', function() { isDragging = false; });
            })();
            </script>
        """
        camera_panel_html = client.gui.add_html(camera_html_content)

        camera_folder = client.gui.add_folder("📹 Camera View (Docked)", visible=False)
        with camera_folder:
            client.gui.add_markdown("<div style='text-align: center; color: #00E676;'><b>📷 Live Camera Stream (Width: 640px)</b></div>")
            client.gui.add_image(dummy_cam_img, label="Front Camera Stream")

        # Camera Toggle Button Callback
        @camera_btn.on_click
        def _(_):
            camera_folder.visible = not camera_folder.visible
            # Also toggle floating panel DOM display
            toggle_js = """
            <script>
            (function() {
                var el = document.getElementById('camera-floating-panel');
                if (el) {
                    el.style.display = (el.style.display === 'none' || !el.style.display) ? 'block' : 'none';
                }
            })();
            </script>
            """
            client.gui.add_html(toggle_js)



        # Control Callbacks
        @speed_slider.on_update
        def _(_):
            if not speed_slider.disabled:
                self.robot.speed = speed_slider.value

        # Real-time CAN Steering Angle Control (-28 ~ +28 deg)
        @steer_slider.on_update
        def _(_):
            if not steer_slider.disabled:
                self.robot.set_steering_angle(steer_slider.value)

        @gear_dropdown.on_update
        def _(_):
            self.robot.gear = gear_dropdown.value
            mobile_drive_dev = self.robot.devices.get("mobile_drive_s1") if hasattr(self.robot, "devices") else None
            if mobile_drive_dev and hasattr(mobile_drive_dev, "target_gear"):
                mobile_drive_dev.target_gear = gear_dropdown.value

        def execute_emergency_stop():
            self.robot.speed = 0.0
            speed_slider.value = 0.0
            self.robot.set_steering_angle(0.0)
            steer_slider.value = 0.0
            self.robot.gear = "P"
            gear_dropdown.value = "P"
            self.robot.drive_mode = "Emergency Stop"
            speed_slider.disabled = True
            steer_slider.disabled = True
            control_mode_group.value = "🕹️ Remote"

        @estop_button.on_click
        def _(_):
            execute_emergency_stop()

        # Background update loop for UI Markdown refresh and Camera Stream
        def ui_update_loop():
            last_img_ts = 0.0
            while self._running:
                try:
                    robot_drive_status_md.content = self._format_robot_drive_status_text()
                    can_status_md.content = self._format_can_status_text()

                    # Update Data Logger status
                    try:
                        if self._data_logger.is_recording:
                            dur = self._data_logger.recording_duration
                            mins = int(dur // 60)
                            secs = int(dur % 60)
                            session = os.path.basename(self._data_logger.session_dir or "")
                            datalog_status_md.content = (
                                f"**Status**: 🔴 Recording (`{mins:02d}:{secs:02d}`)\n\n"
                                f"**Session**: `{session}`"
                            )
                        else:
                            datalog_status_md.content = "**Status**: ⏹️ Stopped"
                    except Exception:
                        pass

                    # Update live camera stream in GUI folder if frame updated
                    if hasattr(self.robot, "last_camera_frame") and self.robot.last_camera_frame is not None:
                        try:
                            frame_bytes = self.robot.last_camera_frame
                            cam_hdr = getattr(self.robot, "last_camera_header", {}) or {}
                            ts = cam_hdr.get("timestamp", 0.0)
                            if ts != last_img_ts:
                                last_img_ts = ts
                                if isinstance(frame_bytes, bytes) and len(frame_bytes) > 0:
                                    import cv2
                                    nparr = np.frombuffer(frame_bytes, np.uint8)
                                    decoded_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                                    if decoded_bgr is not None:
                                        decoded_rgb = cv2.cvtColor(decoded_bgr, cv2.COLOR_BGR2RGB)
                                        gui_cam_image.image = decoded_rgb
                        except Exception as cam_err:
                            logger.error(f"[ViserUI] Camera image update error: {cam_err}")

                    time.sleep(0.1)
                except Exception as e:
                    logger.error(f"[ViserUI] ui_update_loop error: {e}")
                    time.sleep(0.5)

        threading.Thread(target=ui_update_loop, daemon=True).start()

    def _format_robot_drive_status_text(self) -> str:
        lines = []

        drive_dev = self.robot.devices.get("mobile_drive_s1") if hasattr(self.robot, "devices") and self.robot.devices else None
        incline_dev = self.robot.devices.get("baumer_incline") if hasattr(self.robot, "devices") and self.robot.devices else None

        drive_status = drive_dev.get_status() if drive_dev and hasattr(drive_dev, "get_status") else {}
        parsed_can = drive_status.get("parsed_can_status", {}) if isinstance(drive_status.get("parsed_can_status"), dict) else {}
        incline_status = incline_dev.get_status() if incline_dev and hasattr(incline_dev, "get_status") else {}

        # 1. Msg ID 0x303 & 0x0A0
        drive_mode_state = parsed_can.get("drive_state_mode", drive_status.get("drive_mode", "N/A"))
        lines.append(f"- **Drive Mode State**: `{drive_mode_state}`")

        vehicle_gear = parsed_can.get("vehicle_gear", drive_status.get("gear", "P"))
        lines.append(f"- **Vehicle Gear**: `{vehicle_gear}`")

        bms_soc = parsed_can.get("bms_battery_soc", parsed_can.get("bms_battery_soc", "N/A"))
        lines.append(f"- **Battery SOC**: `{bms_soc}`")

        # 2. Msg ID 0x304
        steer_angle = parsed_can.get("vehicle_steer_angle", f"{drive_status.get('steer_angle', 0.0):.1f} deg")
        lines.append(f"- **Vehicle Steer Angle**: `{steer_angle}`")

        speed = parsed_can.get("vehicle_velocity", f"{drive_status.get('speed', 0.0):.1f} km/h")
        lines.append(f"- **Vehicle Speed**: `{speed}`")

        # 3. Msg ID 0x301
        emerg_button = parsed_can.get("emergency_button", "Not Pressed")
        lines.append(f"- **Emergency Button**: `{emerg_button}`")

        # 4. Baumer Incline
        tx = incline_status.get("tilt_x", 0.0)
        tz = incline_status.get("tilt_z", 0.0)
        lines.append(f"- **Tilt X**: `{tx:.2f}°`")
        lines.append(f"- **Tilt Z**: `{tz:.2f}°`")

        return "\n".join(lines)

    def _format_mission_center_text(self) -> str:
        return (
            "### Autonomous Mission Planner\n"
            "- **Current Mission**: `Patrol Route Alpha`\n"
            "- **Status**: `STANDBY / READY`\n"
            "- **Waypoints Progress**: `0 / 12 Completed`"
        )

    def _format_can_status_text(self) -> str:
        status = self.robot.get_status()
        is_conn = status.get("connected", False)
        channel = status.get("channel", "can0")
        cmd_val = status.get("can_cmd_val", 0)
        
        status_str = f"**ONLINE** (`{channel}`)" if is_conn else f"**OFFLINE** (`{channel}`)"
        return f"- **CAN Bus Status**: {status_str}\n- **Current Steering Cmd**: `{cmd_val}` (-2000: L / +2000: R)"

    def _format_dashboard_text(self) -> str:
        status = self.robot.get_status() if hasattr(self.robot, "get_status") else {}
        speed = status.get("speed", getattr(self.robot, "speed", 0.0))
        steer = status.get("steer_angle", getattr(self.robot, "steer_angle", 0.0))
        lat = status.get("latitude", getattr(self.robot, "latitude", 37.5665))
        lon = status.get("longitude", getattr(self.robot, "longitude", 126.9780))
        gear = status.get("gear", getattr(self.robot, "gear", "P"))
        mode = status.get("drive_mode", getattr(self.robot, "drive_mode", "Manual"))

        w_mm = int(self.robot_width * 1000)
        l_mm = int(self.robot_length * 1000)
        h_mm = int(self.robot_height * 1000)

        return (
            f"### APROS Patrol Robot Status\n"
            f"- **Speed**: `{speed:.1f} km/h`\n"
            f"- **Steer Angle**: `{steer:.1f}°`\n"
            f"- **Position**: `N {lat:.6f}°, E {lon:.6f}°`\n"
            f"- **Gear**: `{gear}` | **Mode**: `{mode}`\n"
            f"- **Dimensions**: `{w_mm} × {l_mm} × {h_mm} mm`"
        )

    def _simulation_loop(self):
        """Update robot kinematics for driving simulation and render real-time VLP-16 point cloud."""
        dt = 0.05
        t = 0.0
        while self._running:
            start_time = time.time()
            t += dt

            # Update simulated physics/kinematics in device controller
            self.robot.update_simulation_step(dt=dt)

            # Update URDF Joint States (Mast height & Steering angle)
            mast_height_m = 1.8
            if hasattr(self.robot, "devices") and "telescopic_mast" in self.robot.devices:
                mast = self.robot.devices["telescopic_mast"]
                mast_height_m = mast.current_height_m

            steer_angle_rad = np.radians(getattr(self.robot, "steer_angle", 0.0))

            if self.urdf_model is not None:
                # Master mast_joint (0.0m ~ 6.2m) & master front_steer_joint update
                mast_extension = max(0.0, mast_height_m - 1.8)
                self.urdf_model.update_cfg({
                    "mast_joint": mast_extension,
                    "front_steer_joint": steer_angle_rad,
                })

            # Real-time VLP-16 point cloud visualization centered at robot frame
            if hasattr(self.robot, 'last_vlp16_points') and self.robot.last_vlp16_points is not None:
                pts = self.robot.last_vlp16_points
                if len(pts) > 0:
                    xyz = np.ascontiguousarray(pts[:, :3], dtype=np.float32)
                    colors = np.zeros((len(pts), 3), dtype=np.uint8)

                    # If ground flag column (5th column) exists
                    if pts.shape[1] >= 5:
                        is_ground = (pts[:, 4] > 0.5)
                        # Ground points: White (255, 255, 255)
                        colors[is_ground] = [255, 255, 255]
                        # Obstacle / Non-ground points: Red (255, 0, 0)
                        colors[~is_ground] = [255, 0, 0]
                    else:
                        colors[:, 0] = 255  # Red channel default
                    colors = np.ascontiguousarray(colors, dtype=np.uint8)
                else:
                    xyz = np.empty((0, 3), dtype=np.float32)
                    colors = np.empty((0, 3), dtype=np.uint8)

                self.vlp16_pc_handle.points = xyz
                self.vlp16_pc_handle.colors = colors

            # Real-time Ouster-SR-128 point cloud visualization
            if hasattr(self.robot, 'last_ouster_points') and self.robot.last_ouster_points is not None:
                pts = self.robot.last_ouster_points
                if len(pts) > 0:
                    xyz = np.ascontiguousarray(pts[:, :3], dtype=np.float32)
                    # Bright purple color for Ouster points (200, 80, 255)
                    colors = np.zeros((len(pts), 3), dtype=np.uint8)
                    colors[:, 0] = 200  # Red channel
                    colors[:, 1] = 80   # Green channel
                    colors[:, 2] = 255  # Blue channel
                    colors = np.ascontiguousarray(colors, dtype=np.uint8)

                    self.ouster_pc_handle.points = xyz
                    self.ouster_pc_handle.colors = colors

            elapsed = time.time() - start_time
            time.sleep(max(0.0, dt - elapsed))

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self._thread.start()
        logger.info(f"[APROS Viser UI] Server started on http://{self.host}:{self.port}")


    def stop(self):
        self._running = False
        if self._data_logger and self._data_logger.is_recording:
            self._data_logger.stop_recording()
        if hasattr(self, 'tile_server') and self.tile_server:
            self.tile_server.stop()
        if self._thread:
            self._thread.join(timeout=1.0)
