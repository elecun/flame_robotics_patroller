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

        if hasattr(self.robot, 'config') and self.robot.config and self.robot.config.has_section("PLATFORM"):
            platform_cfg = self.robot.config["PLATFORM"]
            self.robot_width = float(platform_cfg.get("robot_width", self.robot_width))
            self.robot_length = float(platform_cfg.get("robot_length", self.robot_length))
            self.robot_height = float(platform_cfg.get("robot_height", self.robot_height))
            self.lookahead_distance = float(platform_cfg.get("lookahead_distance", self.lookahead_distance))

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

    def _setup_client_ui(self, client: viser.ClientHandle):
        """Setup UI components for newly connected client."""
        # ROS Z-Up Camera Orientation
        client.camera.up_direction = (0.0, 0.0, 1.0)
        client.camera.position = (-6.0, -3.5, 3.5)
        client.camera.look_at = (3.0, 0.0, 0.0)

        # Create Tab Group for multiple GUI windows
        tabs = client.gui.add_tab_group()

        # Window 1: APROS Control Tab
        with tabs.add_tab("APROS Control", viser.Icon.SETTINGS):
            # 1. Left Telemetry Dashboard
            with client.gui.add_folder("📊 Telemetry Dashboard (APROS System)"):
                dashboard_md = client.gui.add_markdown(self._format_dashboard_text())

            # 2. Robot Drive Status Folder (Real-time Parsed CAN 0 Data)
            with client.gui.add_folder("🚘 Robot Drive Status"):
                robot_drive_status_md = client.gui.add_markdown(self._format_robot_drive_status_text())

            # 3. Remote Control GUI Folder
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

                # Control Mode buttons: Manual and Auto
                client.gui.add_markdown("<span style='font-size: 13px; font-weight: 600; color: #E0E0E0;'>Control Mode:</span>")
                manual_btn = client.gui.add_button("🕹️ Manual")
                auto_btn = client.gui.add_button("🤖 Auto")

            # 4. Telescopic Mast Control Folder (1800mm ~ 8000mm)
            with client.gui.add_folder("🏗️ Telescopic Mast Control"):
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

            estop_button = client.gui.add_button(
                label="🚨 EMERGENCY STOP (P Gear & STOP)",
                color="red"
            )

        # Tab 2: Mission Monitor Tab (Native Viser GUI Window)
        with tabs.add_tab("Mission Monitor", viser.Icon.TARGET):
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

        # 2. Floating/Dockable Camera View Panel
        dummy_cam_img = np.zeros((480, 640, 3), dtype=np.uint8)
        dummy_cam_img[::30, :] = [40, 40, 50]
        dummy_cam_img[:, ::30] = [40, 40, 50]

        camera_html_content = """
            <div id="camera-floating-panel" style="
                position: fixed;
                bottom: 25px;
                left: 25px;
                width: 640px;
                height: 480px;
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

        # Mode switching cooldown and button state management
        pending_mode_transition: Optional[str] = None  # "Manual" or "Auto"
        transition_start_time: float = 0.0

        @manual_btn.on_click
        def _(_):
            nonlocal pending_mode_transition, transition_start_time
            if hasattr(self.robot, "set_mode_manual"):
                self.robot.set_mode_manual()
            self.robot.drive_mode = "Manual (Remote)"
            speed_slider.disabled = False
            steer_slider.disabled = False

            # Lock buttons for 10-second transition period
            manual_btn.disabled = True
            auto_btn.disabled = True
            pending_mode_transition = "Manual"
            transition_start_time = time.time()

        @auto_btn.on_click
        def _(_):
            nonlocal pending_mode_transition, transition_start_time
            if hasattr(self.robot, "set_mode_auto"):
                self.robot.set_mode_auto()
            self.robot.drive_mode = "Auto (Autonomous)"
            speed_slider.disabled = True
            steer_slider.disabled = True

            # Lock buttons for 10-second transition period
            manual_btn.disabled = True
            auto_btn.disabled = True
            pending_mode_transition = "Auto"
            transition_start_time = time.time()

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

        @estop_button.on_click
        def _(_):
            execute_emergency_stop()

        # Background update loop for UI Markdown refresh and Mode Button states
        def ui_update_loop():
            nonlocal pending_mode_transition, transition_start_time
            while self._running:
                try:
                    dashboard_md.content = self._format_dashboard_text()
                    robot_drive_status_md.content = self._format_robot_drive_status_text()
                    can_status_md.content = self._format_can_status_text()

                    # Check CAN Drive_State_Mode
                    status = self.robot.get_status()
                    parsed_can = status.get("parsed_can_status", {})
                    current_drive_state = parsed_can.get("Drive_State_Mode", "")

                    now = time.time()
                    # Check if transition target was achieved early
                    if pending_mode_transition == "Manual" and (current_drive_state == "Remote Control Mode" or "Remote" in current_drive_state):
                        pending_mode_transition = None
                    elif pending_mode_transition == "Auto" and (current_drive_state == "Represents the AD Mode" or "AD" in current_drive_state):
                        pending_mode_transition = None
                    elif pending_mode_transition is not None and (now - transition_start_time >= 10.0):
                        # Timeout 10s elapsed
                        pending_mode_transition = None

                    # If not in transition, set button enabled/disabled states based on Drive_State_Mode
                    if pending_mode_transition is None:
                        if current_drive_state == "Remote Control Mode" or "Remote" in current_drive_state:
                            # In Remote Control Mode -> Auto button is ENABLED, Manual button is DISABLED
                            auto_btn.disabled = False
                            manual_btn.disabled = True
                        elif current_drive_state == "Represents the AD Mode" or "AD" in current_drive_state:
                            # In AD Mode -> Manual button is ENABLED, Auto button is DISABLED
                            manual_btn.disabled = False
                            auto_btn.disabled = True
                        else:
                            # Default fallback if CAN telemetry is absent
                            if self.robot.drive_mode.startswith("Manual"):
                                auto_btn.disabled = False
                                manual_btn.disabled = True
                            else:
                                manual_btn.disabled = False
                                auto_btn.disabled = True

                    time.sleep(0.1)
                except Exception:
                    break

        threading.Thread(target=ui_update_loop, daemon=True).start()

    def _format_robot_drive_status_text(self) -> str:
        lines = []

        if hasattr(self.robot, "devices") and self.robot.devices:
            for dev_name, dev_obj in self.robot.devices.items():
                is_connected = getattr(dev_obj, "is_connected", False)
                enable = getattr(dev_obj, "enable", True)

                if not enable:
                    status_str = "`DISABLED`"
                elif is_connected:
                    status_str = "`ONLINE`"
                else:
                    status_str = "`OFFLINE (UNAVAILABLE)`"

                lines.append(f"- **{dev_name}**: {status_str}")

                # Display detailed data if device is connected & enabled
                if enable and is_connected and hasattr(dev_obj, "get_status"):
                    try:
                        dev_status = dev_obj.get_status()
                        if dev_name == "baumer_incline":
                            tx = dev_status.get("tilt_x", 0.0)
                            tz = dev_status.get("tilt_z", 0.0)
                            lines.append(f"  └ Tilt X: `{tx:.1f}°`, Tilt Z: `{tz:.1f}°`")
                        elif dev_name == "telescopic_mast":
                            h_m = dev_status.get("current_height_m", 1.8)
                            lines.append(f"  └ Mast Height: `{h_m:.2f} m`")
                        elif dev_name == "basler_gige_camera":
                            fps = dev_status.get("fps", 15)
                            cnt = dev_status.get("frame_count", 0)
                            lines.append(f"  └ Target FPS: `{fps}`, Frames: `{cnt}`")
                        elif dev_name == "synerex_rtk":
                            lat = dev_status.get("latitude", 0.0)
                            lon = dev_status.get("longitude", 0.0)
                            lines.append(f"  └ GNSS: `{lat:.6f}, {lon:.6f}`")
                    except Exception:
                        pass
        else:
            lines.append("No configured devices found.")

        # Real-time CAN 0 parsed status
        status = self.robot.get_status()
        parsed_can = status.get("parsed_can_status", {})
        if parsed_can:
            lines.append("\n**CAN Drive Telemetry**")
            for key, val in parsed_can.items():
                lines.append(f"- **{key}**: `{val}`")

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
                    xyz = pts[:, :3]
                    # Solid Red color for all points (255, 0, 0)
                    colors = np.zeros((len(pts), 3), dtype=np.uint8)
                    colors[:, 0] = 255  # Red channel

                    self.vlp16_pc_handle.points = xyz
                    self.vlp16_pc_handle.colors = colors

            elapsed = time.time() - start_time
            time.sleep(max(0.0, dt - elapsed))

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self._thread.start()
        logger.info(f"[APROS Viser UI] Server started on http://{self.host}:{self.port}")


    def stop(self):
        self._running = False
        if hasattr(self, 'tile_server') and self.tile_server:
            self.tile_server.stop()
        if self._thread:
            self._thread.join(timeout=1.0)
