"""
APROS Viser Visualization Server & UI Module.
Handles 3D visualization, robot box model (W:1000, L:2055, H:640 mm), CAN connection status display, and MobileDriveS1 control integration.
"""
import time
import threading
import numpy as np
import viser
import viser.transforms as tf
from core.device.mobile_drive_s1 import MobileDriveS1
from resource.tile_server import TileServerManager
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class ViserServerManager:
    def __init__(self, host: str = "0.0.0.0", port: int = 8080, robot: MobileDriveS1 = None):
        self.host = host
        self.port = port
        self.robot = robot if robot is not None else MobileDriveS1()
        if not self.robot.is_connected:
            self.robot.connect()

        # Viser server
        self.server = viser.ViserServer(host=self.host, port=self.port)

        # Title configuration & Theme setup
        self.server.gui.configure_theme(
            titlebar_content=None,
            control_layout="floating",
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
                    if (window.aprosMap) setTimeout(function(){{ window.aprosMap.invalidateSize(); }}, 200);
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
                🗺️ 2D Map
            </button>
        </div>

        <!-- Leaflet CSS & JS injected into head -->
        <script>
        (function() {{
            if (!document.getElementById('leaflet-css')) {{
                var link = document.createElement('link');
                link.id = 'leaflet-css';
                link.rel = 'stylesheet';
                link.href = 'http://localhost:8082/resource/leaflet.css';
                document.head.appendChild(link);
            }}
            if (!document.getElementById('leaflet-js')) {{
                var script = document.createElement('script');
                script.id = 'leaflet-js';
                script.src = 'http://localhost:8082/resource/leaflet.js';
                document.head.appendChild(script);
            }}
        }})();
        </script>

        <!-- Custom Floating Map Panel (Bottom-Left, Draggable, Leaflet Map Viewer) -->
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

            <!-- Window Header (Draggable Handle) -->
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
                <button onclick="document.getElementById('apros-custom-modal').style.display='none'" style="
                    background: transparent;
                    border: none;
                    color: #FF5252;
                    font-weight: bold;
                    font-size: 16px;
                    cursor: pointer;
                    padding: 0 4px;
                ">✕</button>
            </div>

            <!-- Leaflet Map Container (Embedded map.html using configured IP: {self.platform_ip}) -->
            <div style="position: relative; flex: 1; width: 100%; height: 100%;">
                <iframe id="apros-map-iframe" style="width: 100%; height: 100%; border: none; background: #10141f;" src="http://{self.platform_ip}:8082/resource/map.html"></iframe>
            </div>
        </div>

        <script>
        (function() {{
            var win = document.getElementById('apros-custom-modal');
            var header = document.getElementById('apros-modal-header');
            if (!win || !header || win.dataset.dragInit) return;
            win.dataset.dragInit = "true";

            // Window Dragging logic
            var isDragging = false, startX, startY, initialLeft, initialTop;
            header.addEventListener('mousedown', function(e) {{
                isDragging = true;
                startX = e.clientX;
                startY = e.clientY;
                var rect = win.getBoundingClientRect();
                initialLeft = rect.left;
                initialTop = rect.top;
                win.style.right = 'auto';
                win.style.bottom = 'auto';
                win.style.left = initialLeft + 'px';
                win.style.top = initialTop + 'px';
            }});
            document.addEventListener('mousemove', function(e) {{
                if (!isDragging) return;
                var dx = e.clientX - startX;
                var dy = e.clientY - startY;
                win.style.left = (initialLeft + dx) + 'px';
                win.style.top = (initialTop + dy) + 'px';
            }});
            document.addEventListener('mouseup', function() {{ isDragging = false; }});
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

        # 3. Main Patrol Robot Model in ROS Frame (X: Length 2.055m, Y: Width 1.0m, Z: Height 0.64m)
        self.server.scene.add_box(
            name="/robot/chassis",
            dimensions=(self.robot_length, self.robot_width, self.robot_height),
            color=(30, 144, 255),  # Sleek Blue
            position=(0.0, 0.0, self.robot_height / 2.0)
        )

        # Front Heading Bumper Indicator (Located along +X Front axis)
        self.server.scene.add_box(
            name="/robot/heading_indicator",
            dimensions=(0.2, self.robot_width * 0.8, 0.08),
            color=(255, 69, 0),
            position=(self.robot_length / 2.0 - 0.15, 0.0, self.robot_height + 0.04)
        )

        # VLP-16 Cylinder Model in ROS Frame (X: Forward, Y: Left, Z: Up):
        # Dynamically load installation offset and orientation from config if available
        vlp16_radius = 0.1033 / 2.0  # 0.05165 m
        vlp16_height = 0.0717        # 0.0717 m

        vlp16_offset_x = 0.64   # ROS X (Forward)
        vlp16_offset_y = 0.0    # ROS Y (Left)
        vlp16_offset_z = 1.027  # ROS Z (Up)
        vlp16_pitch_deg = 15.0  # Pitch angle around Y axis

        if hasattr(self.robot, 'config') and self.robot.config and self.robot.config.has_section("vlp-16"):
            cfg = self.robot.config["vlp-16"]
            vlp16_offset_x = float(cfg.get("offset_x", vlp16_offset_x))
            vlp16_offset_y = float(cfg.get("offset_y", vlp16_offset_y))
            vlp16_offset_z = float(cfg.get("offset_z", vlp16_offset_z))
            vlp16_pitch_deg = float(cfg.get("pitch_deg", vlp16_pitch_deg))

        # Cylinder rotation: Viser cylinder axis defaults to +Z (matches ROS Z-Up axis directly!)
        # Pitch angle tilt around ROS Y axis
        R_vlp16 = tf.SO3.from_y_radians(np.radians(vlp16_pitch_deg)).wxyz

        self.server.scene.add_cylinder(
            name="/robot/vlp16_sensor",
            radius=vlp16_radius,
            height=vlp16_height,
            color=(50, 50, 50),  # Dark Grey Housing
            position=(vlp16_offset_x, vlp16_offset_y, vlp16_offset_z),
            wxyz=R_vlp16
        )

        # 4. Telescopic Mast 3D Model (Fixed Diameter 100mm = 0.1m)
        # Position: Positioned based on offset_x, offset_y, offset_z from robot origin frame
        self.mast_radius = 0.100 / 2.0  # 100mm diameter (0.05m radius)
        mast_offset_x = 0.0
        mast_offset_y = 0.0
        mast_offset_z = self.robot_height  # 0.64m

        if hasattr(self.robot, 'config') and self.robot.config and self.robot.config.has_section("telescopic_mast"):
            cfg = self.robot.config["telescopic_mast"]
            mast_offset_x = float(cfg.get("offset_x", mast_offset_x))
            mast_offset_y = float(cfg.get("offset_y", mast_offset_y))
            mast_offset_z = float(cfg.get("offset_z", mast_offset_z))

        self.mast_offset_x = mast_offset_x
        self.mast_offset_y = mast_offset_y
        self.mast_offset_z = mast_offset_z

        self.mast_handle = self.server.scene.add_cylinder(
            name="/robot/telescopic_mast",
            radius=self.mast_radius,
            height=1.8,  # Default 1800mm (1.8m)
            color=(20, 120, 50),  # Dark Green
            position=(self.mast_offset_x, self.mast_offset_y, self.mast_offset_z + 1.8 / 2.0)
        )

        # 5. Lookahead Distance Green Circle Outline on Ground (Center of Robot)
        # Generate 64-segment circle outline points on ground (Z=0.005m)
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
            color=(0, 255, 0),  # Pure Green
            line_width=1.0,
            closed=True
        )

        # 6. VLP-16 Point Cloud visualization attached to /robot/vlp16_sensor frame
        # Viser will automatically position and orient points relative to the sensor origin & pitch angle.
        self.vlp16_pc_handle = self.server.scene.add_point_cloud(
            name="/robot/vlp16_sensor/points",
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
                mode_dropdown = client.gui.add_dropdown(
                    label="Control Mode",
                    options=["Manual (Remote)", "Auto (Autonomous)", "Emergency Stop"],
                    initial_value="Manual (Remote)" if self.robot.drive_mode.startswith("Manual") else self.robot.drive_mode
                )

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
                # Initial disabled state based on Control Mode (Manual enables, Auto/E-stop disables)
                is_manual = mode_dropdown.value.startswith("Manual")
                speed_slider.disabled = not is_manual
                steer_slider.disabled = not is_manual

            estop_button = client.gui.add_button(
                label="🚨 EMERGENCY STOP (P Gear & STOP)",
                color="red"
            )

        # Window 2: Mission Control Tab
        with tabs.add_tab("Mission Control", viser.Icon.MAP_PIN):
            # 1. Top Panel Folder & Controls
            panel_folder = client.gui.add_folder("⚙️ Panel", expand_by_default=True)
            with panel_folder:
                camera_btn = client.gui.add_button("📹 Camera Toggle")


            mission_status_md = client.gui.add_markdown(self._format_mission_center_text())
            with client.gui.add_folder("📌 Patrol Route & Task Execution"):
                mission_select = client.gui.add_dropdown(
                    label="Mission Select",
                    options=["Autonomous Patrol Path A", "Perimeter Security Loop", "Waypoint Inspection B", "Return to Home Base"],
                    initial_value="Autonomous Patrol Path A"
                )
                start_mission_btn = client.gui.add_button("▶️ Start Mission", color="green")
                pause_mission_btn = client.gui.add_button("⏸️ Pause Mission", color="yellow")
                abort_mission_btn = client.gui.add_button("⏹️ Abort Mission", color="red")

            # 2. Floating/Dockable Camera View Panel (Bottom-Left, Width: 640px, Toggle via Camera button)
            dummy_cam_img = np.zeros((480, 640, 3), dtype=np.uint8)
            # Add grid pattern graphics to dummy camera feed
            dummy_cam_img[::30, :] = [40, 40, 50]
            dummy_cam_img[:, ::30] = [40, 40, 50]

            camera_html_content = """
            <div id="camera-floating-panel" style="
                position: fixed;
                bottom: 20px;
                left: 20px;
                width: 640px;
                z-index: 9999;
                background: rgba(18, 24, 38, 0.92);
                border: 1px solid rgba(0, 230, 118, 0.4);
                border-radius: 10px;
                box-shadow: 0 8px 24px rgba(0,0,0,0.6);
                backdrop-filter: blur(8px);
                padding: 12px;
                box-sizing: border-box;
                font-family: sans-serif;
                display: none;
                resize: both;
                overflow: hidden;
            ">
                <div style="
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                    margin-bottom: 8px;
                    padding-bottom: 6px;
                    border-bottom: 1px solid rgba(255,255,255,0.1);
                    cursor: move;
                ">
                    <span style="color: #00E676; font-weight: bold; font-size: 14px;">📹 Live Front IP Camera Feed (640px)</span>
                    <button onclick="document.getElementById('camera-floating-panel').style.display='none'" style="
                        background: transparent;
                        border: none;
                        color: #FF5252;
                        font-weight: bold;
                        cursor: pointer;
                        font-size: 16px;
                    ">✕</button>
                </div>
                <div style="width: 100%; height: auto; text-align: center;">
                    <img src="data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='616' height='360' viewBox='0 0 616 360'><rect width='100%' height='100%' fill='%2310141f'/><grid/><text x='50%' y='45%' fill='%2300E676' font-family='sans-serif' font-size='18' text-anchor='middle'>📷 FRONT IP CAMERA LIVE STREAM (640px)</text><text x='50%' y='55%' fill='%23888' font-family='sans-serif' font-size='14' text-anchor='middle'>Resolution: 1920x1080 @ 30fps | Stream: RTSP/H.264</text></svg>" style="width: 100%; border-radius: 6px;" />
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

        @mode_dropdown.on_update
        def _(_):
            mode_val = mode_dropdown.value
            self.robot.drive_mode = mode_val
            is_manual = mode_val.startswith("Manual")
            speed_slider.disabled = not is_manual
            steer_slider.disabled = not is_manual

        def execute_emergency_stop():
            self.robot.speed = 0.0
            speed_slider.value = 0.0
            self.robot.set_steering_angle(0.0)
            steer_slider.value = 0.0
            self.robot.gear = "P"
            gear_dropdown.value = "P"
            self.robot.drive_mode = "Emergency Stop"
            mode_dropdown.value = "Emergency Stop"
            speed_slider.disabled = True
            steer_slider.disabled = True

        @estop_button.on_click
        def _(_):
            execute_emergency_stop()

        # Background update loop for UI Markdown refresh
        def ui_update_loop():
            while self._running:
                try:
                    dashboard_md.content = self._format_dashboard_text()
                    robot_drive_status_md.content = self._format_robot_drive_status_text()
                    can_status_md.content = self._format_can_status_text()
                    time.sleep(0.1)
                except Exception:
                    break

        threading.Thread(target=ui_update_loop, daemon=True).start()

    def _format_robot_drive_status_text(self) -> str:
        status = self.robot.get_status()
        parsed_can = status.get("parsed_can_status", {})
        
        if not parsed_can:
            return "**Status:** ⚠️ Waiting for CAN 0 Rx Data..."

        lines = ["### 🚘 Live Status\n"]
        for key, val in parsed_can.items():
            lines.append(f"- **{key}**: `{val}`")

        return "\n".join(lines)

    def _format_mission_center_text(self) -> str:
        return """
<div style="background: rgba(0, 150, 255, 0.1); padding: 10px; border-radius: 6px; border-left: 4px solid #00B0FF; margin-bottom: 8px;">
    <h3 style="margin: 0 0 6px 0; color: #00E5FF; font-size: 1.1em;">🎯 Autonomous Mission Planner</h3>
    <p style="margin: 2px 0; font-size: 0.95em;"><b>Current Mission:</b> <span style="color: #00E676;">Patrol Route Alpha</span></p>
    <p style="margin: 2px 0; font-size: 0.95em;"><b>Status:</b> <span style="color: #FFD700;">STANDBY / READY</span></p>
    <p style="margin: 2px 0; font-size: 0.9em; color: #AAAAAA;">Waypoints Progress: <code>0 / 12</code> Completed</p>
</div>
        """

    def _format_can_status_text(self) -> str:
        status = self.robot.get_status()
        is_conn = status.get("connected", False)
        channel = status.get("channel", "can0")
        cmd_val = status.get("can_cmd_val", 0)
        
        status_color = "#00E676" if is_conn else "#FF5252"
        status_str = f"ONLINE ({channel})" if is_conn else f"OFFLINE ({channel})"

        return f"""
<div style="background: rgba(0,0,0,0.2); padding: 8px; border-radius: 6px; margin-bottom: 8px;">
    <b>CAN Bus Status:</b> <span style="color: {status_color}; font-weight: bold;">{status_str}</span><br>
    <small style="color: #BBBBBB;">Current Steering Cmd: <code>{cmd_val}</code> (-2000: L / +2000: R)</small>
</div>
        """

    def _format_dashboard_text(self) -> str:
        status = self.robot.get_status()
        speed = status["speed"]
        steer = status["steer_angle"]
        lat = status["latitude"]
        lon = status["longitude"]
        gear = status["gear"]
        mode = status["drive_mode"]

        w_mm = int(self.robot_width * 1000)
        l_mm = int(self.robot_length * 1000)
        h_mm = int(self.robot_height * 1000)

        return f"""
<div style="width: 100%; font-family: 'Inter', sans-serif; box-sizing: border-box; padding: 4px;">
    <h2 style="color: #00E676; margin-top: 0; margin-bottom: 8px; font-size: 1.4em;">🚀 APROS Patrol Robot Status</h2>
    <div style="background: rgba(255,255,255,0.05); padding: 12px; border-radius: 8px; border-left: 4px solid #308EFF;">
        <p style="margin: 4px 0; font-size: 1.1em;">⚡ <b>Speed (속도)</b>: <span style="color:#00E676; font-size: 1.4em; font-weight:bold;">{speed:.1f} km/h</span></p>
        <p style="margin: 4px 0; font-size: 1.05em;">🧭 <b>Steer Angle (조향각)</b>: <span style="color:#FFD700; font-size: 1.2em; font-weight:bold;">{steer:.1f}°</span></p>
        <p style="margin: 4px 0; font-size: 1.05em;">📍 <b>Latitude (위도)</b>: <code style="font-size: 1.1em; color: #E0E0E0;">N {lat:.6f}°</code></p>
        <p style="margin: 4px 0; font-size: 1.05em;">📍 <b>Longitude (경도)</b>: <code style="font-size: 1.1em; color: #E0E0E0;">E {lon:.6f}°</code></p>
        <p style="margin: 4px 0; font-size: 1.05em;">⚙️ <b>Gear</b>: <b style="color: #FFD700;">{gear}</b> | <b>Mode</b>: <b style="color: #00E676;">{mode}</b></p>
        <p style="margin: 4px 0; font-size: 0.95em; color: #AAAAAA;">📐 <b>Dimensions</b>: {w_mm} × {l_mm} × {h_mm} mm</p>
    </div>
</div>
        """

    def _simulation_loop(self):
        """Update robot kinematics for driving simulation and render real-time VLP-16 point cloud."""
        dt = 0.05
        t = 0.0
        while self._running:
            start_time = time.time()
            t += dt

            # Update simulated physics/kinematics in device controller
            self.robot.update_simulation_step(dt=dt)

            # Update Telescopic Mast 3D Model Height dynamically
            if hasattr(self.robot, "devices") and "telescopic_mast" in self.robot.devices:
                mast = self.robot.devices["telescopic_mast"]
                mast_height_m = mast.current_height_m
                self.mast_handle.height = mast_height_m
                self.mast_handle.position = (
                    getattr(self, 'mast_offset_x', 0.0),
                    getattr(self, 'mast_offset_y', 0.0),
                    getattr(self, 'mast_offset_z', self.robot_height) + mast_height_m / 2.0
                )

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
