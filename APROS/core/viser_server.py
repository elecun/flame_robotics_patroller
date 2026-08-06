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
        ouster_frame_path = "/robot/visual/base_link/mast_stage1_link/mast_stage2_link/mast_stage3_link/mast_stage4_link/mast_stage5_link/mast_stage6_link/mission_pan_link/mission_module_link/ouster_link/points"
        self.ouster_pc_handle = self.server.scene.add_point_cloud(
            name=ouster_frame_path,
            points=np.zeros((1, 3), dtype=np.float32),
            colors=np.array([[200, 80, 255]], dtype=np.uint8),
            point_size=0.015,
            point_shape="circle"
        )

        # 7. Route Visualization Scene Handles (Circles for waypoints, Spline for route path)
        self.route_pc_handle = self.server.scene.add_point_cloud(
            name="/world/route_waypoints",
            points=np.zeros((1, 3), dtype=np.float32),
            colors=np.array([[0, 230, 118]], dtype=np.uint8),
            point_size=0.1,  # Diameter 100mm = 0.1m
            point_shape="circle"
        )

        self.route_line_handle = self.server.scene.add_line_segments(
            name="/world/route_path",
            points=np.zeros((1, 2, 3), dtype=np.float32),
            colors=np.zeros((1, 2, 3), dtype=np.uint8),
            line_width=1.5,
            visible=True
        )

        # 8. POI Visualization Scene Handle (Sky Blue circles for POI points)
        self.poi_pc_handle = self.server.scene.add_point_cloud(
            name="/world/poi_points",
            points=np.zeros((1, 3), dtype=np.float32),
            colors=np.array([[0, 191, 255]], dtype=np.uint8),  # Sky Blue color
            point_size=0.15,  # Diameter 150mm = 0.15m
            point_shape="circle"
        )

        # 9. DWA Local Planner Path Handle (Vivid Neon Green/Cyan thick line on ground plane)
        self.local_path_handle = self.server.scene.add_line_segments(
            name="/world/local_planner_path",
            points=np.zeros((1, 2, 3), dtype=np.float32),
            colors=np.zeros((1, 2, 3), dtype=np.uint8),
            line_width=12.0,
            visible=True
        )

        # 10. Corridor Boundary Handle (Orange boundary lines on ground plane)
        self.corridor_boundary_handle = self.server.scene.add_line_segments(
            name="/world/corridor_boundary",
            points=np.zeros((1, 2, 3), dtype=np.float32),
            colors=np.zeros((1, 2, 3), dtype=np.uint8),
            line_width=2.0,
            visible=True
        )

    def _update_mission_preview(self, route_file_name: str, poi_file_name: str):
        """Load selected .route and .poi CSV files, convert lat/lon to relative meters from origin, and update scene."""
        import csv
        
        # 0. Clear previous scene handles to remove old route, POI & Corridor visualization
        self.route_pc_handle.points = np.zeros((0, 3), dtype=np.float32)
        self.route_pc_handle.colors = np.zeros((0, 3), dtype=np.uint8)
        self.route_line_handle.points = np.zeros((0, 2, 3), dtype=np.float32)
        self.route_line_handle.colors = np.zeros((0, 2, 3), dtype=np.uint8)
        self.route_line_handle.visible = False
        self.poi_pc_handle.points = np.zeros((0, 3), dtype=np.float32)
        self.poi_pc_handle.colors = np.zeros((0, 3), dtype=np.uint8)
        self.corridor_boundary_handle.points = np.zeros((0, 2, 3), dtype=np.float32)
        self.corridor_boundary_handle.colors = np.zeros((0, 2, 3), dtype=np.uint8)
        self.corridor_boundary_handle.visible = False

        route_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "route")
        
        # Read default corridor boundary from config/device if not specified per row
        default_cb = 2.5
        if self.robot:
            if hasattr(self.robot, "config") and self.robot.config and self.robot.config.has_section("mobile_drive_s1"):
                default_cb = float(self.robot.config.get("mobile_drive_s1", "corridor_boundary", fallback=2.5))
            elif hasattr(self.robot, "devices") and "mobile_drive_s1" in self.robot.devices:
                drive_dev = self.robot.devices["mobile_drive_s1"]
                if hasattr(drive_dev, "corridor_boundary"):
                    default_cb = float(drive_dev.corridor_boundary)

        origin_lat = None
        origin_lon = None

        # 1. Load Route Waypoints & Corridor Boundaries
        route_waypoints = []
        route_corridors = []
        if route_file_name and route_file_name != "None":
            route_path = os.path.join(route_dir, route_file_name)
            if os.path.exists(route_path):
                try:
                    with open(route_path, "r", encoding="utf-8") as f:
                        reader = csv.reader(f)
                        header = next(reader, None)
                        cb_idx = -1
                        if header:
                            for i, col in enumerate(header):
                                if col.strip().lower() == "corridor_boundary":
                                    cb_idx = i
                                    break
                        for row in reader:
                            if len(row) >= 3:
                                try:
                                    lat, lon = float(row[1]), float(row[2])
                                    cb_val = float(row[cb_idx]) if cb_idx != -1 and len(row) > cb_idx else default_cb
                                    route_waypoints.append((lat, lon))
                                    route_corridors.append(cb_val)
                                except ValueError:
                                    continue
                except Exception as e:
                    logger.error(f"[ViserServerManager] Error loading route '{route_file_name}': {e}")

        if route_waypoints:
            origin_lat, origin_lon = route_waypoints[0]

        # 2. Load POI Waypoints: (lat, lon, mast_height_mm)
        poi_waypoints = []
        if poi_file_name and poi_file_name != "None":
            poi_path = os.path.join(route_dir, poi_file_name)
            if os.path.exists(poi_path):
                try:
                    with open(poi_path, "r", encoding="utf-8") as f:
                        reader = csv.reader(f)
                        header = next(reader, None)
                        for row in reader:
                            if len(row) >= 3:
                                try:
                                    lat = float(row[1])
                                    lon = float(row[2])
                                    mast_height = float(row[3]) if len(row) >= 4 else 0.0
                                    poi_waypoints.append((lat, lon, mast_height))
                                except ValueError:
                                    continue
                except Exception as e:
                    logger.error(f"[ViserServerManager] Error loading POI '{poi_file_name}': {e}")

        if origin_lat is None and poi_waypoints:
            origin_lat, origin_lon = poi_waypoints[0][0], poi_waypoints[0][1]

        # Update Route & Corridor Boundary Visualization
        if route_waypoints and origin_lat is not None:
            points_3d = []
            for lat, lon in route_waypoints:
                dlat = lat - origin_lat
                dlon = lon - origin_lon
                dx = dlat * 111000.0
                dy = -dlon * 111000.0 * np.cos(np.radians(origin_lat))
                dz = 0.002
                points_3d.append([dx, dy, dz])

            pts_arr = np.array(points_3d, dtype=np.float32)
            self.route_pc_handle.points = pts_arr
            colors = np.zeros((len(pts_arr), 3), dtype=np.uint8)
            colors[:, 0] = 0     # Red
            colors[:, 1] = 230   # Green
            colors[:, 2] = 118   # Blue
            self.route_pc_handle.colors = colors

            if len(pts_arr) >= 2:
                segments = np.stack((pts_arr[:-1], pts_arr[1:]), axis=1)
                self.route_line_handle.points = segments
                seg_colors = np.zeros((len(segments), 2, 3), dtype=np.uint8)
                seg_colors[:, :, 0] = 0
                seg_colors[:, :, 1] = 230
                seg_colors[:, :, 2] = 118
                self.route_line_handle.colors = seg_colors
                self.route_line_handle.visible = True

                # Compute Left & Right Corridor Boundary Lines (perpendicular offset by half_width = corridor_boundary / 2)
                cb_segments = []
                for i in range(len(pts_arr) - 1):
                    p1 = pts_arr[i]
                    p2 = pts_arr[i + 1]
                    half_w = route_corridors[i] / 2.0 if i < len(route_corridors) else default_cb / 2.0

                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    length = np.hypot(dx, dy)
                    if length < 1e-6:
                        continue

                    # Normal vector (nx, ny) perpendicular to segment direction (dx, dy)
                    nx = -dy / length
                    ny = dx / length

                    # Left boundary segment
                    l_p1 = [p1[0] + nx * half_w, p1[1] + ny * half_w, 0.003]
                    l_p2 = [p2[0] + nx * half_w, p2[1] + ny * half_w, 0.003]
                    cb_segments.append([l_p1, l_p2])

                    # Right boundary segment
                    r_p1 = [p1[0] - nx * half_w, p1[1] - ny * half_w, 0.003]
                    r_p2 = [p2[0] - nx * half_w, p2[1] - ny * half_w, 0.003]
                    cb_segments.append([r_p1, r_p2])

                if cb_segments:
                    cb_arr = np.array(cb_segments, dtype=np.float32)
                    self.corridor_boundary_handle.points = cb_arr
                    cb_colors = np.zeros((len(cb_arr), 2, 3), dtype=np.uint8)
                    cb_colors[:, :, 0] = 255  # Red
                    cb_colors[:, :, 1] = 165  # Green (Orange: 255, 165, 0)
                    cb_colors[:, :, 2] = 0    # Blue
                    self.corridor_boundary_handle.colors = cb_colors
                    self.corridor_boundary_handle.visible = True
                else:
                    self.corridor_boundary_handle.visible = False
            else:
                self.route_line_handle.visible = False
                self.corridor_boundary_handle.visible = False
        else:
            self.route_pc_handle.points = np.zeros((0, 3), dtype=np.float32)
            self.route_line_handle.visible = False
            self.corridor_boundary_handle.visible = False

        # Update POI Visualization (Sky Blue Points in 3D Space, mast_height converted from mm to meters Z)
        if poi_waypoints and origin_lat is not None:
            poi_points_3d = []
            for lat, lon, mast_h_mm in poi_waypoints:
                dlat = lat - origin_lat
                dlon = lon - origin_lon
                dx = dlat * 111000.0
                dy = -dlon * 111000.0 * np.cos(np.radians(origin_lat))
                dz = mast_h_mm / 1000.0  # Convert mm to meters for Z-axis
                poi_points_3d.append([dx, dy, dz])

            poi_pts_arr = np.array(poi_points_3d, dtype=np.float32)
            self.poi_pc_handle.points = poi_pts_arr
            poi_colors = np.zeros((len(poi_pts_arr), 3), dtype=np.uint8)
            poi_colors[:, 0] = 0     # Red
            poi_colors[:, 1] = 191   # Green
            poi_colors[:, 2] = 255   # Blue (Sky Blue: 0, 191, 255)
            self.poi_pc_handle.colors = poi_colors
        else:
            self.poi_pc_handle.points = np.zeros((0, 3), dtype=np.float32)

        logger.info(f"[ViserServerManager] Preview updated for Route '{route_file_name}' ({len(route_waypoints)} pts) & POI '{poi_file_name}' ({len(poi_waypoints)} pts)")



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
                color="red",
                disabled=not init_is_auto
            )

            # estop_button disabled state updated dynamically in ui_update_loop

        # Helper function to list route files from APROS/route directory
        def get_route_files() -> list:
            route_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "route")
            if os.path.exists(route_dir) and os.path.isdir(route_dir):
                files = [f for f in os.listdir(route_dir) if f.endswith(".route")]
                if files:
                    return sorted(files)
            return ["None"]

        # Helper function to list POI files from APROS/route directory
        def get_poi_files() -> list:
            route_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "route")
            if os.path.exists(route_dir) and os.path.isdir(route_dir):
                files = [f for f in os.listdir(route_dir) if f.endswith(".poi")]
                if files:
                    return sorted(files)
            return ["None"]

        # Helper getter for mobile_drive_s1 device from current platform
        def get_mobile_drive_dev():
            if hasattr(self.robot, "drive_base") and self.robot.drive_base:
                return self.robot.drive_base
            if hasattr(self.robot, "devices") and "mobile_drive_s1" in self.robot.devices:
                return self.robot.devices["mobile_drive_s1"]
            return None

        # Tab 2: AD Control Tab (Autonomous Drive Control & Lighting GUI Window)
        with tabs.add_tab("AD Control", viser.Icon.ADJUSTMENTS):
            with client.gui.add_folder("⚡ Vehicle Control", expand_by_default=True):
                drive_dev = get_mobile_drive_dev()
                min_vel = getattr(drive_dev, "MIN_VELOCITY_KMH", -1.0) if drive_dev else -1.0
                max_vel = getattr(drive_dev, "MAX_VELOCITY_KMH", 3.0) if drive_dev else 3.0
                max_steer = getattr(drive_dev, "max_steering_angle", 30.0) if drive_dev else 30.0

                vel_slider = client.gui.add_slider(
                    label="Velocity (km/h)",
                    min=float(min_vel),
                    max=float(max_vel),
                    step=0.1,
                    initial_value=0.0
                )
                steer_slider = client.gui.add_slider(
                    label="Steer Angle (deg)",
                    min=-float(max_steer),
                    max=float(max_steer),
                    step=0.1,
                    initial_value=0.0
                )
                brake_slider = client.gui.add_slider(
                    label="Brake Pressure (%)",
                    min=0.0,
                    max=100.0,
                    step=1.0,
                    initial_value=0.0
                )

                @vel_slider.on_update
                def _(_):
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_speed"):
                        dev.set_speed(vel_slider.value)

                @steer_slider.on_update
                def _(_):
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_steering_angle"):
                        dev.set_steering_angle(steer_slider.value)

                @brake_slider.on_update
                def _(_):
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_brake"):
                        dev.set_brake(brake_slider.value)

                stop_button_group = client.gui.add_button_group(
                    label="Vehicle Stop Mode",
                    options=["🐌 Slow Stop", "🛑 Brake Stop"]
                )

                @stop_button_group.on_click
                def _(_):
                    if self._current_control_mode != "Auto":
                        logger.warning("[ViserUI] Vehicle Stop Mode clicked but vehicle is not in Auto mode.")
                        return
                    selected = stop_button_group.value
                    dev = get_mobile_drive_dev()
                    if dev:
                        if "Slow" in selected and hasattr(dev, "slow_stop"):
                            dev.slow_stop()
                            vel_slider.value = 0.0
                            brake_slider.value = 0.0
                        elif "Brake" in selected and hasattr(dev, "brake_stop"):
                            dev.brake_stop()
                            brake_slider.value = 10.0

            with client.gui.add_folder("💡 Vehicle Lighting & Signal Control", expand_by_default=True):
                left_turn_cb = client.gui.add_checkbox("Left Turn Light", initial_value=False)
                right_turn_cb = client.gui.add_checkbox("Right Turn Light", initial_value=False)
                head_light_cb = client.gui.add_checkbox("Head Light", initial_value=False)
                brake_light_cb = client.gui.add_checkbox("Brake Light", initial_value=False)

                @left_turn_cb.on_update
                def _(_):
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_lights"):
                        dev.set_lights(left_turn=left_turn_cb.value)

                @right_turn_cb.on_update
                def _(_):
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_lights"):
                        dev.set_lights(right_turn=right_turn_cb.value)

                @head_light_cb.on_update
                def _(_):
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_lights"):
                        dev.set_lights(head=head_light_cb.value)

                @brake_light_cb.on_update
                def _(_):
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_lights"):
                        dev.set_lights(brake=brake_light_cb.value)

            # AD Control UI enablement state is dynamically updated in background ui_update_loop
            ad_controls = [vel_slider, steer_slider, brake_slider, left_turn_cb, right_turn_cb, head_light_cb, brake_light_cb]

        # Tab 3: Mission Control Tab (Native Viser GUI Window)
        with tabs.add_tab("Mission", viser.Icon.TARGET):
            with client.gui.add_folder("📌 Patrol Route & Task Execution"):
                route_files = get_route_files()
                mission_route_dropdown = client.gui.add_dropdown(
                    label="Mission Route",
                    options=route_files,
                    initial_value=route_files[0]
                )
                poi_files = get_poi_files()
                mission_poi_dropdown = client.gui.add_dropdown(
                    label="Mission POI",
                    options=poi_files,
                    initial_value=poi_files[0]
                )
                @mission_route_dropdown.on_update
                def _(_):
                    val = mission_route_dropdown.value
                    if val not in route_files:
                        mission_route_dropdown.value = route_files[0]

                @mission_poi_dropdown.on_update
                def _(_):
                    val = mission_poi_dropdown.value
                    if val not in poi_files:
                        mission_poi_dropdown.value = poi_files[0]

                preview_btn = client.gui.add_button("👁️ Preview Mission", color="blue")
                refresh_missions_btn = client.gui.add_button("🔄 Refresh Missions", color="gray")
                start_mission_btn = client.gui.add_button("▶️ Start Mission", color="green")
                abort_mission_btn = client.gui.add_button("⏹️ Abort Mission", color="red")

                @refresh_missions_btn.on_click
                def _(_):
                    nonlocal route_files, poi_files
                    route_files = get_route_files()
                    poi_files = get_poi_files()
                    mission_route_dropdown.options = route_files
                    mission_route_dropdown.value = route_files[0]
                    mission_poi_dropdown.options = poi_files
                    mission_poi_dropdown.value = poi_files[0]
                    logger.info("[ViserUI] Refresh Missions clicked -> Reloaded route & POI lists.")

                @preview_btn.on_click
                def _(_):
                    route_file = mission_route_dropdown.value
                    poi_file = mission_poi_dropdown.value
                    self._update_mission_preview(route_file, poi_file)
                    logger.info(f"[ViserUI] Preview Mission clicked -> Visualized route '{route_file}' & POI '{poi_file}'")

                @start_mission_btn.on_click
                def _(_):
                    route_file = mission_route_dropdown.value
                    poi_file = mission_poi_dropdown.value
                    if hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                        self.robot.drive_executor.start_mission(route_file, poi_file_name=poi_file)
                        logger.info(f"[ViserUI] Start Mission clicked -> Started DriveExecutor with route '{route_file}' & POI '{poi_file}'")

                @abort_mission_btn.on_click
                def _(_):
                    if hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                        self.robot.drive_executor.abort_mission()
                        logger.info("[ViserUI] Abort Mission clicked -> Aborted DriveExecutor")

            # Data Logger Folder (bottom of Mission Control tab)
            with client.gui.add_folder("💾 Data Logger", expand_by_default=True):
                datalog_status_md = client.gui.add_markdown(
                    "**Status**: ⏹️ Stopped"
                )

                datalog_mode_group = client.gui.add_button_group(
                    "Action",
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

            # Route Builder (RTK) Folder
            with client.gui.add_folder("🗺️ Route Builder (RTK)", expand_by_default=True):
                rtk_builder_status_md = client.gui.add_markdown("**Status**: ⏹️ Stopped")
                rtk_builder_points_md = client.gui.add_markdown("**Points**: 0")
                rtk_builder_group = client.gui.add_button_group(
                    "Action",
                    options=["⏺️ Record", "⏹️ Stop"]
                )

                # State variables for RTK route builder
                rtk_recording_state = {"is_recording": False, "file_path": None, "file_handle": None, "point_count": 0, "last_lat": None, "last_lon": None}

                @rtk_builder_group.on_click
                def _(event: viser.GuiEvent) -> None:
                    selected = rtk_builder_group.value
                    if selected == "⏺️ Record":
                        if not rtk_recording_state["is_recording"]:
                            apros_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                            route_dir = os.path.join(apros_root, "route")
                            os.makedirs(route_dir, exist_ok=True)
                            filename = f"record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.route"
                            file_path = os.path.join(route_dir, filename)
                            try:
                                f = open(file_path, "w", encoding="utf-8")
                                f.write("index,latitude,longitude,corridor_boundary\n")
                                f.flush()
                                rtk_recording_state["is_recording"] = True
                                rtk_recording_state["file_path"] = file_path
                                rtk_recording_state["file_handle"] = f
                                rtk_recording_state["point_count"] = 0
                                rtk_recording_state["last_lat"] = None
                                rtk_recording_state["last_lon"] = None
                                rtk_builder_status_md.content = f"**Status**: 🔴 Recording (`{filename}`)"
                                rtk_builder_points_md.content = "**Points**: 0"
                                logger.info(f"[ViserUI] RTK Route Builder recording started: {file_path}")
                            except Exception as err:
                                logger.error(f"[ViserUI] Failed to start RTK Route recording: {err}")
                    elif selected == "⏹️ Stop":
                        if rtk_recording_state["is_recording"]:
                            rtk_recording_state["is_recording"] = False
                            if rtk_recording_state["file_handle"]:
                                try:
                                    rtk_recording_state["file_handle"].close()
                                except Exception:
                                    pass
                            rtk_recording_state["file_handle"] = None
                            rtk_builder_status_md.content = "**Status**: ⏹️ Stopped"
                            logger.info(f"[ViserUI] RTK Route Builder recording stopped. Total points: {rtk_recording_state['point_count']}")

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





        def execute_emergency_stop():
            self.robot.speed = 1.0
            # self.robot.set_steering_angle(0.0)
            self.robot.gear = "D"
            # self.robot.drive_mode = "Emergency Stop"

        @estop_button.on_click
        def _(_):
            execute_emergency_stop()

        # Background update loop for UI Markdown refresh and Camera Stream
        def ui_update_loop():
            last_img_ts = 0.0
            while self._running:
                try:
                    robot_drive_status_md.content = self._format_robot_drive_status_text()

                    # Dynamically enable/disable AD Control, Mission Start/Abort & ESTOP UI based on system Drive Mode State (Auto/AD)
                    try:
                        drive_dev = get_mobile_drive_dev()
                        drive_status = drive_dev.get_status() if drive_dev and hasattr(drive_dev, "get_status") else {}
                        parsed_can = drive_status.get("parsed_can_status", {}) if isinstance(drive_status.get("parsed_can_status"), dict) else {}
                        drive_mode_state = str(parsed_can.get("drive_state_mode", drive_status.get("drive_mode", getattr(self.robot, "drive_mode", "Remote")))).strip()
                        is_ad_active = drive_mode_state.startswith("Auto") or drive_mode_state.startswith("AD") or drive_mode_state == "1"

                        estop_button.disabled = not is_ad_active
                        start_mission_btn.disabled = not is_ad_active
                        abort_mission_btn.disabled = not is_ad_active
                        for ctrl in ad_controls:
                            ctrl.disabled = not is_ad_active
                    except Exception as gui_err:
                        pass

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

                    # Update RTK Route Builder recording logic
                    try:
                        if rtk_recording_state["is_recording"] and rtk_recording_state["file_handle"]:
                            rtk_dev = self.robot.devices.get("synerex_rtk") if hasattr(self.robot, "devices") and self.robot.devices else None
                            if rtk_dev is not None:
                                cur_lat = getattr(rtk_dev, "latitude", None)
                                cur_lon = getattr(rtk_dev, "longitude", None)
                                if cur_lat is not None and cur_lon is not None:
                                    if cur_lat != rtk_recording_state["last_lat"] or cur_lon != rtk_recording_state["last_lon"]:
                                        cb = float(self.robot.config.get("mobile_drive_s1", "corridor_boundary", fallback=2.5)) if hasattr(self.robot, "config") and self.robot.config else 2.5
                                        idx = rtk_recording_state["point_count"]
                                        rtk_recording_state["file_handle"].write(f"{idx},{cur_lat:.8f},{cur_lon:.8f},{cb}\n")
                                        rtk_recording_state["file_handle"].flush()
                                        rtk_recording_state["point_count"] += 1
                                        rtk_recording_state["last_lat"] = cur_lat
                                        rtk_recording_state["last_lon"] = cur_lon
                                        rtk_builder_points_md.content = f"**Points**: {rtk_recording_state['point_count']}"
                    except Exception as rtk_err:
                        logger.error(f"[ViserUI] RTK Route Builder update loop error: {rtk_err}")

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
        mast_dev = self.robot.devices.get("telescopic_mast") if hasattr(self.robot, "devices") and self.robot.devices else None

        drive_status = drive_dev.get_status() if drive_dev and hasattr(drive_dev, "get_status") else {}
        parsed_can = drive_status.get("parsed_can_status", {}) if isinstance(drive_status.get("parsed_can_status"), dict) else {}
        incline_status = incline_dev.get_status() if incline_dev and hasattr(incline_dev, "get_status") else {}

        # 1. Msg ID 0x303 & 0x0A0
        drive_mode_state = parsed_can.get("drive_state_mode", drive_status.get("drive_mode", "N/A"))
        lines.append(f"- **Drive Mode State**: `{drive_mode_state}`")

        vehicle_gear = parsed_can.get("vehicle_gear", drive_status.get("gear", "P"))
        lines.append(f"- **Vehicle Gear**: `{vehicle_gear}`")

        clamping_brake = parsed_can.get("clamping_brake_status", "Released")
        lines.append(f"- **Clamping Brake**: `{clamping_brake}`")

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

        # 5. Telescopic Mast Height
        mast_height_val = None
        if mast_dev:
            if hasattr(mast_dev, "get_status"):
                m_status = mast_dev.get_status()
                if isinstance(m_status, dict):
                    mast_height_val = m_status.get("current_height_m")
                    if mast_height_val is None and "current_height_mm" in m_status:
                        mast_height_val = m_status["current_height_mm"] / 1000.0
            if mast_height_val is None and hasattr(mast_dev, "current_height_m"):
                mast_height_val = mast_dev.current_height_m

        if mast_height_val is None and hasattr(self.robot, "telescopic_mast_connector") and self.robot.telescopic_mast_connector:
            last_mast = getattr(self.robot.telescopic_mast_connector, "last_mast_data", None)
            if isinstance(last_mast, dict):
                mast_height_val = last_mast.get("current_height_m")
                if mast_height_val is None and "current_height_mm" in last_mast:
                    mast_height_val = last_mast["current_height_mm"] / 1000.0

        if mast_height_val is not None:
            mast_height_str = f"{mast_height_val:.2f} m"
            if not hasattr(self, "_last_logged_dash_mast_height") or self._last_logged_dash_mast_height != mast_height_str:
                self._last_logged_dash_mast_height = mast_height_str
                logger.info(f"[Viser UI Dashboard] Robot Drive Status -> Mast Height displayed: {mast_height_str} (raw: {mast_height_val:.3f} m)")
        else:
            mast_height_str = "N/A"

        lines.append(f"- **Mast Height**: `{mast_height_str}`")

        # 5. Synerex RTK GNSS
        rtk_dev = self.robot.devices.get("synerex_rtk") if hasattr(self.robot, "devices") and self.robot.devices else None
        rtk_data = rtk_dev.get_status() if rtk_dev and hasattr(rtk_dev, "get_status") else (getattr(self.robot, "last_rtk_data", {}) or {})
        
        is_rtk_conn = rtk_data.get("connected", False) if rtk_data else False
        lat_val = rtk_data.get("latitude") if rtk_data else getattr(rtk_dev, "latitude", None)
        lon_val = rtk_data.get("longitude") if rtk_data else getattr(rtk_dev, "longitude", None)
        heading_val = rtk_data.get("heading") if rtk_data else getattr(rtk_dev, "heading", None)
        fq = rtk_data.get("fix_quality") if rtk_data else getattr(rtk_dev, "fix_quality", 0)

        # Fallback / Disconnected / Invalid status formatting
        if not is_rtk_conn or lat_val is None or lon_val is None or fq == 0:
            lat_str = "-"
            lon_str = "-"
            heading_str = "-"
            quality_str = "-"
        else:
            lat_str = f"{lat_val} deg"
            lon_str = f"{lon_val} deg"
            heading_str = f"{heading_val:.1f}°" if heading_val is not None else "-"
            from core.device.synerex_rtk import SynerexRTK
            quality_str = SynerexRTK.quality2str(fq)

        lines.append(f"- **GPS(Lat)**: `{lat_str}`")
        lines.append(f"- **GPS(Lon)**: `{lon_str}`")
        lines.append(f"- **GPS(Heading)**: `{heading_str}`")
        lines.append(f"- **GPS(Quality)**: `{quality_str}`")

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

            # Real-time Local Planner Planned Trajectory visualization on ground plane
            local_planner = getattr(self.robot, "local_planner", None)
            if local_planner is None and hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                local_planner = getattr(self.robot.drive_executor, "local_planner", None)
            if local_planner and hasattr(local_planner, "best_local_path") and local_planner.best_local_path:
                lpath = local_planner.best_local_path
                if len(lpath) >= 2:
                    lpts = np.array([[pt["x"], pt["y"], 0.03] for pt in lpath], dtype=np.float32)
                    lsegments = np.stack((lpts[:-1], lpts[1:]), axis=1)  # (N, 2, 3)
                    lcolors = np.zeros((len(lsegments), 2, 3), dtype=np.uint8)
                    lcolors[:, :, 0] = 0     # Red
                    lcolors[:, :, 1] = 255   # Green (High-contrast Vivid Lime Green)
                    lcolors[:, :, 2] = 128   # Blue
                    self.local_path_handle.points = lsegments
                    self.local_path_handle.colors = lcolors
                    self.local_path_handle.visible = True
                else:
                    self.local_path_handle.visible = False
            else:
                self.local_path_handle.visible = False

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
