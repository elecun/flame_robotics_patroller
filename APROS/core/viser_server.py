"""
APROS Viser Visualization Server & UI Module.
Handles 3D visualization, robot box model (W:1000, L:2055, H:640 mm), CAN connection status display, and MobileDriveS1 control integration.
"""
import math
import os
import importlib
import time
from datetime import datetime
import threading
import cv2
import numpy as np
import viser
import viser.transforms as tf
from typing import Any, Optional
from pyproj import Transformer
from core.data_logger import DataLogger
from core.gnss_rtk_recorder import GNSSRTKRecorder
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

        # Fixed WGS84 scene origin used to anchor the ground tile map.
        self.default_lat = 34.824628
        self.default_lon = 127.661044
        self.default_heading = 0.0

        # Pyproj Transformer: WGS84 (EPSG:4326) -> Korean Central Belt TM (EPSG:5186)
        self.tm_transformer = Transformer.from_crs("EPSG:4326", "EPSG:5186", always_xy=True)

        self.robot_frame_handle: Optional[Any] = None

        # Data Logger
        self._data_logger = DataLogger(robot=self.robot)
        self._gnss_rtk_recorder = GNSSRTKRecorder(robot=self.robot)

        # Title configuration & Theme setup (control_layout="fixed" docks panel to the right side)
        self.server.gui.configure_theme(
            titlebar_content=None,
            control_layout="fixed",
            control_width="large",
            dark_mode=True,
            show_logo=False,
            brand_color=(30, 144, 255)
        )
        self.server.gui.set_panel_label("Control Center")

        # Read the configured fixed scene origin from apros.cfg [PLATFORM] section.
        if hasattr(self.robot, 'config') and self.robot.config and self.robot.config.has_section("PLATFORM"):
            plat_sec = self.robot.config["PLATFORM"]
            if "default_lat" in plat_sec:
                try:
                    self.default_lat = float(plat_sec["default_lat"])
                except ValueError:
                    pass
            if "default_lon" in plat_sec:
                try:
                    self.default_lon = float(plat_sec["default_lon"])
                except ValueError:
                    pass
            if "default_alt" in plat_sec:
                try:
                    self.default_alt = float(plat_sec["default_alt"])
                except ValueError:
                    pass

        # Compute origin TM coordinates for relative Viser 3D local scene (centered at site origin)
        self.origin_easting, self.origin_northing = self.tm_transformer.transform(self.default_lon, self.default_lat)
        self.ground_tile_map_handle = None
        self.ground_tile_map_tile_key = None
        self.ground_tile_map_retry_after = 0.0
        self.ground_grid_handle = None

        # Custom APROS titlebar. The former floating map panel has been removed.
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
            justify-content: flex-start;
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
        </div>

        <script>
        (function() {{
            function attachTitlebar() {{
                var titlebar = document.getElementById('apros-top-titlebar');
                if (titlebar && titlebar.parentElement !== document.body) document.body.appendChild(titlebar);
            }}
            attachTitlebar();
            setTimeout(attachTitlebar, 50);
            setTimeout(attachTitlebar, 300);
        }})();
        </script>

        <style>
            /* Reserve the fixed titlebar height once; padding nested roots stacks. */
            body {{
                padding-top: 48px !important;
            }}
            /* Hide default empty header if any */
            header, [class*="mantine-Header-root"] {{
                display: none !important;
            }}
        </style>
        """
        self.server.gui.add_html(titlebar_html)

        # Robot dimensions in meters; V2 includes its upper chassis in overall height.
        model_name = "iae_patrol_v1"
        if self.robot and getattr(self.robot, "config", None) and self.robot.config.has_section("PLATFORM"):
            model_name = self.robot.config.get("PLATFORM", "robot_model", fallback=model_name).strip().lower()
        model_dimensions = {
            "iae_patrol_v1": (1.000, 2.055, 0.640),
            "iae_patrol_v2": (0.970, 2.030, 0.837),
        }
        self.robot_width, self.robot_length, self.robot_height = model_dimensions.get(model_name, model_dimensions["iae_patrol_v1"])
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

    def _wgs84_to_tm_viser(self, lat: float, lon: float) -> tuple[float, float]:
        """
        Convert WGS84 (Lat, Lon) to Korean Central Belt TM (EPSG:5186) relative Viser coordinates.
        Viser Mapping (relative to origin default_lat, default_lon):
          +X axis = True North (Northing offset N)
          -Y axis = True East (Easting offset E) -> Y_viser = -Easting offset
        """
        easting, northing = self.tm_transformer.transform(lon, lat)
        rel_n = northing - self.origin_northing
        rel_e = easting - self.origin_easting
        return float(rel_n), float(-rel_e)

    def _heading_to_wxyz(self, heading_deg: float) -> tuple[float, float, float, float]:
        """
        Convert Heading (degrees, 0=North, 90=East, 180=South, 270=West) to Viser WXYZ Quaternion.
        Yaw = -radians(heading_deg) around Z-axis.
        """
        rad = -math.radians(heading_deg)
        w = math.cos(rad / 2.0)
        z = math.sin(rad / 2.0)
        return (w, 0.0, 0.0, z)

    def _add_ground_tile_map(self, center_lat: float, center_lon: float) -> bool:
        """Keep a north-up tile mosaic centered under the current geodetic position."""
        zoom = 18
        tile_px = 256
        tile_count = 2 ** zoom
        center_tile_x = (center_lon + 180.0) / 360.0 * tile_count
        lat_rad = math.radians(center_lat)
        center_tile_y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * tile_count
        meters_per_pixel = (math.cos(lat_rad) * 2.0 * math.pi * 6378137.0) / (tile_count * tile_px)
        tile_x = math.floor(center_tile_x)
        tile_y = math.floor(center_tile_y)
        tile_key = (tile_x, tile_y)
        min_x, min_y = tile_x - 1, tile_y - 1
        mosaic_width = tile_px * 3

        if tile_key != self.ground_tile_map_tile_key:
            if time.monotonic() < self.ground_tile_map_retry_after:
                if self.ground_tile_map_handle is not None:
                    self.ground_tile_map_handle.visible = False
                return False

            mosaic = np.full((mosaic_width, mosaic_width, 3), (42, 48, 58), dtype=np.uint8)
            tile_root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "maptile", str(zoom))
            try:
                for tile_x in range(min_x, min_x + 3):
                    for tile_y in range(min_y, min_y + 3):
                        tile_path = os.path.join(tile_root, str(tile_x), f"{tile_y}.png")
                        tile_bgr = cv2.imread(tile_path, cv2.IMREAD_COLOR)
                        if tile_bgr is None:
                            raise OSError(f"Local map tile is missing or invalid: {tile_path}")
                        if tile_bgr.shape[:2] != (tile_px, tile_px):
                            tile_bgr = cv2.resize(tile_bgr, (tile_px, tile_px), interpolation=cv2.INTER_AREA)
                        tile_rgb = cv2.cvtColor(tile_bgr, cv2.COLOR_BGR2RGB)
                        x0 = (tile_x - min_x) * tile_px
                        y0 = (tile_y - min_y) * tile_px
                        mosaic[y0:y0 + tile_px, x0:x0 + tile_px] = tile_rgb
            except OSError as exc:
                self.ground_tile_map_retry_after = time.monotonic() + 10.0
                if self.ground_tile_map_handle is not None:
                    self.ground_tile_map_handle.visible = False
                logger.warning(f"[ViserServerManager] Ground tile map unavailable: {exc}")
                return False

            # Viser's image plane applies an internal 180-degree X rotation, which
            # reverses its vertical image axis. Flip the raster to keep north at +X.
            image = cv2.flip(mosaic, 0)
            if self.ground_tile_map_handle is None:
                self.ground_tile_map_handle = self.server.scene.add_image(
                    name="/world/ground_tile_map",
                    image=image,
                    render_width=mosaic_width * meters_per_pixel,
                    render_height=mosaic_width * meters_per_pixel,
                    format="jpeg",
                    jpeg_quality=90,
                    wxyz=(math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5)),
                    cast_shadow=False,
                    receive_shadow=False,
                )
            else:
                self.ground_tile_map_handle.image = image

            self.ground_tile_map_tile_key = tile_key
            self.ground_tile_map_retry_after = 0.0
            logger.info(f"[ViserServerManager] Ground tiles refreshed at {center_lat:.6f}, {center_lon:.6f} (z{zoom})")

        # Update on every RTK pose while keeping expensive tile fetches limited to
        # crossing a Web Mercator tile boundary. Scene axes are +X north, +Y west.
        pixel_x = (center_tile_x - min_x) * tile_px
        pixel_y = (center_tile_y - min_y) * tile_px
        image_center_x = (pixel_y - mosaic_width / 2.0) * meters_per_pixel
        image_center_y = (pixel_x - mosaic_width / 2.0) * meters_per_pixel
        robot_x, robot_y = self._wgs84_to_tm_viser(center_lat, center_lon)
        image_position = (robot_x + image_center_x, robot_y + image_center_y, 0.001)
        image_width = mosaic_width * meters_per_pixel

        self.ground_tile_map_handle.position = image_position
        self.ground_tile_map_handle.render_width = image_width
        self.ground_tile_map_handle.render_height = image_width
        self.ground_tile_map_handle.visible = True
        return True

    def _setup_scene(self):
        """Build environment scene in Korean Central Belt TM Coordinate System (EPSG:5186)."""
        init_x, init_y = self._wgs84_to_tm_viser(self.default_lat, self.default_lon)
        init_wxyz = self._heading_to_wxyz(self.default_heading)

        # 1. Ground tile image centered at the fixed scene origin; keep the grid as fallback.
        tile_map_loaded = self._add_ground_tile_map(self.default_lat, self.default_lon)
        self.ground_grid_handle = self.server.scene.add_grid(
            name="/ground_grid",
            width=200.0,
            height=200.0,
            position=(init_x, init_y, 0.0),
            plane="xy",
            cell_color=(80, 90, 100),
            section_color=(140, 150, 160),
            visible=not tile_map_loaded,
        )

        # 2. World Axes Frame in TM Space (+X=North, -Y=East, Z=Up)
        self.server.scene.add_frame(
            name="/world_axes",
            position=(init_x, init_y, 0.0),
            show_axes=True,
            axes_length=1.5,
            axes_radius=0.03,
            origin_radius=0.04
        )
        

        # 3. Add /robot Root Frame Node in TM Space
        self.robot_frame_handle = self.server.scene.add_frame(
            name="/robot",
            position=(init_x, init_y, 0.0),
            wxyz=init_wxyz
        )

        # 3. Load URDF Robot Model into Viser Scene
        platform_cfg = self.robot.config["PLATFORM"] if self.robot and getattr(self.robot, "config", None) and self.robot.config.has_section("PLATFORM") else {}
        robot_model = str(platform_cfg.get("robot_model", "iae_patrol_v1")).strip().lower()
        urdf_file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "urdf", f"{robot_model}.urdf")
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
                import traceback
                logger.error(f"[ViserServerManager] Failed to load URDF model: {e}\n{traceback.format_exc()}")

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
        ouster_base_path = "/robot/visual/base_link"
        if robot_model == "iae_patrol_v2":
            ouster_base_path += "/upper_chassis_link"
        ouster_frame_path = ouster_base_path + "/mast_stage1_link/mast_stage2_link/mast_stage3_link/mast_stage4_link/mast_stage5_link/mast_stage6_link/mission_pan_link/mission_module_link/ouster_link/points"
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

        # 8. POI Visualization Scene Handles & Labels
        self.poi_label_handles = []
        self.poi_pc_handle = self.server.scene.add_point_cloud(
            name="/world/poi_points",
            points=np.zeros((1, 3), dtype=np.float32),
            colors=np.array([[0, 191, 255]], dtype=np.uint8),  # Sky Blue color
            point_size=0.2,  # Diameter 200mm = 0.2m
            point_shape="circle"
        )
        self.poi_mast_line_handle = self.server.scene.add_line_segments(
            name="/world/poi_mast_lines",
            points=np.zeros((1, 2, 3), dtype=np.float32),
            colors=np.array([[[0, 191, 255], [0, 191, 255]]], dtype=np.uint8),
            line_width=3.0,
            visible=True
        )

        # Local planner centerline, drawn as a visible ground-level strip.
        self.local_path_handle = self.server.scene.add_mesh_simple(
            name="/world/local_planner_path",
            vertices=np.array([[0, 0, 0.04], [0, 0, 0.04], [0.01, 0, 0.04], [0.01, 0, 0.04]], dtype=np.float32),
            faces=np.array([[0, 1, 2], [1, 3, 2]], dtype=np.uint32),
            color=(255, 200, 0), opacity=1.0, side="double", visible=False
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
        if hasattr(self, "poi_mast_line_handle"):
            self.poi_mast_line_handle.points = np.zeros((0, 2, 3), dtype=np.float32)
            self.poi_mast_line_handle.visible = False

        for lbl in self.poi_label_handles:
            try:
                lbl.remove()
            except Exception:
                pass
        self.poi_label_handles.clear()

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

        # Sample route waypoints by 1m distance intervals for visualization & preview
        if route_waypoints:
            try:
                mod_ru = importlib.import_module("APROS.util.route_utils" if __name__.startswith("APROS") else "util.route_utils")
                sample_route_by_distance = getattr(mod_ru, "sample_route_by_distance")
                route_waypoints, route_corridors = sample_route_by_distance(route_waypoints, route_corridors, sample_step_m=1.0)
            except Exception as e:
                logger.error(f"[ViserServerManager] Error sampling route by 1m: {e}")

        if route_waypoints:
            origin_lat, origin_lon = route_waypoints[0]

        # 2. Load POI Waypoints: (lat, lon, mast_height_mm [int])
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
                                    mast_height = int(round(float(row[3]))) if len(row) >= 4 else 0
                                    poi_waypoints.append((lat, lon, mast_height))
                                except ValueError:
                                    continue
                except Exception as e:
                    logger.error(f"[ViserServerManager] Error loading POI '{poi_file_name}': {e}")

        if origin_lat is None and poi_waypoints:
            origin_lat, origin_lon = poi_waypoints[0][0], poi_waypoints[0][1]

        # Update Route & Corridor Boundary Visualization in TM Coordinates
        if route_waypoints:
            points_3d = []
            for lat, lon in route_waypoints:
                rx, ry = self._wgs84_to_tm_viser(lat, lon)
                points_3d.append([rx, ry, 0.002])

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

        # Update POI Visualization in TM Coordinates & Add Labels for mast_height (int)
        if poi_waypoints:
            poi_points_3d = []
            mast_lines = []
            for idx, (lat, lon, mast_h_mm) in enumerate(poi_waypoints):
                rx, ry = self._wgs84_to_tm_viser(lat, lon)
                dz = mast_h_mm / 1000.0  # Convert mm to meters for Z-axis
                poi_points_3d.append([rx, ry, dz])
                mast_lines.append([[rx, ry, 0.0], [rx, ry, dz]])

                # Add 3D text label for each POI showing integer mast_height
                lbl_text = f"📍 POI #{idx}: {mast_h_mm} mm"
                lbl_handle = self.server.scene.add_label(
                    name=f"/world/poi_labels/label_{idx}",
                    text=lbl_text,
                    position=(rx, ry, dz + 0.3)
                )
                self.poi_label_handles.append(lbl_handle)

            poi_pts_arr = np.array(poi_points_3d, dtype=np.float32)
            self.poi_pc_handle.points = poi_pts_arr
            poi_colors = np.zeros((len(poi_pts_arr), 3), dtype=np.uint8)
            poi_colors[:, 0] = 0     # Red
            poi_colors[:, 1] = 191   # Green
            poi_colors[:, 2] = 255   # Blue (Sky Blue: 0, 191, 255)
            self.poi_pc_handle.colors = poi_colors

            if mast_lines and hasattr(self, "poi_mast_line_handle"):
                mast_lines_arr = np.array(mast_lines, dtype=np.float32)
                self.poi_mast_line_handle.points = mast_lines_arr
                line_colors = np.zeros((len(mast_lines_arr), 2, 3), dtype=np.uint8)
                line_colors[:, :, 0] = 0
                line_colors[:, :, 1] = 191
                line_colors[:, :, 2] = 255
                self.poi_mast_line_handle.colors = line_colors
                self.poi_mast_line_handle.visible = True
        else:
            self.poi_pc_handle.points = np.zeros((0, 3), dtype=np.float32)
            if hasattr(self, "poi_mast_line_handle"):
                self.poi_mast_line_handle.points = np.zeros((0, 2, 3), dtype=np.float32)
                self.poi_mast_line_handle.visible = False

        logger.info(f"[ViserServerManager] Preview updated for Route '{route_file_name}' ({len(route_waypoints)} pts) & POI '{poi_file_name}' ({len(poi_waypoints)} pts)")



    def _setup_client_ui(self, client: viser.ClientHandle):
        """Setup UI components for newly connected client."""
        # TM Coordinate Frame Camera Orientation
        init_x, init_y = self._wgs84_to_tm_viser(self.default_lat, self.default_lon)
        client.camera.up_direction = (0.0, 0.0, 1.0)
        client.camera.position = (init_x - 6.0, init_y - 3.5, 3.5)
        client.camera.look_at = (init_x, init_y, 0.5)

        # Left dock: live device status and camera stream.
        status_panel = client.gui.add_panel()
        with status_panel.add_tab("Status Monitor", viser.Icon.CHART_BAR):
            with client.gui.add_folder("Robot Drive Status", expand_by_default=True):
                robot_drive_status_md = client.gui.add_markdown(self._format_status_text(sensor_only=False))

            with client.gui.add_folder("Camera Stream", expand_by_default=True):
                init_cam_img = np.zeros((240, 320, 3), dtype=np.uint8)
                init_cam_img[::20, :] = [30, 30, 45]
                init_cam_img[:, ::20] = [30, 30, 45]
                gui_cam_image = client.gui.add_image(image=init_cam_img, format="jpeg")
                auto_exposure_checkbox = client.gui.add_checkbox(
                    label="Auto Exposure",
                    initial_value=True,
                )
                checkbox_id = auto_exposure_checkbox._impl.uuid
                client.gui.add_html(f"""
                    <style>
                    .mantine-Flex-root:has(input[id="{checkbox_id}"]) > .mantine-Box-root:first-child {{
                        order: 2;
                        width: auto !important;
                        padding-left: 8px;
                        padding-right: 0;
                    }}
                    .mantine-Flex-root:has(input[id="{checkbox_id}"]) > .mantine-Box-root:last-child {{
                        order: 1;
                        flex-grow: 0 !important;
                    }}
                    </style>
                """)

                @auto_exposure_checkbox.on_update
                def _(event):
                    cam_dev = self.robot.devices.get("basler_gige_camera") if hasattr(self.robot, "devices") and self.robot.devices else None
                    if cam_dev and hasattr(cam_dev, "set_auto_exposure"):
                        cam_dev.set_auto_exposure(bool(event.target.value))

            with client.gui.add_folder("AIC Stream", expand_by_default=True):
                init_aic_img = np.zeros((240, 320, 3), dtype=np.uint8)
                gui_aic_image = client.gui.add_image(image=init_aic_img, format="jpeg")
        with status_panel.add_tab("Sensor", viser.Icon.RADAR):
            with client.gui.add_folder("Sensor Status", expand_by_default=True):
                sensor_status_md = client.gui.add_markdown(self._format_status_text(sensor_only=True))
        status_panel.dock_left()
        status_panel.set_width(360)

        # Tab group for the APROS Control Center panels.
        tabs = client.gui.add_tab_group()

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

        # Control Tab (Vehicle, lighting, and mast controls)
        with tabs.add_tab("Control", viser.Icon.ADJUSTMENTS):
            # The UI starts in Manual (Remote Control) until a mode change is requested.
            self._current_control_mode = "Remote"
            _is_processing_mode_change = False
            reset_motion_controls_on_auto = False
            current_mode_md = client.gui.add_markdown("**Current Mode:** 🟢 Manual")
            mode_button_group = client.gui.add_button_group(
                label="Mode",
                options=("Auto", "Manual"),
            )
            mode_button_group.value = "Manual"

            def update_mode_display(selected_mode: str):
                label = "Auto" if selected_mode == "Auto" else "Manual"
                mode_button_group.value = label
                current_mode_md.content = f"**Current Mode:** 🟢 {label}"

            def apply_control_mode(selected_mode: str):
                nonlocal reset_motion_controls_on_auto
                self._current_control_mode = selected_mode
                mobile_drive_dev = get_mobile_drive_dev()
                if mobile_drive_dev and hasattr(mobile_drive_dev, "change_drive_mode"):
                    mobile_drive_dev.change_drive_mode(selected_mode)
                elif hasattr(self.robot, "set_mode_remote") and selected_mode == "Remote":
                    self.robot.set_mode_remote()
                elif hasattr(self.robot, "set_mode_auto") and selected_mode == "Auto":
                    self.robot.set_mode_auto()
                reset_motion_controls_on_auto = selected_mode == "Auto"
                update_mode_display(selected_mode)
                logger.info(f"[ViserUI] Control mode changed to {selected_mode}.")

            def request_control_mode(selected_mode: str):
                nonlocal _is_processing_mode_change
                if _is_processing_mode_change or selected_mode == self._current_control_mode:
                    return
                if selected_mode == "Auto" and self._current_control_mode == "Remote":
                    _is_processing_mode_change = True
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
                            update_mode_display(self._current_control_mode)
                            _is_processing_mode_change = False
                else:
                    apply_control_mode(selected_mode)

            @mode_button_group.on_click
            def _(event):
                request_control_mode("Auto" if event.target.value == "Auto" else "Remote")

            vehicle_controls_enabled = False
            syncing_vehicle_controls = False
            with client.gui.add_folder("Vehicle Control", expand_by_default=True):
                drive_dev = get_mobile_drive_dev()
                min_vel = getattr(drive_dev, "MIN_VELOCITY_KMH", -1.0) if drive_dev else -1.0
                max_vel = getattr(drive_dev, "MAX_VELOCITY_KMH", 3.0) if drive_dev else 3.0
                max_steer = getattr(drive_dev, "max_steering_angle", 30.0) if drive_dev else 30.0

                slider_min_vel = min(0.0, float(min_vel))
                slider_max_vel = max(float(max_vel), slider_min_vel + 0.1)
                init_vel = max(slider_min_vel, min(slider_max_vel, 0.0))

                vel_slider = client.gui.add_slider(
                    label="Velocity (km/h)",
                    min=slider_min_vel,
                    max=slider_max_vel,
                    step=0.1,
                    initial_value=init_vel,
                    disabled=True,
                )
                slider_max_steer = max(0.1, abs(float(max_steer)))
                steer_slider = client.gui.add_slider(
                    label="Steer Angle (deg)",
                    min=-slider_max_steer,
                    max=slider_max_steer,
                    step=0.1,
                    initial_value=0.0,
                    disabled=True,
                )
                brake_slider = client.gui.add_slider(
                    label="Brake Pressure (%)",
                    min=0.0,
                    max=100.0,
                    step=1.0,
                    initial_value=0.0,
                    disabled=True,
                )

                @vel_slider.on_update
                def _(_):
                    if not vehicle_controls_enabled or syncing_vehicle_controls:
                        return
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_speed"):
                        dev.set_speed(vel_slider.value)

                @steer_slider.on_update
                def _(_):
                    if not vehicle_controls_enabled or syncing_vehicle_controls:
                        return
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_steering_angle"):
                        dev.set_steering_angle(steer_slider.value)

                @brake_slider.on_update
                def _(_):
                    if not vehicle_controls_enabled or syncing_vehicle_controls:
                        return
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_brake"):
                        dev.set_brake(brake_slider.value)

                slow_stop_btn = client.gui.add_button("🐌 Slow Stop", disabled=True)
                brake_stop_btn = client.gui.add_button("🛑 Brake Stop", disabled=True)

                @slow_stop_btn.on_click
                def _(_):
                    if not vehicle_controls_enabled:
                        return
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "slow_stop"):
                        dev.slow_stop()
                        vel_slider.value = 0.0
                        brake_slider.value = 0.0

                @brake_stop_btn.on_click
                def _(_):
                    if not vehicle_controls_enabled:
                        return
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "brake_stop"):
                        dev.brake_stop()
                        brake_slider.value = 10.0

                # Vehicle Lighting & Signal Controls (integrated into Vehicle Control folder)
                left_turn_cb = client.gui.add_checkbox("Left Turn Light", initial_value=False, disabled=True)
                right_turn_cb = client.gui.add_checkbox("Right Turn Light", initial_value=False, disabled=True)
                head_light_cb = client.gui.add_checkbox("Head Light", initial_value=False, disabled=True)
                brake_light_cb = client.gui.add_checkbox("Brake Light", initial_value=False, disabled=True)

                @left_turn_cb.on_update
                def _(_):
                    if not vehicle_controls_enabled or syncing_vehicle_controls:
                        return
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_lights"):
                        dev.set_lights(left_turn=left_turn_cb.value)

                @right_turn_cb.on_update
                def _(_):
                    if not vehicle_controls_enabled or syncing_vehicle_controls:
                        return
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_lights"):
                        dev.set_lights(right_turn=right_turn_cb.value)

                @head_light_cb.on_update
                def _(_):
                    if not vehicle_controls_enabled or syncing_vehicle_controls:
                        return
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_lights"):
                        dev.set_lights(head=head_light_cb.value)

                @brake_light_cb.on_update
                def _(_):
                    if not vehicle_controls_enabled or syncing_vehicle_controls:
                        return
                    dev = get_mobile_drive_dev()
                    if dev and hasattr(dev, "set_lights"):
                        dev.set_lights(brake=brake_light_cb.value)

            vehicle_controls = (
                vel_slider, steer_slider, brake_slider, slow_stop_btn, brake_stop_btn,
                left_turn_cb, right_turn_cb, head_light_cb, brake_light_cb,
            )

            def sync_vehicle_controls(drive_status: dict, parsed_can: dict):
                nonlocal syncing_vehicle_controls
                syncing_vehicle_controls = True
                try:
                    telemetry = self._vehicle_control_status(drive_status, parsed_can)
                    vel_slider.value = max(vel_slider.min, min(vel_slider.max, telemetry["speed"]))
                    steer_slider.value = max(steer_slider.min, min(steer_slider.max, telemetry["steer"]))
                    brake_slider.value = max(brake_slider.min, min(brake_slider.max, telemetry["brake"]))
                    left_turn_cb.value = telemetry["left_turn"]
                    right_turn_cb.value = telemetry["right_turn"]
                    head_light_cb.value = telemetry["head_light"]
                    brake_light_cb.value = telemetry["brake_light"]
                finally:
                    syncing_vehicle_controls = False

            # DES service controls. Sliders only stage values; buttons send commands.
            with client.gui.add_folder("DES Control", expand_by_default=True):
                des_status_md = client.gui.add_markdown("**DES**: 대기")

                def des_call(method: str, *args):
                    des = self.robot.devices.get("des_control") if self.robot and self.robot.devices else None
                    if des is None:
                        des_status_md.content = "**DES**: 장치가 설정되지 않았습니다"
                        return
                    try:
                        result = getattr(des, method)(*args)
                        des_status_md.content = f"**DES**: {method} 응답 수신 (동작 완료 상태는 별도 확인)"
                        logger.info(f"[ViserUI] DES {method}: {result}")
                    except Exception as exc:
                        des_status_md.content = f"**DES 오류**: {exc}"
                        logger.error(f"[ViserUI] DES {method} failed: {exc}")

                des_mode = client.gui.add_button_group("Mode", options=["Program", "Manual"])

                @des_mode.on_click
                def _(_):
                    des_call("set_mode_program" if des_mode.value == "Program" else "set_mode_manual")

                with client.gui.add_folder("PTU", expand_by_default=True):
                    ptu_pan = client.gui.add_slider("Pan (deg)", min=-180.0, max=180.0,
                                                    step=1.0, initial_value=0.0)
                    ptu_tilt = client.gui.add_slider("Tilt (deg)", min=-90.0, max=90.0,
                                                     step=1.0, initial_value=0.0)
                    ptu_action = client.gui.add_button_group("PTU Action", options=["Move", "Stop"])
                    ptu_speed = client.gui.add_slider("Speed Code", min=0, max=63,
                                                      step=1, initial_value=32)
                    ptu_speed_apply = client.gui.add_button("Set Speed")

                    @ptu_action.on_click
                    def _(_):
                        if ptu_action.value == "Move":
                            des_call("set_ptu_move", float(ptu_pan.value), float(ptu_tilt.value))
                        else:
                            des_call("set_ptu_stop")

                    @ptu_speed_apply.on_click
                    def _(_):
                        des_call("set_ptu_vel", int(ptu_speed.value))

                with client.gui.add_folder("Mast", expand_by_default=True):
                    mast_target = client.gui.add_slider("Target Height (mm)", min=2900,
                                                        max=9000, step=100, initial_value=2900)
                    mast_current_height_md = client.gui.add_markdown("**Current Height**: 확인 전")
                    mast_action = client.gui.add_button_group(
                        "Mast Action", options=["Extend", "Retract", "Stop"])

                    @mast_action.on_click
                    def _(_):
                        action = mast_action.value
                        if action == "Extend":
                            des_call("set_mast_extend", float(mast_target.value))
                        elif action == "Retract":
                            des_call("set_mast_retract", float(mast_target.value))
                        else:
                            des_call("set_mast_stop")

                with client.gui.add_folder("Erector", expand_by_default=True):
                    erector_action = client.gui.add_button_group(
                        "Erector Action", options=["Fold", "Unfold", "Stop"])

                    @erector_action.on_click
                    def _(_):
                        des_call({"Fold": "set_erector_fold", "Unfold": "set_erector_unfold",
                                  "Stop": "set_erector_stop"}[erector_action.value])

            with client.gui.add_folder("AIC Control", expand_by_default=True):
                fluke_dev = self.robot.devices.get("fluke_sv600") if hasattr(self.robot, "devices") and self.robot.devices else None
                configured_ranges = getattr(fluke_dev, "frequency_ranges", None) or [[20000, 28000]]
                active_ranges = getattr(fluke_dev, "active_frequency_ranges", None)
                initial_range = active_ranges[0] if active_ranges else configured_ranges[0]
                aic_min_frequency = client.gui.add_number(
                    "Min Frequency (Hz)", initial_value=float(initial_range[0]), min=2000, max=52000, step=100
                )
                aic_max_frequency = client.gui.add_number(
                    "Max Frequency (Hz)", initial_value=float(initial_range[1]), min=2000, max=52000, step=100
                )
                aic_range_status_md = client.gui.add_markdown(
                    f"**Active Range:** `{active_ranges[0][0]:.0f}–{active_ranges[0][1]:.0f} Hz`"
                    if active_ranges else "**Active Range:** `Waiting for SoundMap`"
                )
                aic_apply_btn = client.gui.add_button("Apply Frequency Range", color="blue")

                @aic_apply_btn.on_click
                def _(_):
                    min_hz = int(aic_min_frequency.value)
                    max_hz = int(aic_max_frequency.value)
                    if min_hz < 2000 or max_hz > 52000 or min_hz >= max_hz or max_hz - min_hz > 8000:
                        aic_range_status_md.content = "**Error:** Use 2000–52000 Hz with a range of at most 8000 Hz."
                        return
                    device = self.robot.devices.get("fluke_sv600") if hasattr(self.robot, "devices") and self.robot.devices else None
                    if device is None:
                        aic_range_status_md.content = "**Error:** Fluke SV600 is unavailable."
                        return
                    try:
                        applied = device.set_frequency_ranges([[min_hz, max_hz]])
                        aic_range_status_md.content = f"**Active Range:** `{applied[0][0]:.0f}–{applied[0][1]:.0f} Hz`"
                        logger.info(f"[ViserUI] Fluke SV600 frequency range applied: {min_hz}–{max_hz} Hz")
                    except Exception as exc:
                        aic_range_status_md.content = f"**Error:** Could not apply frequency range: `{exc}`"
                        logger.error(f"[ViserUI] Fluke SV600 frequency range update failed: {exc}")

        # Mission Control Tab (Native Viser GUI Window)
        with tabs.add_tab("Mission", viser.Icon.TARGET):
            with client.gui.add_folder("📌 Patrol Route & Task Execution"):
                # Mission Status (reads from drive_executor, persists across browser refresh)
                _init_mission_status = "Idle"
                if hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                    _init_mission_status = getattr(self.robot.drive_executor, "mission_status", "Idle")
                mission_status_md = client.gui.add_markdown(f"**Mission Status**: `{_init_mission_status}`")

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

                _init_poi_enabled = True
                if hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                    _init_poi_enabled = getattr(self.robot.drive_executor, "poi_enabled", True)

                poi_enable_cb = client.gui.add_checkbox("POI Enable", initial_value=_init_poi_enabled)

                @poi_enable_cb.on_update
                def _(_):
                    if hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                        self.robot.drive_executor.poi_enabled = poi_enable_cb.value
                    logger.info(f"[ViserUI] POI Enable set to {poi_enable_cb.value}.")

                _init_hil_sim_enabled = False
                if hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                    _init_hil_sim_enabled = getattr(self.robot.drive_executor, "hil_simulation_enabled", False)

                hil_sim_enable_cb = client.gui.add_checkbox("HIL Simulation", initial_value=_init_hil_sim_enabled)

                @hil_sim_enable_cb.on_update
                def _(_):
                    if hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                        self.robot.drive_executor.hil_simulation_enabled = hil_sim_enable_cb.value
                    logger.info(f"[ViserUI] HIL Simulation set to {hil_sim_enable_cb.value}.")
                @mission_route_dropdown.on_update
                def _(_):
                    val = mission_route_dropdown.value
                    if val not in route_files:
                        mission_route_dropdown.value = route_files[0]
                    else:
                        base_name = os.path.splitext(val)[0]
                        matching_poi = f"{base_name}.poi"
                        if matching_poi in poi_files:
                            mission_poi_dropdown.value = matching_poi

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
                    modal = client.gui.add_modal("Mission 시작 확인")
                    with modal:
                        client.gui.add_markdown("패트롤 로봇 Mission을 시작합니다.")
                        confirm_btn = client.gui.add_button("확인", color="green")
                        cancel_btn = client.gui.add_button("취소", color="red")

                        @confirm_btn.on_click
                        def _(_):
                            modal.close()
                            mission_mgr = getattr(self.robot, "mission_manager", None)
                            drive_exec = getattr(self.robot, "drive_executor", None)
                            if mission_mgr and hasattr(mission_mgr, "start_mission"):
                                mission_mgr.start_mission(route_file, poi_file_name=poi_file)
                            elif drive_exec and hasattr(drive_exec, "start_mission"):
                                drive_exec.start_mission(route_file, poi_file_name=poi_file)
                            mission_status_md.content = "**Mission Status**: `Patrolling...`"
                            logger.info(f"[ViserUI] Start Mission confirmed -> Started MissionManager with route '{route_file}' & POI '{poi_file}'")

                        @cancel_btn.on_click
                        def _(_):
                            modal.close()
                            logger.info("[ViserUI] Start Mission cancelled by user.")

                @abort_mission_btn.on_click
                def _(_):
                    mission_mgr = getattr(self.robot, "mission_manager", None)
                    drive_exec = getattr(self.robot, "drive_executor", None)
                    if mission_mgr and hasattr(mission_mgr, "abort_mission"):
                        mission_mgr.abort_mission()
                    elif drive_exec and hasattr(drive_exec, "abort_mission"):
                        drive_exec.abort_mission()
                    mission_status_md.content = "**Mission Status**: `Aborted`"
                    logger.info("[ViserUI] Abort Mission clicked -> Aborted MissionManager / DriveExecutor.")

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
                            filename = f"record_{datetime.now().strftime('%Y%m%d-%H%M%S')}.route"
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
                            # Refresh route dropdown options in POI Sampler
                            try:
                                updated_routes = get_route_files()
                                poi_sampler_route_dropdown.options = updated_routes
                            except Exception:
                                pass

            # POI Sampler Folder
            with client.gui.add_folder("📍 POI Sampler", expand_by_default=True):
                poi_sampler_status_md = client.gui.add_markdown("**Status**: ⏹️ Idle")

                route_files_for_sampler = get_route_files()
                poi_sampler_route_dropdown = client.gui.add_dropdown(
                    label="Route Files",
                    options=route_files_for_sampler,
                    initial_value=route_files_for_sampler[0]
                )

                poi_sampler_max_samples = client.gui.add_number(
                    label="Max Samples",
                    initial_value=10,
                    min=1,
                    step=1
                )

                poi_sampler_max_height = client.gui.add_number(
                    label="Max Height",
                    initial_value=9100,
                    min=2900,
                    max=9100,
                    step=100
                )

                poi_sampler_gen_btn = client.gui.add_button("📍 Generate", color="blue")

                @poi_sampler_gen_btn.on_click
                def _(event: viser.GuiEvent) -> None:
                    sel_route = poi_sampler_route_dropdown.value
                    if not sel_route or sel_route == "None":
                        modal = client.gui.add_modal("⚠️ Route 파일 오류")
                        with modal:
                            client.gui.add_markdown("선택된 Route 파일이 없습니다.")
                            close_btn = client.gui.add_button("확인", color="red")
                            @close_btn.on_click
                            def _(_):
                                modal.close()
                        return

                    apros_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                    route_dir = os.path.join(apros_root, "route")
                    route_file_path = os.path.join(route_dir, sel_route)

                    if not os.path.exists(route_file_path):
                        poi_sampler_status_md.content = f"**Status**: ❌ File not found: `{sel_route}`"
                        return

                    base_name = os.path.splitext(sel_route)[0]
                    poi_filename = f"{base_name}.poi"
                    poi_file_path = os.path.join(route_dir, poi_filename)

                    num_samples = int(poi_sampler_max_samples.value)
                    max_h = float(poi_sampler_max_height.value)
                    min_h = 2900.0
                    if max_h < min_h:
                        max_h = min_h

                    try:
                        route_data = []
                        with open(route_file_path, "r", encoding="utf-8") as f_in:
                            reader = csv.reader(f_in)
                            header = next(reader, None)
                            lat_idx, lon_idx = 1, 2
                            if header:
                                for i, col in enumerate(header):
                                    col_clean = col.strip().lower()
                                    if col_clean in ["latitude", "lat"]:
                                        lat_idx = i
                                    elif col_clean in ["longitude", "lon", "lng"]:
                                        lon_idx = i

                            for row in reader:
                                if not row or len(row) <= max(lat_idx, lon_idx):
                                    continue
                                try:
                                    lat = float(row[lat_idx])
                                    lon = float(row[lon_idx])
                                    route_data.append((lat, lon))
                                except (ValueError, IndexError):
                                    continue

                        if not route_data:
                            poi_sampler_status_md.content = f"**Status**: ❌ No valid points in `{sel_route}`"
                            return

                        total_pts = len(route_data)
                        n_samp = max(1, num_samples)
                        if total_pts <= n_samp:
                            sampled_data = route_data
                        else:
                            if n_samp == 1:
                                indices = [0]
                            else:
                                indices = [int(round(i * (total_pts - 1) / (n_samp - 1))) for i in range(n_samp)]
                            sampled_data = [route_data[idx] for idx in indices]

                        with open(poi_file_path, "w", encoding="utf-8", newline="") as f_out:
                            writer = csv.writer(f_out)
                            writer.writerow(["index", "latitude", "longitude", "mast_height", "ptu_pan", "ptu_tilt"])
                            for idx, (lat, lon) in enumerate(sampled_data):
                                mast_h = random.randint(int(min_h), int(max_h))
                                writer.writerow([idx, f"{lat:.8f}", f"{lon:.8f}", mast_h, 0.0, 0.0])

                        poi_sampler_status_md.content = f"**Status**: ✅ Generated `{poi_filename}` ({len(sampled_data)} points)"
                        logger.info(f"[ViserUI] POI file generated: {poi_file_path} ({len(sampled_data)} points)")

                        modal = client.gui.add_modal("🎉 POI 생성 완료")
                        with modal:
                            client.gui.add_markdown(
                                f"### 🎉 POI 파일 생성이 완료되었습니다.\n\n"
                                f"- **Route 파일**: `{sel_route}`\n"
                                f"- **POI 파일**: `{poi_filename}`\n"
                                f"- **샘플 수**: {len(sampled_data)} pts\n"
                                f"- **Mast Height 범위**: 2900 ~ {int(max_h)} mm"
                            )
                            close_btn = client.gui.add_button("확인", color="green")
                            @close_btn.on_click
                            def _(_):
                                modal.close()

                    except Exception as gen_err:
                        poi_sampler_status_md.content = f"**Status**: ❌ Error: `{gen_err}`"
                        logger.error(f"[ViserUI] Failed to generate POI: {gen_err}")

        with tabs.add_tab("Data Manage", viser.Icon.DATABASE):
            with client.gui.add_folder("💾 Data Logger", expand_by_default=True):
                datalog_status_md = client.gui.add_markdown("**Status**: ⏹️ Stopped")
                datalog_mode_group = client.gui.add_button_group("Action", options=["⏺️ Record", "⏹️ Stop"])

                @datalog_mode_group.on_click
                def _(event: viser.GuiEvent) -> None:
                    selected = datalog_mode_group.value
                    if selected == "⏺️ Record" and not self._data_logger.is_recording:
                        self._data_logger.start_recording()
                    elif selected == "⏹️ Stop" and self._data_logger.is_recording:
                        self._data_logger.stop_recording()

            with client.gui.add_folder("GNSS/RTK", expand_by_default=True):
                gnss_rtk_status_md = client.gui.add_markdown("**Status**: ⏹️ Stopped")
                gnss_rtk_mode_group = client.gui.add_button_group(
                    "Action", options=["⏺️ Record", "⏹️ Stop"]
                )

                @gnss_rtk_mode_group.on_click
                def _(event: viser.GuiEvent) -> None:
                    if gnss_rtk_mode_group.value == "⏺️ Record":
                        try:
                            self._gnss_rtk_recorder.start()
                        except Exception as error:
                            gnss_rtk_status_md.content = f"**Status**: ❌ {error}"
                            logger.error(f"[ViserUI] GNSS/RTK recording failed to start: {error}")
                    elif gnss_rtk_mode_group.value == "⏹️ Stop":
                        try:
                            result = self._gnss_rtk_recorder.stop()
                            if result and result.get("route_path"):
                                routes = get_route_files()
                                mission_route_dropdown.options = routes
                                mission_route_dropdown.value = os.path.basename(result["route_path"])
                                poi_sampler_route_dropdown.options = routes
                        except Exception as error:
                            gnss_rtk_status_md.content = f"**Status**: ❌ {error}"
                            logger.error(f"[ViserUI] GNSS/RTK export failed: {error}")

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


        # Background update loop for UI Markdown refresh and Camera Stream
        def ui_update_loop():
            nonlocal vehicle_controls_enabled, syncing_vehicle_controls, reset_motion_controls_on_auto
            last_camera_frame_id = None
            last_aic_frame_count = 0
            last_mission_done_state = False
            last_des_height_poll = 0.0
            while self._running:
                try:
                    robot_drive_status_md.content = self._format_status_text(sensor_only=False)
                    sensor_status_md.content = self._format_status_text(sensor_only=True)

                    # Update the selected mode and mission actions from drive status.
                    try:
                        drive_dev = get_mobile_drive_dev()
                        drive_status = drive_dev.get_status() if drive_dev and hasattr(drive_dev, "get_status") else {}
                        parsed_can = drive_status.get("parsed_can_status", {}) if isinstance(drive_status.get("parsed_can_status"), dict) else {}
                        drive_mode_state = str(parsed_can.get("drive_state_mode", drive_status.get("drive_mode", getattr(self.robot, "drive_mode", "Remote")))).strip()
                        is_ad_active = drive_mode_state.startswith("Auto") or drive_mode_state.startswith("AD") or drive_mode_state == "1"

                        reported_mode = "Auto" if is_ad_active else "Remote"
                        self._current_control_mode = reported_mode
                        update_mode_display(reported_mode)
                        start_mission_btn.disabled = not is_ad_active
                        abort_mission_btn.disabled = not is_ad_active
                        if not is_ad_active:
                            vehicle_controls_enabled = False
                            for ctrl in vehicle_controls:
                                ctrl.disabled = True
                            sync_vehicle_controls(drive_status, parsed_can)
                        else:
                            if reset_motion_controls_on_auto:
                                syncing_vehicle_controls = True
                                try:
                                    vel_slider.value = 0.0
                                    steer_slider.value = 0.0
                                finally:
                                    syncing_vehicle_controls = False
                                reset_motion_controls_on_auto = False
                            for ctrl in vehicle_controls:
                                ctrl.disabled = False
                            vehicle_controls_enabled = True
                    except Exception as gui_err:
                        pass

                    # Update Mission Status from mission_manager or drive_executor (syncs Done./Aborted/Patrolling state)
                    try:
                        mission_mgr = getattr(self.robot, "mission_manager", None)
                        drive_exec = getattr(self.robot, "drive_executor", None)
                        executor_status = "Idle"
                        if mission_mgr and hasattr(mission_mgr, "mission_status"):
                            executor_status = getattr(mission_mgr, "mission_status", "Idle")
                        elif drive_exec and hasattr(drive_exec, "mission_status"):
                            executor_status = getattr(drive_exec, "mission_status", "Idle")

                        mission_status_md.content = f"**Mission Status**: `{executor_status}`"

                        # Display completion modal window when patrol mission finishes
                        if executor_status == "Done." and not last_mission_done_state:
                            last_mission_done_state = True
                            modal = client.gui.add_modal("🎉 구간 패트롤 완료")
                            with modal:
                                client.gui.add_markdown(
                                    "### 🎉 구간 패트롤이 완료되었습니다.\n\n"
                                    "설정된 패트롤 경로(Route) 및 POI 태스크 완주가 정상 완료되었습니다."
                                )
                                close_btn = client.gui.add_button("확인", color="green")
                                @close_btn.on_click
                                def _(_):
                                    modal.close()
                            logger.info("[ViserUI] Patrol Mission Completed modal displayed.")
                        elif executor_status != "Done.":
                            last_mission_done_state = False
                    except Exception:
                        pass

                    # Update current mast height from the DES service when available.
                    try:
                        des_dev = self.robot.devices.get("des_control") if self.robot and self.robot.devices else None
                        if des_dev and des_dev.is_connected and time.monotonic() - last_des_height_poll >= 2.0:
                            last_des_height_poll = time.monotonic()
                            curr_h_mm = des_dev.get_mast_height()
                            mast_current_height_md.content = f"**Current Height**: `{curr_h_mm:.0f} mm`"
                    except Exception:
                        mast_current_height_md.content = "**Current Height**: 확인 불가"

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

                    recorder = self._gnss_rtk_recorder
                    if recorder.last_error:
                        gnss_rtk_status_md.content = f"**Status**: ❌ {recorder.last_error}"
                    elif recorder.is_recording:
                        gnss_rtk_status_md.content = f"**Status**: 🔴 Recording (`{os.path.basename(recorder.file_path)}`)"
                    elif recorder.last_result:
                        result = recorder.last_result
                        if result.get("route_path"):
                            route_name = os.path.basename(result["route_path"])
                            if route_name not in mission_route_dropdown.options:
                                routes = get_route_files()
                                mission_route_dropdown.options = routes
                                poi_sampler_route_dropdown.options = routes
                            gnss_rtk_status_md.content = (
                                f"**Status**: ✅ Saved {result['point_count']} route points\n\n"
                                f"**CSV**: `{os.path.basename(result['csv_path'])}` · "
                                f"**KML**: `{os.path.basename(result['kml_path'])}` · "
                                f"**Route**: `{route_name}`"
                            )
                        else:
                            gnss_rtk_status_md.content = (
                                f"**Status**: ⏹️ Saved raw CSV ({result['raw_count']} sentences); "
                                "at least two distinct valid fixes are needed for a route."
                            )
                    else:
                        gnss_rtk_status_md.content = "**Status**: ⏹️ Stopped"

                    # Update RTK Route Builder recording logic
                    try:
                        if rtk_recording_state["is_recording"] and rtk_recording_state["file_handle"]:
                            rtk_dev = next((self.robot.devices[name] for name in ("synerex_syn2000", "synerex_syn3100", "synerex_rtk")
                                            if hasattr(self.robot, "devices") and self.robot.devices and name in self.robot.devices), None)
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

                    # Show the Basler device's latest JPEG, with the IPC copy as fallback.
                    cam_dev = self.robot.devices.get("basler_gige_camera") if hasattr(self.robot, "devices") and self.robot.devices else None
                    if cam_dev is not None:
                        try:
                            latest_frame = getattr(cam_dev, "latest_frame", None)
                            if latest_frame is not None:
                                cam_hdr, frame_bytes = latest_frame
                            else:
                                cam_hdr = getattr(self.robot, "last_camera_header", {}) or {}
                                frame_bytes = getattr(self.robot, "last_camera_frame", None)
                            if isinstance(frame_bytes, bytes) and frame_bytes:
                                frame_id = cam_hdr.get("frame_id", cam_hdr.get("timestamp", id(frame_bytes)))
                                if frame_id != last_camera_frame_id:
                                    decoded_bgr = cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)
                                    if decoded_bgr is not None:
                                        gui_cam_image.image = cv2.cvtColor(decoded_bgr, cv2.COLOR_BGR2RGB)
                                        last_camera_frame_id = frame_id
                        except Exception as cam_err:
                            logger.error(f"[ViserUI] Camera image update error: {cam_err}")

                    fluke_dev = self.robot.devices.get("fluke_sv600") if hasattr(self.robot, "devices") and self.robot.devices else None
                    if fluke_dev is not None:
                        frame_count = getattr(fluke_dev, "frame_count", 0)
                        if frame_count != last_aic_frame_count:
                            frame_bytes = getattr(fluke_dev, "last_frame", None)
                            if frame_bytes:
                                decoded_bgr = cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)
                                if decoded_bgr is not None:
                                    gui_aic_image.image = cv2.cvtColor(decoded_bgr, cv2.COLOR_BGR2RGB)
                                else:
                                    logger.warning("[ViserUI] Fluke SV600 frame is not a supported image.")
                            last_aic_frame_count = frame_count

                    time.sleep(0.1)
                except Exception as e:
                    logger.error(f"[ViserUI] ui_update_loop error: {e}")
                    time.sleep(0.5)

        threading.Thread(target=ui_update_loop, daemon=True).start()

    @staticmethod
    def _vehicle_control_status(drive_status: dict, parsed_can: dict) -> dict:
        """Values shared by the Manual controls and Robot Drive Status display."""
        return {
            "speed": float(parsed_can.get("vehicle_velocity_val", drive_status.get("speed", 0.0))),
            "steer": float(parsed_can.get("vehicle_steer_angle_val", drive_status.get("steer_angle", 0.0))),
            "brake": float(drive_status.get("brake_pressure_cmd", 0.0)),
            "left_turn": bool(drive_status.get("left_turn_light", False)),
            "right_turn": bool(drive_status.get("right_turn_light", False)),
            "head_light": bool(drive_status.get("head_light", False)),
            "brake_light": bool(drive_status.get("brake_light", False)),
        }

    def _format_status_text(self, *, sensor_only: bool) -> str:
        devices = getattr(self.robot, "devices", {}) or {}
        config = getattr(self.robot, "config", None)
        configured = config.get("PLATFORM", "devices", fallback="") if config and config.has_section("PLATFORM") else ""
        device_names = [name.strip() for name in configured.split(",") if name.strip()] if configured else list(devices)
        sensor_names = {"vlp-16", "ouster-sr-128", "xsens_mti_630r", "basler_gige_camera", "fluke_sv600"}
        device_names = [name for name in device_names if (name in sensor_names) == sensor_only]
        lines = []

        def formatted(value):
            if value is None:
                return "N/A"
            if isinstance(value, bool):
                return "ON" if value else "OFF"
            if isinstance(value, float):
                return f"{value:.2f}"
            if isinstance(value, (tuple, list)):
                return ", ".join(formatted(item) for item in value) if len(value) <= 8 else "N/A"
            return str(value)

        for name in device_names:
            device = devices.get(name)
            status = device.get_status() if device and hasattr(device, "get_status") else {}
            if not isinstance(status, dict):
                status = {}
            connected = bool(status.get("connected"))
            if name in ("vlp-16", "ouster-sr-128"):
                last_packet = status.get("last_packet_time")
                connected = connected and last_packet is not None and time.time() - last_packet < 2.0
            elif name == "xsens_mti_630r":
                last_sample = status.get("timestamp")
                connected = connected and last_sample is not None and time.time() - last_sample < 2.0
            lines.append(f"### {'🟢' if connected else '🔴'} {name}")
            if device is None:
                continue
            selected = config.get(name, "status", fallback="") if config and config.has_section(name) else ""
            selected_keys = [key.strip().lower() for key in selected.split(",") if key.strip() and key.strip().lower() != "connected"]
            if not selected_keys:
                continue

            special = {}
            if name == "mobile_drive_s1":
                parsed = status.get("parsed_can_status") or {}
                control = self._vehicle_control_status(status, parsed)
                special = {
                    "channel": ("CAN Channel", status.get("channel")),
                    "drive_mode": ("Drive Mode State", parsed.get("drive_state_mode", status.get("drive_mode"))),
                    "gear": ("Vehicle Gear", parsed.get("vehicle_gear", status.get("gear"))),
                    "speed": ("Vehicle Speed", f"{control['speed']:.1f} km/h"),
                    "steer_angle": ("Vehicle Steer Angle", f"{control['steer']:.1f} deg"),
                    "brake_pressure_cmd": ("Brake Pressure Cmd", f"{control['brake']:.0f} %"),
                    "left_turn_light": ("Left Turn Light Cmd", control["left_turn"]),
                    "right_turn_light": ("Right Turn Light Cmd", control["right_turn"]),
                    "head_light": ("Head Light Cmd", control["head_light"]),
                    "brake_light": ("Brake Light Cmd", control["brake_light"]),
                    "clamping_brake_status": ("Clamping Brake", parsed.get("clamping_brake_status")),
                    "bms_battery_soc": ("Battery SOC", parsed.get("bms_battery_soc")),
                    "emergency_button": ("Emergency Button", parsed.get("emergency_button")),
                    "head_light_actual": ("Head Light Actual", parsed.get("head_light")),
                    "brake_light_actual": ("Brake Light Actual", parsed.get("brake_light")),
                }
            elif name == "vlp-16":
                last_packet = status.get("last_packet_time")
                special = {
                    "ip": ("Expected LiDAR IP", status.get("ip")),
                    "port": ("UDP Port", status.get("port")),
                    "packet_count": ("Received Packets", status.get("packet_count")),
                    "last_packet_time": ("Last Packet", f"{time.time() - last_packet:.1f} s ago" if last_packet else "No packets"),
                    "last_point_count": ("Last Point Count", status.get("last_point_count")),
                }
            elif name == "xsens_mti_630r":
                sample_time = status.get("timestamp")
                acceleration = status.get("acceleration")
                rate_of_turn = status.get("rate_of_turn")
                special = {
                    "timestamp": ("Last Sample", f"{time.time() - sample_time:.1f} s ago" if sample_time else "No samples"),
                    "roll": ("Roll", f"{status['roll']:.2f} deg" if status.get("roll") is not None else None),
                    "pitch": ("Pitch", f"{status['pitch']:.2f} deg" if status.get("pitch") is not None else None),
                    "yaw": ("Yaw", f"{status['yaw']:.2f} deg" if status.get("yaw") is not None else None),
                    "acceleration": ("Accel. XYZ", ", ".join(f"{float(value):.3f}" for value in acceleration) + " m/s²" if acceleration else None),
                    "rate_of_turn": ("Rate of Turn XYZ", ", ".join(f"{math.degrees(float(value)):.3f}" for value in rate_of_turn) + " deg/s" if rate_of_turn else None),
                }
            elif name == "basler_gige_camera":
                timestamp = status.get("timestamp") or status.get("last_frame_timestamp")
                resolution = status.get("resolution")
                timestamp_text = datetime.fromtimestamp(timestamp).strftime("%y-%m-%d %H:%M:%S") if timestamp else None
                special = {
                    "timestamp": ("Timestamp", timestamp_text),
                    "last_frame_timestamp": ("Timestamp", timestamp_text),
                    "resolution": ("Resolution", f"{resolution[0]} × {resolution[1]}" if resolution else None),
                }
            elif name in ("synerex_syn2000", "synerex_syn3100", "synerex_rtk"):
                latitude = status.get("latitude") if connected else None
                longitude = status.get("longitude") if connected else None
                special = {
                    "latitude": ("Latitude", f"{latitude:.7f}" if latitude is not None else None),
                    "longitude": ("Longitude", f"{longitude:.7f}" if longitude is not None else None),
                    "fix_quality": ("Fix Quality", status.get("quality_str")),
                }

            for key in selected_keys:
                if key in special:
                    label, value = special[key]
                else:
                    label, value = key.replace("_", " ").title(), status.get(key)
                if isinstance(value, (dict, bytes, np.ndarray)):
                    value = None
                lines.append(f"- **{label}**: `{formatted(value)}`")

        return "\n".join(lines) if lines else "No devices configured."

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
        return f"- **CAN Bus Status**: {status_str}\n- **Current Steering Cmd**: `{cmd_val}` (-2000: R / +2000: L)"

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

        easting, northing = self.tm_transformer.transform(lon, lat)
        return (
            f"### APROS Patrol Robot Status\n"
            f"- **Speed**: `{speed:.1f} km/h`\n"
            f"- **Steer Angle**: `{steer:.1f}°`\n"
            f"- **Position (WGS84)**: `N {lat:.6f}°, E {lon:.6f}°`\n"
            f"- **TM Position (EPSG:5186)**: `E {easting:.2f}m, N {northing:.2f}m`\n"
            f"- **Gear**: `{gear}` | **Mode**: `{mode}`\n"
            f"- **Dimensions**: `{w_mm} × {l_mm} × {h_mm} mm`"
        )

    def _simulation_loop(self):
        """Update robot kinematics for driving simulation and render real-time VLP-16 point cloud."""
        dt = 0.05
        t = 0.0
        last_ouster_points = None
        while self._running:
            start_time = time.time()
            t += dt

            # Update simulated physics/kinematics in device controller
            self.robot.update_simulation_step(dt=dt)

            # Update real-time RTK TM Position & Heading pose of /robot node in Viser 3D scene
            rtk_dev = next((self.robot.devices[name] for name in ("synerex_syn2000", "synerex_syn3100", "synerex_rtk")
                            if hasattr(self.robot, "devices") and self.robot.devices and name in self.robot.devices), None)
            rtk_status = rtk_dev.get_status() if rtk_dev and hasattr(rtk_dev, "get_status") else {}
            if (rtk_status.get("connected") and rtk_status.get("fix_quality", 0) != 0
                    and rtk_status.get("latitude") is not None and rtk_status.get("longitude") is not None):
                self._last_valid_visual_pose = (
                    rtk_status.get("latitude", self.default_lat),
                    rtk_status.get("longitude", self.default_lon),
                    rtk_status.get("heading", self.default_heading),
                )
            lat_val, lon_val, heading_val = getattr(
                self, "_last_valid_visual_pose", (self.default_lat, self.default_lon, self.default_heading)
            )

            drive_exec = getattr(self.robot, "drive_executor", None)
            simulation_origin_lat = getattr(drive_exec, "origin_lat", None) if drive_exec else None
            simulation_origin_lon = getattr(drive_exec, "origin_lon", None) if drive_exec else None
            if getattr(self.robot, "simulation_mode", False) and drive_exec and simulation_origin_lat is not None and simulation_origin_lon is not None:
                lat_val = simulation_origin_lat + float(self.robot.simulated_x) / 111000.0
                lon_scale = max(0.1, math.cos(math.radians(simulation_origin_lat)))
                lon_val = simulation_origin_lon - float(self.robot.simulated_y) / (111000.0 * lon_scale)
                heading_val = -math.degrees(float(self.robot.simulated_heading))

            rx, ry = self._wgs84_to_tm_viser(lat_val, lon_val)
            if getattr(self.robot, "simulation_mode", False) and drive_exec and simulation_origin_lat is not None and simulation_origin_lon is not None:
                sim_h = float(self.robot.simulated_heading)
                rwxyz = (math.cos(sim_h / 2.0), 0.0, 0.0, math.sin(sim_h / 2.0))
            else:
                rwxyz = self._heading_to_wxyz(heading_val)

            if self.robot_frame_handle:
                self.robot_frame_handle.position = (rx, ry, 0.0)
                self.robot_frame_handle.wxyz = rwxyz

            ground_map_available = self._add_ground_tile_map(lat_val, lon_val)
            if self.ground_grid_handle is not None:
                self.ground_grid_handle.visible = not ground_map_available

            # Update URDF Joint States (Mast height & Steering angle)
            mast_height_m = 2.9  # Ground-relative height in meters (default 2.9m = 2900mm)
            if hasattr(self.robot, "devices") and "telescopic_mast" in self.robot.devices:
                mast = self.robot.devices["telescopic_mast"]
                mast_status = mast.get_status() if hasattr(mast, "get_status") else {}
                mast_height_m = mast_status.get("current_height_m", mast_height_m)

            drive_status = self.robot.get_status() if hasattr(self.robot, "get_status") else {}
            parsed_can = drive_status.get("parsed_can_status", {}) if isinstance(drive_status.get("parsed_can_status"), dict) else {}
            steer_angle_rad = np.radians(self._vehicle_control_status(drive_status, parsed_can)["steer"])

            if self.urdf_model is not None:
                # Convert ground-relative total height into extension beyond the
                # upper chassis, collapsed mast (1.87m), and PTU body top (0.19686m).
                mast_extension = max(
                    0.0,
                    min(6.2, mast_height_m - self.robot_height - 1.87 - 0.19686),
                )
                self.urdf_model.update_cfg({
                    "mast_joint": mast_extension,
                    "front_steer_joint": steer_angle_rad,
                })

            # Real-time VLP-16 point cloud visualization centered at robot frame
            if hasattr(self.robot, 'last_vlp16_points') and self.robot.last_vlp16_points is not None:
                pts = self.robot.last_vlp16_points
                vlp_dev = getattr(self.robot, "devices", {}).get("vlp-16")
                scan_time = getattr(vlp_dev, "last_scan_time", None)
                if len(pts) > 0 and scan_time is not None and time.time() - scan_time <= 2.0:
                    xyz = np.ascontiguousarray(pts[:, :3], dtype=np.float32)
                    colors = np.zeros((len(pts), 3), dtype=np.uint8)

                    # Sensor-frame cloud is placed under the URDF LiDAR link.
                    # The fifth column is the ground-height classification flag.
                    if pts.shape[1] >= 5:
                        is_ground = (pts[:, 4] > 0.5)
                        # Ground points: White (255, 255, 255)
                        colors[is_ground] = [255, 255, 255]
                        # Obstacle / Non-ground points: Red (255, 0, 0)
                        colors[~is_ground] = [255, 0, 0]
                    else:
                        colors[:, :] = [255, 255, 255]
                    colors = np.ascontiguousarray(colors, dtype=np.uint8)
                else:
                    xyz = np.empty((0, 3), dtype=np.float32)
                    colors = np.empty((0, 3), dtype=np.uint8)

                self.vlp16_pc_handle.points = xyz
                self.vlp16_pc_handle.colors = colors

            # Real-time Ouster-SR-128 point cloud visualization
            if hasattr(self.robot, 'last_ouster_points') and self.robot.last_ouster_points is not None:
                pts = self.robot.last_ouster_points
                if pts is not last_ouster_points and len(pts) > 0:
                    xyz = np.ascontiguousarray(pts[:, :3], dtype=np.float32)
                    # Bright purple color for Ouster points (200, 80, 255)
                    colors = np.zeros((len(pts), 3), dtype=np.uint8)
                    colors[:, 0] = 200  # Red channel
                    colors[:, 1] = 80   # Green channel
                    colors[:, 2] = 255  # Blue channel
                    colors = np.ascontiguousarray(colors, dtype=np.uint8)

                    self.ouster_pc_handle.points = xyz
                    self.ouster_pc_handle.colors = colors
                    last_ouster_points = pts

            # Real-time local planner trajectory as a thick yellow line on the ground.
            local_planner = getattr(self.robot, "local_planner", None)
            if local_planner is None and hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                local_planner = getattr(self.robot.drive_executor, "local_planner", None)
            if local_planner and hasattr(local_planner, "best_local_path") and local_planner.best_local_path:
                lpath = local_planner.best_local_path
                if len(lpath) >= 2:
                    drive_exec = getattr(self.robot, "drive_executor", None)
                    route_lat = getattr(drive_exec, "origin_lat", None) if drive_exec else None
                    route_lon = getattr(drive_exec, "origin_lon", None) if drive_exec else None
                    if route_lat is not None and route_lon is not None:
                        anchor_x, anchor_y = self._wgs84_to_tm_viser(route_lat, route_lon)
                    else:
                        anchor_x, anchor_y = self._wgs84_to_tm_viser(self.default_lat, self.default_lon)
                    half_width = 0.10  # 20 cm line width in world coordinates
                    vertices = []
                    for i, pt in enumerate(lpath):
                        prev_pt = lpath[max(0, i - 1)]
                        next_pt = lpath[min(len(lpath) - 1, i + 1)]
                        tx = float(next_pt["x"]) - float(prev_pt["x"])
                        ty = float(next_pt["y"]) - float(prev_pt["y"])
                        length = math.hypot(tx, ty)
                        if length > 1e-6:
                            nx, ny = -ty / length, tx / length
                        else:
                            h = float(pt.get("heading", 0.0))
                            nx, ny = -math.sin(h), math.cos(h)
                        cx, cy = anchor_x + float(pt["x"]), anchor_y + float(pt["y"])
                        vertices.append([cx - nx * half_width, cy - ny * half_width, 0.04])
                        vertices.append([cx + nx * half_width, cy + ny * half_width, 0.04])
                    faces = []
                    for i in range(len(lpath) - 1):
                        a = 2 * i
                        faces.extend(([a, a + 1, a + 2], [a + 1, a + 3, a + 2]))
                    self.local_path_handle.vertices = np.asarray(vertices, dtype=np.float32)
                    self.local_path_handle.faces = np.asarray(faces, dtype=np.uint32)
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
        self._gnss_rtk_recorder.stop()
        if self._data_logger and self._data_logger.is_recording:
            self._data_logger.stop_recording()
        if self._thread:
            self._thread.join(timeout=1.0)
