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
            titlebar_content=viser.theme.TitlebarConfig(
                buttons=(),
                image=None
            ),
            control_layout="floating",
            control_width="large",
            dark_mode=True,
            show_logo=False,
            brand_color=(30, 144, 255)
        )
        self.server.gui.set_panel_label("APROS Control Center")
        
        # Scale: Robot dimensions 1000mm W, 2055mm L, 640mm H => 1.0m W, 2.055m L, 0.64m H
        self.robot_width = 1.000   # meters (1000 mm)
        self.robot_length = 2.055  # meters (2055 mm)
        self.robot_height = 0.640  # meters (640 mm)

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
        # 1. Single Ground Grid
        self.server.scene.add_grid(
            name="/ground_grid",
            width=50.0,
            height=50.0,
            plane="xz",
            cell_color=(80, 90, 100),
            section_color=(140, 150, 160)
        )

        # 2. World Axes Frame with X, Y, Z labels (Scene Tree show/hide support)
        self.server.scene.add_frame(
            name="/world_axes",
            show_axes=True,
            axes_length=1.5,
            axes_radius=0.03,
            origin_radius=0.04
        )
        
        self.server.scene.add_label(
            name="/world_axes/label_x",
            text="X",
            position=(1.7, 0.0, 0.0)
        )
        self.server.scene.add_label(
            name="/world_axes/label_y",
            text="Y",
            position=(0.0, 1.7, 0.0)
        )
        self.server.scene.add_label(
            name="/world_axes/label_z",
            text="Z",
            position=(0.0, 0.0, 1.7)
        )

        # 3. Main Patrol Robot Model (pure box shape: 1000 x 2055 x 640 mm)
        self.server.scene.add_box(
            name="/robot/chassis",
            dimensions=(self.robot_width, self.robot_height, self.robot_length),
            color=(30, 144, 255),  # Sleek Blue
            position=(0.0, self.robot_height / 2.0, 0.0)
        )

        # Front Heading Bumper Indicator
        self.server.scene.add_box(
            name="/robot/heading_indicator",
            dimensions=(self.robot_width * 0.8, 0.08, 0.2),
            color=(255, 69, 0),
            position=(0.0, self.robot_height + 0.04, self.robot_length / 2.0 - 0.15)
        )

    def _setup_client_ui(self, client: viser.ClientHandle):
        """Setup UI components for newly connected client."""
        # Standard Y-Up Camera Orientation
        client.camera.up_direction = (0.0, 1.0, 0.0)
        client.camera.position = (0.0, 3.5, -6.0)
        client.camera.look_at = (0.0, 0.5, 3.0)

        # 1. Left Telemetry Dashboard
        with client.gui.add_folder("📊 Telemetry Dashboard (APROS System)"):
            dashboard_md = client.gui.add_markdown(self._format_dashboard_text())

        # 2. Remote Control GUI Folder
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
            # Initial disabled state based on Control Mode (Manual enables, Auto/E-stop disables)
            is_manual = mode_dropdown.value.startswith("Manual")
            speed_slider.disabled = not is_manual
            steer_slider.disabled = not is_manual

        estop_button = client.gui.add_button(
            label="🚨 EMERGENCY STOP (P Gear & STOP)",
            color="red"
        )

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
                    can_status_md.content = self._format_can_status_text()
                    time.sleep(0.1)
                except Exception:
                    break

        threading.Thread(target=ui_update_loop, daemon=True).start()

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

        return f"""
<div style="min-width: 420px; font-family: 'Inter', sans-serif; padding: 4px;">
    <h2 style="color: #00E676; margin-top: 0; margin-bottom: 8px; font-size: 1.4em;">🚀 APROS Patrol Robot Status</h2>
    <div style="background: rgba(255,255,255,0.05); padding: 12px; border-radius: 8px; border-left: 4px solid #308EFF;">
        <p style="margin: 4px 0; font-size: 1.1em;">⚡ <b>Speed (속도)</b>: <span style="color:#00E676; font-size: 1.4em; font-weight:bold;">{speed:.1f} km/h</span></p>
        <p style="margin: 4px 0; font-size: 1.05em;">🧭 <b>Steer Angle (조향각)</b>: <span style="color:#FFD700; font-size: 1.2em; font-weight:bold;">{steer:.1f}°</span></p>
        <p style="margin: 4px 0; font-size: 1.05em;">📍 <b>Latitude (위도)</b>: <code style="font-size: 1.1em; color: #E0E0E0;">N {lat:.6f}°</code></p>
        <p style="margin: 4px 0; font-size: 1.05em;">📍 <b>Longitude (경도)</b>: <code style="font-size: 1.1em; color: #E0E0E0;">E {lon:.6f}°</code></p>
        <p style="margin: 4px 0; font-size: 1.05em;">⚙️ <b>Gear</b>: <b style="color: #FFD700;">{gear}</b> | <b>Mode</b>: <b style="color: #00E676;">{mode}</b></p>
        <p style="margin: 4px 0; font-size: 0.95em; color: #AAAAAA;">📐 <b>Dimensions</b>: 1000 × 2055 × 640 mm</p>
    </div>
</div>
        """

    def _simulation_loop(self):
        """Update robot kinematics for driving simulation."""
        dt = 0.05
        t = 0.0
        while self._running:
            start_time = time.time()
            t += dt

            # Update simulated physics/kinematics in device controller
            self.robot.update_simulation_step(dt=dt)

            elapsed = time.time() - start_time
            time.sleep(max(0.0, dt - elapsed))

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self._thread.start()
        print(f"[APROS Viser UI] Server started on http://{self.host}:{self.port}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
