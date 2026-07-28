import time
import numpy as np
import viser

server = viser.ViserServer()

# Viser theme & GUI setup
server.gui.configure_theme(dark_mode=True)
server.gui.set_panel_label("Multi-Camera & Sensor Dashboard")

# ==========================================
# 1. 메인 탭 그룹 (Left / Right 패널 구성)
# ==========================================
tabs = server.gui.add_tab_group()

# 좌측 카메라 탭
with tabs.add_tab("Main Cam", viser.Icon.CAMERA):
    with server.gui.add_folder("📹 Main Camera (Front)"):
        img_left = np.zeros((240, 320, 3), dtype=np.uint8)
        cam_gui_1 = server.gui.add_image(
            img_left, label="Front Camera (320x240)", format="jpeg", jpeg_quality=70
        )

# 우측 카메라 & 상태 탭
with tabs.add_tab("Sub Cam & Status", viser.Icon.EYE):
    with server.gui.add_folder("📷 Sub Camera (Thermal/Depth)"):
        img_right = np.zeros((240, 320, 3), dtype=np.uint8)
        cam_gui_2 = server.gui.add_image(
            img_right, label="Thermal / Depth", format="jpeg", jpeg_quality=70
        )
    with server.gui.add_folder("📊 System Status"):
        server.gui.add_markdown("**System Status**")
        status_text = server.gui.add_text("FPS", initial_value="30 FPS", disabled=True)


# ==========================================
# 2. 실시간 프레임 갱신 루프
# ==========================================
print("[panel.py] Starting multi-camera frame update loop...")
try:
    while True:
        # 각 패널에 들어갈 프레임 생성
        frame_left = np.random.randint(0, 255, size=(240, 320, 3), dtype=np.uint8)
        frame_right = np.random.randint(0, 255, size=(240, 320, 3), dtype=np.uint8)

        # 각각의 GUI 인스턴스 업데이트
        cam_gui_1.image = frame_left
        cam_gui_2.image = frame_right

        time.sleep(0.03)
except KeyboardInterrupt:
    print("[panel.py] Stopped.")
    server.stop()