import os
import json
import asyncio
import websockets
from PyQt6.QtCore import QObject, QUrl, QThread, QTimer
from PyQt6.QtWidgets import QVBoxLayout
from util.logger.console import ConsoleLogger

try:
    import vedo
    from vedo import Plotter, Grid, Box
    VEDO_AVAILABLE = True
except ImportError:
    VEDO_AVAILABLE = False

"""
TabNavigation module handles the "Navigation" tab of the application.
It uses QWebEngineView to embed the map directly into the UI widget.
"""

class WSServerThread(QThread):
    def __init__(self, host="127.0.0.1", port=9090):
        super().__init__()
        self.host = host
        self.port = port
        self.clients = set()
        self.loop = None

    def run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_ws_server())
        self.loop.run_forever()

    async def start_ws_server(self):
        await websockets.serve(self.ws_handler, self.host, self.port)

    async def ws_handler(self, websocket):
        self.clients.add(websocket)
        try:
            async for _ in websocket:
                pass
        finally:
            self.clients.remove(websocket)

    def broadcast_message(self, message: str):
        if self.loop and self.clients:
            asyncio.run_coroutine_threadsafe(self._broadcast(message), self.loop)
            
    async def _broadcast(self, message: str):
        websockets.broadcast(self.clients, message)

class TabNavigation(QObject):
    def __init__(self, main_ui):
        super().__init__()
        self.__console = ConsoleLogger.get_logger()
        self.main_ui = main_ui
        self._plotter = None
        self.web_view = None
        
        # Start WebSocket Server
        self.ws_server = WSServerThread(host="127.0.0.1", port=9090)
        self.ws_server.start()
        self.__console.info("Started local WebSocket server on 127.0.0.1:9090")
        
        # Initialize Map and 3D View with a slight delay to ensure UI is ready
        QTimer.singleShot(1000, self._deferred_init)

    def _deferred_init(self):
        # 1. Initialize Map (QWebEngineView) 먼저 실행
        self._init_map()

        # 2. VTK(vedo) 초기화는 WebEngine loadFinished 이후로 지연
        # WebEngine GPU 프로세스 초기화와 VTK OpenGL 컨텍스트가 동시에 실행되면
        # OpenGL 컨텍스트 충돌로 segfault 발생 → 2초 후 별도 타이머로 실행
        if VEDO_AVAILABLE and hasattr(self.main_ui, "widget_navigation"):
            QTimer.singleShot(2000, self._init_3d_view)

        self.__console.debug("TabNavigation deferred initialization complete")

    def _on_map_load_finished(self, ok: bool):
        self.__console.debug(f"Map HTML load finished: ok={ok}")
        # 로드 완료 후 500ms 뒤 뷰 갱신 (VTK가 이미 초기화된 경우만)
        QTimer.singleShot(500, self.refresh_view)

    def __del__(self):
        """ Cleanup to prevent segmentation fault """
        try:
            if hasattr(self, 'ws_server') and self.ws_server:
                self.ws_server.quit()
                self.ws_server.wait()
            if hasattr(self, 'web_view') and self.web_view:
                self.web_view.setParent(None)
                self.web_view.deleteLater()
        except:
            pass

    def _init_map(self):
        self.__console.debug("Initializing map view in widget_map...")
        resource_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "resource"))
        map_path = os.path.join(resource_dir, "map.html")

        if os.path.isfile(map_path) and hasattr(self.main_ui, "widget_map"):
            from PyQt6.QtWebEngineWidgets import QWebEngineView
            from PyQt6.QtWebEngineCore import QWebEngineSettings
            from PyQt6.QtWidgets import QSizePolicy
            from PyQt6.QtGui import QColor

            self.web_view = QWebEngineView(self.main_ui.widget_map)
            self.web_view.page().setBackgroundColor(QColor("white"))

            # Ubuntu 22.04에서 렌더러 프로세스 크래시 감지 (black screen 원인 진단)
            self.web_view.page().renderProcessTerminated.connect(
                lambda status, code: self.__console.error(
                    f"WebEngine renderer terminated! status={status}, code={code}"
                )
            )

            # 로드 상태 진단 시그널
            self.web_view.loadStarted.connect(lambda: self.__console.debug("Map HTML load started"))
            self.web_view.loadFinished.connect(self._on_map_load_finished)

            # 로컬 파일에서 WebSocket(127.0.0.1) 접근 허용
            settings = self.web_view.settings()
            settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
            settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)

            # web_view를 부모 위젯 크기에 맞게 명시적으로 설정
            self.web_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            parent_widget = self.main_ui.widget_map
            self.web_view.resize(parent_widget.size())

            layout = parent_widget.layout() or QVBoxLayout(parent_widget)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)
            layout.addWidget(self.web_view)

            # setHtml + base_url 조합은 Ubuntu에서 로컬 파일 보안 정책과 충돌할 수 있음
            # load(QUrl.fromLocalFile()) 방식이 더 안정적임
            map_url = QUrl.fromLocalFile(map_path)
            self.web_view.load(map_url)
            self.__console.info(f"Loading map.html via QUrl.fromLocalFile: {map_url.toString()}")
        else:
            self.__console.error(f"Cannot load map.html: exists={os.path.isfile(map_path)}")

    def _init_3d_view(self):
        self.__console.debug("Initializing vedo 3D plotter in widget_navigation...")
        try:
            widget = self.main_ui.widget_navigation
            from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

            if widget.layout() is None:
                layout = QVBoxLayout(widget)
                layout.setContentsMargins(0, 0, 0, 0)
            else:
                layout = widget.layout()

            vtkWidget = QVTKRenderWindowInteractor(widget)
            layout.addWidget(vtkWidget)

            # Build vedo Plotter with white background
            self._plotter = Plotter(bg="white", qt_widget=vtkWidget, axes=0, interactive=True)

            # Create Grid (50cm interval) with black lines
            grid = Grid(pos=(0, 0, 0), s=(20, 20), res=(40, 40), c="black", alpha=0.9)
            self._plotter.add(grid)

            # Create a black box representing the robot
            robot_box = Box(pos=(0, 0, 0.32), length=1.0, width=2.055, height=0.64, c="gray", alpha=0.9)
            self._plotter.add(robot_box)

            # Set Camera viewpoint
            self._plotter.show(grid, robot_box, interactive=False, camera={"pos": (0, -20, 10),"focalPoint": (0, 10, 0),"viewup": (0, 0, 1)})

        except Exception as e:
            self.__console.error(f"Failed to initialize vedo plotter in widget_navigation: {e}")

    def refresh_view(self):
        if hasattr(self, "web_view") and self.web_view:
            self.web_view.update()
        if self._plotter:
            self._plotter.render()

    def update_gnss_rtk(self, data):
        lat = data.get("latitude")
        lon = data.get("longitude")
        connected = data.get("connected", False)
        if connected and lat is not None and lon is not None:
            if hasattr(self.main_ui, "label_latlon"):
                self.main_ui.label_latlon.setText(f"{lat:.6f}, {lon:.6f}")
            payload = json.dumps({"lat": lat, "lon": lon})
            if hasattr(self, 'ws_server'):
                self.ws_server.broadcast_message(payload)
        
        if hasattr(self.main_ui, "progress_gnss_quality"):
            q = data.get("fix_quality", 0)
            self.main_ui.progress_gnss_quality.setFormat(self.quality2str(q))
            self.main_ui.progress_gnss_quality.setValue({0:0, 1:1, 2:2, 5:4, 4:5}.get(q, 0))

        speed = data.get("speed_knots")
        if speed is not None and hasattr(self.main_ui, "lcd_speed"):
            self.main_ui.lcd_speed.display(f"{float(speed)*0.514444:.1f}")

    def quality2str(self, quality: int) -> str:
        return {0:"Invalid", 1:"3D", 2:"DGPS", 4:"RTK Fixed", 5:"RTK Float", 6:"Estimated"}.get(quality, "Unknown")
