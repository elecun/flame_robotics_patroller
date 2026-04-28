import os
import json
import asyncio
import websockets
import platform
import multiprocessing
from PyQt6.QtCore import QObject, QUrl, QThread, QTimer
from PyQt6.QtWidgets import QVBoxLayout
from util.logger.console import ConsoleLogger

try:
    import vedo
    from vedo import Plotter, Grid, Box
    VEDO_AVAILABLE = True
except ImportError:
    VEDO_AVAILABLE = False

try:
    import webview
    PYWEBVIEW_AVAILABLE = True
except ImportError:
    PYWEBVIEW_AVAILABLE = False

"""
TabNavigation module handles the "Navigation" tab of the application.
On Ubuntu/Linux, it uses pywebview in a separate Process to avoid QWebEngine issues.
"""

def run_map_process(map_url):
    """
    Function to be run in a separate multiprocessing.Process.
    This provides its own main thread for pywebview.
    """
    try:
        import webview
        # Create a window for the map
        window = webview.create_window('Navigation Map', map_url, width=1000, height=800)
        # Start the event loop (blocking)
        webview.start(gui='gtk')
    except Exception as e:
        print(f"Error in Map Process: {e}")

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
        self.map_process = None
        
        # Start WebSocket Server
        self.ws_server = WSServerThread(host="127.0.0.1", port=9090)
        self.ws_server.start()
        self.__console.info("Started local WebSocket server on 127.0.0.1:9090")
        
        # 1. Initialize Map
        self._init_map()

        # 2. Embed vedo Plotter into widget_navigation
        if VEDO_AVAILABLE:
            if hasattr(self.main_ui, "widget_navigation"):
                self._init_3d_view()
            else:
                self.__console.warning("widget_navigation not found in UI — 3D navigation view disabled.")
        else:
            self.__console.error("vedo is not installed. 3D navigation view disabled.")

        QTimer.singleShot(500, self.refresh_view)

        self.__console.debug("TabNavigation initialized")

    def _init_map(self):
        self.__console.debug("Initializing map view...")
        resource_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "resource"))
        map_path = os.path.join(resource_dir, "map.html")
        
        if not os.path.isfile(map_path):
            self.__console.error(f"Map file not found: {map_path}")
            return

        is_linux = platform.system().lower() == "linux"

        # On Linux, use pywebview in a separate PROCESS
        if is_linux and PYWEBVIEW_AVAILABLE:
            self.__console.info("Ubuntu detected. Launching pywebview in a separate process.")
            map_url = QUrl.fromLocalFile(map_path).toString()
            self.map_process = multiprocessing.Process(target=run_map_process, args=(map_url,))
            self.map_process.daemon = True # Ensure it closes when the main app exits
            self.map_process.start()
            
            from PyQt6.QtWidgets import QLabel
            from PyQt6.QtCore import Qt
            if hasattr(self.main_ui, "widget_map"):
                layout = self.main_ui.widget_map.layout() or QVBoxLayout(self.main_ui.widget_map)
                placeholder = QLabel("Map is running in a separate window (pywebview).")
                placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
                layout.addWidget(placeholder)
        else:
            # On macOS or other systems, use QWebEngineView
            self.__console.info(f"Using standard QWebEngineView for map rendering on {platform.system()}.")
            if hasattr(self.main_ui, "widget_map"):
                try:
                    from PyQt6.QtWebEngineWidgets import QWebEngineView
                    from PyQt6.QtWebEngineCore import QWebEngineSettings
                    from PyQt6.QtGui import QColor

                    self.web_view = QWebEngineView(self.main_ui.widget_map)
                    self.web_view.page().setBackgroundColor(QColor("white"))
                    
                    settings = self.web_view.settings()
                    settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
                    settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)

                    layout = self.main_ui.widget_map.layout() or QVBoxLayout(self.main_ui.widget_map)
                    layout.setContentsMargins(0, 0, 0, 0)
                    layout.addWidget(self.web_view)

                    with open(map_path, 'r', encoding='utf-8') as f:
                        html_content = f.read()
                    base_url = QUrl.fromLocalFile(resource_dir + os.path.sep)
                    self.web_view.setHtml(html_content, base_url)
                except Exception as e:
                    self.__console.error(f"Failed to initialize QWebEngineView: {e}")

    def _init_3d_view(self):
        try:
            widget = self.main_ui.widget_navigation
            from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
            layout = widget.layout() or QVBoxLayout(widget)
            layout.setContentsMargins(0, 0, 0, 0)

            vtkWidget = QVTKRenderWindowInteractor(widget)
            layout.addWidget(vtkWidget)

            self._plotter = Plotter(bg="white", qt_widget=vtkWidget, axes=0, interactive=True)
            grid = Grid(pos=(0, 0, 0), s=(20, 20), res=(40, 40), c="black", alpha=0.9)
            robot_box = Box(pos=(0, 0, 0.32), length=1.0, width=2.055, height=0.64, c="gray", alpha=0.9)
            
            self._plotter.add(grid)
            self._plotter.add(robot_box)
            self._plotter.show(grid, robot_box, interactive=False, 
                               camera={"pos": (0, -20, 10), "focalPoint": (0, 10, 0), "viewup": (0, 0, 1)})
        except Exception as e:
            self.__console.error(f"Failed to initialize vedo: {e}")

    def refresh_view(self):
        if hasattr(self, "web_view"):
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
