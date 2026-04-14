import os
import json
import asyncio
import websockets
from PyQt6.QtCore import QObject, QUrl, QThread
from util.logger.console import ConsoleLogger

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
        # Must be called asynchronously to ensure get_running_loop() returns a valid loop
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

class TabLocation(QObject):
    def __init__(self, main_ui):
        super().__init__()
        self.__console = ConsoleLogger.get_logger()
        self.main_ui = main_ui
        
        # Start WebSocket Server
        self.ws_server = WSServerThread(host="127.0.0.1", port=9090)
        self.ws_server.start()
        self.__console.info("Started local WebSocket server on 127.0.0.1:9090")
        
        # Load map HTML into widget_map by dynamically adding QWebEngineView
        # map.html is placed in resource/ alongside leaflet.js and leaflet.css
        resource_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "resource"))
        map_path = os.path.join(resource_dir, "map.html")
        if os.path.isfile(map_path) and hasattr(self.main_ui, "widget_map"):
            from PyQt6.QtWebEngineWidgets import QWebEngineView
            from PyQt6.QtWebEngineCore import QWebEngineSettings
            from PyQt6.QtWidgets import QVBoxLayout

            # Instantiate web engine and place it inside the widget_map container
            self.web_view = QWebEngineView(self.main_ui.widget_map)
            
            # Allow local content to access remote URLs (needed for OSM tile loading)
            self.web_view.settings().setAttribute(
                QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
            self.web_view.settings().setAttribute(
                QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
            
            layout = QVBoxLayout(self.main_ui.widget_map)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.addWidget(self.web_view)

            self.web_view.load(QUrl.fromLocalFile(map_path))
            self.__console.info(f"Loaded map.html from: {map_path}")
        else:
            self.__console.error(f"Cannot load map.html: map_path exists={os.path.isfile(map_path)}, widget_map exists={hasattr(self.main_ui, 'widget_map')}")
        
    def update_gnss_rtk(self, data):
        # 1. Update Connection and Quality
        connected = data.get("connected", False)

        if not connected:
            self.main_ui.progress_gnss_quality.setFormat("No GNSS Connection")
            self.main_ui.progress_gnss_quality.setValue(0)
            
        else:
            if hasattr(self.main_ui, "progress_gnss_quality"):
                quality_val = data.get("fix_quality", None)
                if quality_val is not None:
                    quality_str = self.quality2str(quality_val)
                    self.main_ui.progress_gnss_quality.setFormat(quality_str)
                    
                    quality_scale_map = {0: 0, 1: 1, 2: 2, 5: 4, 4: 5}
                    bar_value = quality_scale_map.get(quality_val, 0)
                    self.main_ui.progress_gnss_quality.setValue(bar_value)

        # 2. Update Speed (knots to m/s, 1 decimal place)
        speed_knots = data.get("speed_knots")
        if speed_knots is not None and hasattr(self.main_ui, "lcd_speed"):
            speed_ms = float(speed_knots) * 0.514444
            self.main_ui.lcd_speed.display(f"{speed_ms:.1f}")
            
        # 3. Update Time (local datetime)
        local_time_str = data.get("local_datetime")
        if local_time_str and hasattr(self.main_ui, "label_time"):
            self.main_ui.label_time.setText(local_time_str)

        # 4. Update HDOP
        if hasattr(self.main_ui, "lcd_hdop"):
            hdop_val = data.get("hdop")
            
            if hdop_val is None:
                hdop_val = 99.9
                
            self.main_ui.lcd_hdop.display(f"{hdop_val:.1f}")
            
            if hdop_val <= 1.0:
                color = "#00E5FF"
            elif hdop_val <= 2.0:
                color = "#00C853"
            elif hdop_val <= 5.0:
                color = "#B2FF59"
            elif hdop_val <= 10.0:
                color = "#FFD600"
            elif hdop_val <= 20.0:
                color = "#FF6D00"
            else:
                color = "#D50000"
                
            self.main_ui.lcd_hdop.setStyleSheet(f"background-color: {color};")
            
        # 5. Update coordinate on Map
        lat = data.get("latitude")
        lon = data.get("longitude")
        if connected and lat is not None and lon is not None:
            # Broadcast location over WebSocket
            payload = json.dumps({"lat": lat, "lon": lon})
            if hasattr(self, 'ws_server'):
                self.ws_server.broadcast_message(payload)

    def quality2str(self, quality: int) -> str:
        quality_map = {
            0: "Invalid",
            1: "3D",
            2: "DGPS",
            4: "RTK Fixed",
            5: "RTK Float",
            6: "Estimated/DR"
        }
        return quality_map.get(quality, "Unknown")
