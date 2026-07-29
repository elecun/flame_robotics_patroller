"""
APROS Tile & Resource HTTP Server Helper (resource/tile_server.py)
Serves Leaflet JS/CSS resources and maptile PNG assets over local HTTP for Leaflet map viewers.
"""

import os
import threading
from typing import Optional
from http.server import HTTPServer, SimpleHTTPRequestHandler
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class TileResourceHTTPRequestHandler(SimpleHTTPRequestHandler):
    """Custom HTTP handler serving /resource and /maptile directories."""

    def translate_path(self, path):
        # Base APROS directory
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        # Strip query parameters
        clean_path = path.split('?')[0]

        if clean_path.startswith("/resource/"):
            rel = clean_path[len("/resource/"):]
            return os.path.join(base_dir, "resource", rel)
        elif clean_path.startswith("/maptile/"):
            rel = clean_path[len("/maptile/"):]
            return os.path.join(base_dir, "maptile", rel)
        else:
            return os.path.join(base_dir, clean_path.lstrip('/'))

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        super().end_headers()

    def log_message(self, format, *args):
        # Quiet down HTTP GET log spam for map tile requests
        pass


class TileServerManager:
    """Manages background HTTP server for map tiles and JS/CSS resources."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8082):
        self.host = host
        self.port = port
        self.httpd: Optional[HTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def start(self):
        try:
            self.httpd = HTTPServer((self.host, self.port), TileResourceHTTPRequestHandler)
            self._thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self._thread.start()
            logger.info(f"[TileServer] HTTP Asset Server started on http://{self.host}:{self.port}")
        except Exception as e:
            logger.error(f"[TileServer] Failed to start HTTP server on port {self.port}: {e}")

    def stop(self):
        if self.httpd:
            try:
                self.httpd.shutdown()
                self.httpd.server_close()
            except Exception:
                pass
            self.httpd = None
        logger.info("[TileServer] HTTP Asset Server stopped.")
