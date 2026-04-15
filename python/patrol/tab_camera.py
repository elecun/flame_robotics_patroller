from PyQt6.QtCore import QObject, Qt
from PyQt6.QtGui import QImage, QPixmap
from util.logger.console import ConsoleLogger
import numpy as np

class TabCamera(QObject):
    def __init__(self, main_ui):
        super().__init__()
        self.__console = ConsoleLogger.get_logger()
        self.main_ui = main_ui

        # Connect zoom buttons if they exist
        try:
            if hasattr(self.main_ui, "btn_camera_zoomin"):
                self.main_ui.btn_camera_zoomin.clicked.connect(self._on_zoom_in)
            if hasattr(self.main_ui, "btn_camera_zoomout"):
                self.main_ui.btn_camera_zoomout.clicked.connect(self._on_zoom_out)
        except Exception as e:
            self.__console.warning(f"Failed to connect zoom buttons: {e}")

        self.__console.debug("TabCamera initialized")

    # ------------------------------------------------------------------
    # Zoom button handlers
    # ------------------------------------------------------------------
    def _on_zoom_in(self):
        camera = self._get_camera_module()
        if camera:
            camera.zoom_in()

    def _on_zoom_out(self):
        camera = self._get_camera_module()
        if camera:
            camera.zoom_out()

    def _get_camera_module(self):
        """Retrieve the pylon_camera module from the active modules dict."""
        if hasattr(self.main_ui, "active_modules"):
            return self.main_ui.active_modules.get("pylon_camera")
        return None

    # ------------------------------------------------------------------
    # Image display
    # ------------------------------------------------------------------
    def on_image_received(self, image_array):
        """
        Slot connected to pylon_camera's signal_updated.
        Receives a numpy image array (already cropped & resized),
        converts to QPixmap, and displays it in label_camera.
        """
        if not hasattr(self.main_ui, "label_camera"):
            return

        try:
            image_array = np.ascontiguousarray(image_array)

            if image_array.ndim == 2:
                # Grayscale
                h, w = image_array.shape
                qimage = QImage(image_array.data, w, h, w, QImage.Format.Format_Grayscale8)
            elif image_array.ndim == 3 and image_array.shape[2] == 3:
                # RGB / BGR — Pylon Mono/Color converter outputs in RGB888
                h, w, ch = image_array.shape
                qimage = QImage(image_array.data, w, h, ch * w, QImage.Format.Format_RGB888)
            else:
                self.__console.warning(f"Unsupported image shape: {image_array.shape}")
                return

            label = self.main_ui.label_camera
            pixmap = QPixmap.fromImage(qimage).scaled(
                label.width(),
                label.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            label.setPixmap(pixmap)

        except Exception as e:
            self.__console.error(f"Error rendering camera image: {e}")
