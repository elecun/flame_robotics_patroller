from PyQt6.QtCore import QObject
from PyQt6.QtGui import QImage, QPixmap
from util.logger.console import ConsoleLogger
import numpy as np

class TabCamera(QObject):
    def __init__(self, main_ui):
        super().__init__()
        self.__console = ConsoleLogger.get_logger()
        self.main_ui = main_ui
        self.__console.debug("TabCamera initialized")

    def on_image_received(self, image_array):
        """
        Slot connected to pylon_camera's signal_updated.
        Receives a numpy image array, converts to QPixmap, scales to label size,
        and displays it in label_camera.
        """
        if not hasattr(self.main_ui, "label_camera"):
            return
        
        try:
            # Determine the image format based on channels
            if image_array.ndim == 2:
                # Grayscale
                h, w = image_array.shape
                bytes_per_line = w
                qimage = QImage(image_array.data, w, h, bytes_per_line, QImage.Format.Format_Grayscale8)
            elif image_array.ndim == 3 and image_array.shape[2] == 3:
                # RGB (Pylon returns BGR in some cases, convert if needed)
                h, w, ch = image_array.shape
                bytes_per_line = ch * w
                qimage = QImage(image_array.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            else:
                self.__console.warning(f"Unsupported image shape: {image_array.shape}")
                return

            # Scale to fit label_camera while preserving aspect ratio
            label = self.main_ui.label_camera
            pixmap = QPixmap.fromImage(qimage).scaled(
                label.width(),
                label.height(),
                aspectRatioMode=__import__("PyQt6.QtCore", fromlist=["Qt"]).Qt.AspectRatioMode.KeepAspectRatio,
                transformMode=__import__("PyQt6.QtCore", fromlist=["Qt"]).Qt.TransformationMode.SmoothTransformation
            )
            label.setPixmap(pixmap)

        except Exception as e:
            self.__console.error(f"Error rendering camera image: {e}")
