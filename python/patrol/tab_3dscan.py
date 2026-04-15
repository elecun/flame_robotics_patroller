from PyQt6.QtCore import QObject
from PyQt6.QtWidgets import QVBoxLayout
from util.logger.console import ConsoleLogger
import numpy as np

try:
    import vedo
    from vedo import Plotter, Points
    VEDO_AVAILABLE = True
except ImportError:
    VEDO_AVAILABLE = False


class Tab3DScan(QObject):
    """
    Tab controller for the 3D LiDAR scan view.

    Embeds a vedo Plotter inside widget_navigation (QWidget defined in the .ui file)
    and updates the rendered point cloud whenever new data arrives via signal_updated.
    """

    def __init__(self, main_ui):
        super().__init__()
        self.__console = ConsoleLogger.get_logger()
        self.main_ui = main_ui
        self._plotter = None
        self._pts_actor = None

        if not VEDO_AVAILABLE:
            self.__console.error("vedo is not installed. Install via 'pip install vedo'.")
            return

        # Embed vedo Plotter into widget_navigation
        if hasattr(self.main_ui, "widget_navigation"):
            self._init_plotter()
        else:
            self.__console.warning("widget_navigation not found in UI — 3D view disabled.")

        self.__console.debug("Tab3DScan initialized")

    # ------------------------------------------------------------------
    # vedo plotter setup
    # ------------------------------------------------------------------
    def _init_plotter(self):
        try:
            widget = self.main_ui.widget_navigation

            # Create layout if not set
            if widget.layout() is None:
                layout = QVBoxLayout(widget)
                layout.setContentsMargins(0, 0, 0, 0)
            else:
                layout = widget.layout()

            # Build vedo Qt-embedded plotter
            self._plotter = Plotter(
                bg="#1a1a2e",
                qt_widget=widget,
                axes=1,
                interactive=True
            )

            # Place the vedo canvas widget inside our Qt container
            layout.addWidget(self._plotter.interactor)

            # Start event loop (non-blocking)
            self._plotter.show(interactive=False)
            self.__console.info("vedo 3D plotter embedded in widget_navigation.")

        except Exception as e:
            self.__console.error(f"Failed to initialize vedo plotter: {e}")
            self._plotter = None

    # ------------------------------------------------------------------
    # Signal handler — called by window.py on 'velodyne_lidar' signal
    # ------------------------------------------------------------------
    def on_lidar_data_received(self, payload: dict):
        """
        Receives a dict with keys:
            xyz        : (N, 3) float32 numpy array of point positions
            intensity  : (N,)   float32 numpy array
            point_size : int
            colormap   : str
            bg_color   : str
        """
        if not VEDO_AVAILABLE or self._plotter is None:
            return

        try:
            xyz = payload.get("xyz")
            intensity = payload.get("intensity")

            if xyz is None or len(xyz) == 0:
                return

            # Normalize intensity for colormap (0.0 – 1.0)
            if intensity is not None and len(intensity) == len(xyz):
                i_min, i_max = intensity.min(), intensity.max()
                if i_max > i_min:
                    scalars = (intensity - i_min) / (i_max - i_min)
                else:
                    scalars = np.zeros(len(xyz), dtype=np.float32)
            else:
                # Color by Z height if no intensity
                z = xyz[:, 2]
                z_min, z_max = z.min(), z.max()
                scalars = (z - z_min) / (z_max - z_min + 1e-6)

            colormap = payload.get("colormap", "jet")
            point_size = int(payload.get("point_size", 3))

            # Build / update Points actor
            cloud = Points(xyz, r=point_size)
            cloud.cmap(colormap, scalars, on="points")

            # Replace previous actor
            if self._pts_actor is not None:
                self._plotter.remove(self._pts_actor)

            self._pts_actor = cloud
            self._plotter.add(cloud)
            self._plotter.render()

        except Exception as e:
            self.__console.error(f"Error rendering point cloud: {e}")
