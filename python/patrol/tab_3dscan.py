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

    Embeds a vedo Plotter inside widget_3d (QWidget defined in the .ui file)
    and updates the rendered point cloud whenever new data arrives via signal_updated.
    """

    def __init__(self, main_ui):
        super().__init__()
        self.__console = ConsoleLogger.get_logger()
        self.main_ui = main_ui
        self._plotter = None
        self._pts_actor = None
        self._coord_actor = None

        if not VEDO_AVAILABLE:
            self.__console.error("vedo is not installed. Install via 'pip install vedo'.")
            return

        # Embed vedo Plotter into widget_3d
        if hasattr(self.main_ui, "widget_3d"):
            self._init_plotter()
        else:
            self.__console.warning("widget_3d not found in UI — 3D view disabled.")

        self.__console.debug("Tab3DScan initialized")

    # ------------------------------------------------------------------
    # vedo plotter setup
    # ------------------------------------------------------------------
    def _init_plotter(self):
        try:
            widget = self.main_ui.widget_3d

            from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

            # Create layout if not set
            if widget.layout() is None:
                layout = QVBoxLayout(widget)
                layout.setContentsMargins(0, 0, 0, 0)
            else:
                layout = widget.layout()

            # The VTK Qt interactor is what vedo needs for qt_widget
            vtkWidget = QVTKRenderWindowInteractor(widget)
            layout.addWidget(vtkWidget)

            # Build vedo Qt-embedded plotter
            self._plotter = Plotter(
                bg="#1a1a2e",
                qt_widget=vtkWidget,
                axes=0,
                interactive=True
            )

            # Start event loop (non-blocking)
            self._plotter.show(
                interactive=False,
                camera={"pos": (-5, -5, 5), "focalPoint": (0, 0, 0), "viewup": (0, 0, 1)}
            )

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
            self.__console.warning("3D points received but vedo plotter is unavailable/not initialized.")
            return

        try:
            # self.__console.debug("Received LiDAR payload in 3D Scan tab.")
            xyz = payload.get("xyz")
            intensity = payload.get("intensity")
            
            # Fallbacks for ouster_lidar
            if intensity is None:
                intensity = payload.get("signal")
                if intensity is None:
                    intensity = payload.get("reflectivity")

            # Flatten structured point clouds (e.g., Ouster outputs H x W x 3)
            valid_mask = payload.get("valid_mask")
            if valid_mask is not None and xyz is not None and xyz.ndim == 3:
                xyz = xyz[valid_mask]
                if intensity is not None:
                    intensity = intensity[valid_mask]
            elif xyz is not None and xyz.ndim == 3:
                xyz = xyz.reshape(-1, 3)
                if intensity is not None:
                    intensity = intensity.flatten()

            if xyz is None or len(xyz) == 0:
                self.__console.warning("No valid xyz points in payload after mask.")
                return

            # self.__console.debug(f"Rendering {len(xyz)} points in vedo plotter.")

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

            # Draw coordinate axes if requested
            if payload.get("show_coordinate", False) and self._coord_actor is None:
                from vedo import Line, Assembly
                # X: Red, Y: Green, Z: Blue (Length = 2.0 meters, thin, unlit)
                arr_x = Line((0, 0, 0), (2, 0, 0), c="red", lw=2).lighting("off")
                arr_y = Line((0, 0, 0), (0, 2, 0), c="green", lw=2).lighting("off")
                arr_z = Line((0, 0, 0), (0, 0, 2), c="blue", lw=2).lighting("off")
                self._coord_actor = Assembly([arr_x, arr_y, arr_z])
                self._plotter.add(self._coord_actor)

            self._plotter.render()

        except Exception as e:
            self.__console.error(f"Error rendering point cloud: {e}")
