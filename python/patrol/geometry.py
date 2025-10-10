
import numpy as np
import pyvista as pv
from util.logger.console import ConsoleLogger
from scipy.spatial.transform import Rotation as R
import vtk

console = ConsoleLogger.get_logger() # logger


class geometry:
    def __init__(self):
        self.geometry_container = {}  # Container to store geometry objects for deletion

    def API_add_coord_frame(self, scene, name:str, pos:list=[0,0,0], ori:list=[0,0,0], size:float=0.1) -> None:
        """
        Add coordinate frame using PyVista's create_axes_marker
        """
        try:
            # Convert lists to numpy arrays
            position = np.array(pos)
            orientation = np.array(ori)
            
            # Create rotation matrix from Euler angles using scipy
            rotation = R.from_euler('xyz', orientation, degrees=False)
            rotation_matrix = rotation.as_matrix()
            
            # Create axes marker using PyVista's built-in function
            axes_marker = pv.create_axes_marker(
                label_color='black',
                line_width=6,
                shaft_length=size*0.8,
                tip_length=size*0.2,
                cone_radius=0.1,
                label_size=(size*0.2, size*0.2)
            )
            
            # Create VTK transform for rotation and translation
            transform = vtk.vtkTransform()
            
            # Apply translation
            transform.Translate(position[0], position[1], position[2])
            
            # Apply rotation
            # Convert rotation matrix to Euler angles for VTK
            r = R.from_matrix(rotation_matrix)
            angles = r.as_euler('xyz', degrees=True)
            transform.RotateX(angles[0])
            transform.RotateY(angles[1])
            transform.RotateZ(angles[2])
            
            # Apply transform to axes marker
            axes_marker.SetUserTransform(transform)
            
            # Add axes marker to scene
            coord_frame_actors = []
            actor = scene.add_actor(axes_marker, name=f"{name}_axes")
            coord_frame_actors.append(actor)
            
            # Add text label at the coordinate frame position
            label_position = position + np.array([size, size, size])  # Offset from origin
            text_actor = scene.add_point_labels(
                points=[label_position], 
                labels=[name],
                point_size=0,  # Hide the point, show only text
                font_size=16,
                text_color='white',
                always_visible=True,  # Keep text visible regardless of zoom
                show_points=False,  # Don't show point markers
                name=f"{name}_label"
            )
            
            coord_frame_actors.append(text_actor)
            
            # Store all actors in the geometry container
            self.geometry_container[name] = coord_frame_actors
            
            console.info(f"Added coordinate frame '{name}' at position {pos} with orientation {ori}")
            
        except Exception as e:
            console.error(f"Failed to add coordinate frame '{name}': {e}")

    def API_add_pcd(self, scene, name: str, path: str, pos: list = [0, 0, 0], ori: list = [0, 0, 0], color: list = [0, 0, 0], point_size: float = 1.0) -> None:
        """
        Add point cloud data from PLY file to the scene
        
        Args:
            scene: PyVista plotter object (self.plotter)
            name: Name identifier for the point cloud
            path: Path to the PLY file
            pos: Position [x, y, z] offset for the point cloud
            ori: Orientation [rx, ry, rz] in radians (Euler angles) 
            color: RGB color [r, g, b] values (0-1 range)
            point_size: Size of the rendered points
        """
        try:
            # Load PLY file using PyVista
            point_cloud = pv.read(path)
            
            # Convert position and orientation to numpy arrays
            position = np.array(pos)
            orientation = np.array(ori)
            
            # Apply transformation if needed
            if not np.allclose(orientation, [0, 0, 0]) or not np.allclose(position, [0, 0, 0]):
                # Create rotation matrix from Euler angles
                rotation = R.from_euler('xyz', orientation, degrees=False)
                rotation_matrix = rotation.as_matrix()
                
                # Create 4x4 transformation matrix
                transform_matrix = np.eye(4)
                transform_matrix[:3, :3] = rotation_matrix
                transform_matrix[:3, 3] = position
                
                # Apply transformation to point cloud
                point_cloud.transform(transform_matrix)
            
            # Set color - if color is [0,0,0], use original colors or default
            if np.allclose(color, [0, 0, 0]):
                # Use original colors if available, otherwise use white
                if 'RGB' in point_cloud.point_data or 'Colors' in point_cloud.point_data:
                    # Keep original colors
                    actor = scene.add_mesh(
                        point_cloud, 
                        point_size=point_size,
                        render_points_as_spheres=True,
                        name=name
                    )
                else:
                    # Default to white points
                    actor = scene.add_mesh(
                        point_cloud, 
                        color='white',
                        point_size=point_size,
                        render_points_as_spheres=True,
                        name=name
                    )
            else:
                # Use specified color (convert to 0-255 range if needed)
                if max(color) <= 1.0:
                    color_255 = [int(c * 255) for c in color]
                else:
                    color_255 = [int(c) for c in color]
                
                actor = scene.add_mesh(
                    point_cloud,
                    color=color_255,
                    point_size=point_size,
                    render_points_as_spheres=True,
                    name=name
                )
            
            # Add text label for the point cloud
            # Calculate label position (use centroid of point cloud + offset)
            bounds = point_cloud.bounds  # [xmin, xmax, ymin, ymax, zmin, zmax]
            center = np.array([
                (bounds[0] + bounds[1]) / 2,
                (bounds[2] + bounds[3]) / 2, 
                (bounds[4] + bounds[5]) / 2
            ])
            label_position = center + np.array([0.1, 0.1, 0.1])  # Offset from center
            
            text_actor = scene.add_point_labels(
                points=[label_position], 
                labels=[name],
                point_size=0,  # Hide the point, show only text
                font_size=16,
                text_color='white',
                always_visible=True,  # Keep text visible regardless of zoom
                show_points=False,  # Don't show point markers
                name=f"{name}_label"
            )
            
            # Store both actors in geometry container
            self.geometry_container[name] = [actor, text_actor]
            
            console.info(f"Added point cloud '{name}' from {path} at position {pos} with orientation {ori}")
            
        except FileNotFoundError:
            console.error(f"PLY file not found: {path}")
        except Exception as e:
            console.error(f"Failed to add point cloud '{name}': {e}")

    def API_load_testpoints(self, scene, name: str, path: str) -> None:
        """
        Load test points from CSV file and display them as labeled points
        """
        try:
            import pandas as pd
            
            # Load CSV file
            df = pd.read_csv(path)
            
            if df.empty:
                console.warning(f"CSV file is empty: {path}")
                return
            
            # Get column names
            columns = list(df.columns)
            console.info(f"CSV columns: {columns}")
            
            # Assume first column is label/index, and next 3 columns are x, y, z coordinates
            if len(columns) < 4:
                console.error(f"CSV file must have at least 4 columns (label, x, y, z), found {len(columns)}")
                return
            
            label_col = columns[0]
            x_col = columns[1]
            y_col = columns[2] 
            z_col = columns[3]
            
            # Extract points and labels
            points = []
            labels = []
            
            for _, row in df.iterrows():
                # Get coordinates
                x = float(row[x_col])
                y = float(row[y_col])
                z = float(row[z_col])
                points.append([x, y, z])
                
                # Get label (convert to string)
                label = str(int(row[label_col]))
                labels.append(label)
            
            console.info(f"Loaded {len(points)} test points from {path}")
            
            # Create point cloud from coordinates
            points_array = np.array(points)
            point_cloud = pv.PolyData(points_array)
            
            # Add points to scene as spheres
            actor = scene.add_mesh(
                point_cloud,
                color='yellow',
                point_size=8.0,
                render_points_as_spheres=True,
                name=f"{name}_points"
            )
            
            # Add labels for each point
            text_actor = scene.add_point_labels(
                points=points_array,
                labels=labels,
                point_size=0,  # Hide the points, show only labels
                font_size=12,
                text_color='white',
                always_visible=True,
                show_points=False,
                name=f"{name}_labels"
            )
            
            # Store both actors in geometry container
            self.geometry_container[name] = [actor, text_actor]
            
            console.info(f"Added {len(points)} test points '{name}' from {path}")
            
        except ImportError:
            console.error("pandas is required to load CSV files. Install with: pip install pandas")
        except FileNotFoundError:
            console.error(f"CSV file not found: {path}")
        except Exception as e:
            console.error(f"Failed to load test points '{name}': {e}")

    def API_add_urdf(self, scene, name: str, robot, base_pos: list = [0, 0, 0], base_ori: list = [0, 0, 0], joint_config: dict = None) -> None:
        """
        Add URDF robot model to the scene using PyVista
        
        Args:
            scene: PyVista plotter object (self.plotter)
            name: Name identifier for the robot
            robot: URDF robot object (already loaded)
            base_pos: Base position [x, y, z] of the robot
            base_ori: Base orientation [rx, ry, rz] in radians (Euler angles)
            joint_config: Dictionary of joint angles {joint_name: angle_in_radians}
        """
        try:
            console.debug(f"Call API_add_urdf : {name}")
            
            # Convert position and orientation to numpy arrays
            position = np.array(base_pos)
            orientation = np.array(base_ori)
            
            # Create rotation matrix from Euler angles
            rotation = R.from_euler('xyz', orientation, degrees=False)
            rotation_matrix = rotation.as_matrix()
            
            # Create 4x4 transformation matrix for base transform
            base_transform = np.eye(4)
            base_transform[:3, :3] = rotation_matrix
            base_transform[:3, 3] = position
            
            # Use provided joint configuration or default to empty dict
            cfg = joint_config if joint_config is not None else {}
            
            # Store all mesh actors for this robot
            robot_actors = []
            
            try:
                # Compute forward kinematics using trimesh
                fk = robot.visual_trimesh_fk(cfg=cfg)
                console.debug(f"Got {len(fk)} meshes from forward kinematics")
                
                # Process each mesh from forward kinematics
                mesh_count = 0
                for tm, T in fk.items():
                    try:
                        # Convert trimesh to PyVista mesh
                        vertices = tm.vertices
                        faces = tm.faces
                        
                        # PyVista expects faces in a specific format: [n_vertices, v0, v1, v2, ...]
                        pv_faces = []
                        for face in faces:
                            pv_faces.extend([3, face[0], face[1], face[2]])  # 3 vertices per triangle
                        
                        # Create PyVista mesh
                        pv_mesh = pv.PolyData(vertices, faces=pv_faces)
                        
                        # Apply link transform from forward kinematics
                        pv_mesh.transform(T)
                        
                        # Apply base transform
                        pv_mesh.transform(base_transform)
                        
                        # Set color - use default gray or material color if available
                        color = 'lightgray'
                        if hasattr(tm.visual, 'material') and tm.visual.material:
                            if hasattr(tm.visual.material, 'diffuse'):
                                # Convert RGBA to RGB if needed
                                diffuse = tm.visual.material.diffuse
                                if len(diffuse) >= 3:
                                    color = diffuse[:3] / 255.0 if max(diffuse[:3]) > 1 else diffuse[:3]
                        
                        # Add mesh to scene with performance optimizations
                        actor = scene.add_mesh(
                            pv_mesh,
                            color=color,
                            name=f"{name}_mesh_{mesh_count}",
                            smooth_shading=True,  # Better visual quality
                            show_edges=False,     # Faster rendering
                            lighting=True,        # Enable lighting for better visuals
                            specular=0.1,         # Reduce specular highlights for performance
                            ambient=0.3           # Increase ambient lighting
                        )
                        robot_actors.append(actor)
                        mesh_count += 1
                        
                        console.debug(f"Added mesh {mesh_count} for robot '{name}': {vertices.shape[0]} vertices")
                        
                    except Exception as mesh_error:
                        console.warning(f"Failed to process mesh {mesh_count} for robot '{name}': {mesh_error}")
                        continue
                
            except Exception as fk_error:
                console.error(f"Failed to compute forward kinematics for robot '{name}': {fk_error}")
                return
            
            # Add text label for the robot
            label_position = position + np.array([0.1, 0.1, 0.1])
            text_actor = scene.add_point_labels(
                points=[label_position],
                labels=[name],
                point_size=0,
                font_size=16,
                text_color='white',
                always_visible=True,
                show_points=False,
                name=f"{name}_label"
            )
            robot_actors.append(text_actor)
            
            # Store all actors in geometry container
            self.geometry_container[name] = robot_actors
            
            console.info(f"Added URDF robot '{name}' with {len(robot_actors)-1} mesh parts at position {base_pos}")
            
        except Exception as e:
            console.error(f"Failed to add URDF robot '{name}': {e}")

    def API_add_ground(self, scene, name: str, end_pos: list = [1, 1, 0], thickness: float = 0.01) -> None:
        """
        Add ground plane as a thin box from origin to specified position
        
        Args:
            scene: PyVista plotter object (self.plotter)
            name: Name identifier for the ground
            end_pos: End position [x, y, z] to define the ground extent
            thickness: Thickness of the ground box (default 0.01)
        """
        try:
            # Convert end position to numpy array
            end_position = np.array(end_pos)
            
            # Calculate box dimensions
            # Box extends from origin to end_pos in x,y and goes down in z
            x_size = abs(end_position[0])
            y_size = abs(end_position[1])
            z_size = thickness
            
            # Calculate box center position
            # Center the box in x,y and place it at -thickness/2 in z (so top is at z=0)
            center_x = end_position[0] / 2
            center_y = end_position[1] / 2
            center_z = -thickness / 2
            
            # Create box bounds for PyVista
            # PyVista Box uses bounds: (xmin, xmax, ymin, ymax, zmin, zmax)
            x_min = center_x - x_size / 2
            x_max = center_x + x_size / 2
            y_min = center_y - y_size / 2
            y_max = center_y + y_size / 2
            z_min = center_z - z_size / 2
            z_max = center_z + z_size / 2
            
            # Create ground box
            ground_box = pv.Box(bounds=(x_min, x_max, y_min, y_max, z_min, z_max))
            
            # Add ground to scene with white color
            actor = scene.add_mesh(
                ground_box,
                color='white',
                name=f"{name}_ground"
            )
            
            # Add coordinate frame at the center of the ground
            ground_center_pos = [center_x, center_y, 0.005]  # Slightly above ground surface
            self.API_add_coord_frame(scene, "ground_center", pos=ground_center_pos, ori=[0,0,0], size=0.1)
            
            # Add text label at corner of the ground
            label_position = np.array([end_position[0] * 0.9, end_position[1] * 0.9, 0.01])
            text_actor = scene.add_point_labels(
                points=[label_position],
                labels=[name],
                point_size=0,
                font_size=16,
                text_color='white',
                always_visible=True,
                show_points=False,
                name=f"{name}_label"
            )
            
            # Store ground actors and coordinate frame together
            # Get ground_center actors from geometry_container
            ground_center_actors = self.geometry_container.get("ground_center", [])
            
            # Combine all actors for this ground
            all_actors = [actor, text_actor] + ground_center_actors
            
            # Store combined actors in geometry container
            self.geometry_container[name] = all_actors
            
            # Remove ground_center from separate entry since it's now part of the ground
            if "ground_center" in self.geometry_container:
                del self.geometry_container["ground_center"]
            
            console.info(f"Added ground '{name}' from origin to {end_pos} with thickness {thickness}")
            
        except Exception as e:
            console.error(f"Failed to add ground '{name}': {e}")

    def API_show_label(self, scene, show: bool) -> None:
        """
        Show or hide all geometry labels
        
        Args:
            scene: PyVista plotter object (self.plotter)
            show: True to show labels, False to hide labels
        """
        try:
            console.info(f"Setting all geometry labels visibility to: {show}")
            
            # Get all geometry names
            geometry_names = list(self.geometry_container.keys())
            
            for name in geometry_names:
                if name in self.geometry_container:
                    actors = self.geometry_container[name]
                    
                    # Find and toggle label actors (those with '_label' in name)
                    for actor in actors:
                        # Check if this is a label actor by looking at its name
                        if hasattr(actor, 'name') and '_label' in str(actor.name):
                            actor.SetVisibility(show)
                        elif hasattr(actor, 'GetMapper'):
                            # Check if this is a text actor by examining the mapper
                            try:
                                mapper = actor.GetMapper()
                                if hasattr(mapper, 'GetInput'):
                                    input_data = mapper.GetInput()
                                    if hasattr(input_data, 'GetPointData'):
                                        point_data = input_data.GetPointData()
                                        # Check if this contains label data
                                        for i in range(point_data.GetNumberOfArrays()):
                                            array_name = point_data.GetArrayName(i)
                                            if array_name and ('label' in array_name.lower() or 'text' in array_name.lower()):
                                                actor.SetVisibility(show)
                                                break
                            except Exception:
                                # If we can't determine, skip this actor
                                pass
            
            # Also check scene actors that contain 'label' in their names
            try:
                for actor_name in scene.renderer.actors:
                    if 'label' in actor_name.lower():
                        actor = scene.renderer.actors[actor_name]
                        actor.SetVisibility(show)
            except Exception as e:
                console.debug(f"Could not access scene renderer actors: {e}")
            
            status = "visible" if show else "hidden"
            console.info(f"All geometry labels are now {status}")
            
        except Exception as e:
            console.error(f"Failed to show/hide labels: {e}")

    def API_remove_geometry(self, scene, name: str) -> bool:
        """
        Remove geometry object from the scene by name
        """
        try:
            if name in self.geometry_container:
                actors = self.geometry_container[name]
                
                # Remove all actors associated with this geometry
                for actor in actors:
                    scene.remove_actor(actor)
                
                # Remove from container
                del self.geometry_container[name]
                console.info(f"Removed geometry '{name}' from scene")
                return True
            else:
                console.warning(f"Geometry '{name}' not found in container")
                return False
                
        except Exception as e:
            console.error(f"Failed to remove geometry '{name}': {e}")
            return False

    def API_clear_all_geometry(self, scene) -> bool:
        """
        Remove all geometry objects from the scene and clear the container
        """
        try:
            removed_count = 0
            geometry_names = list(self.geometry_container.keys())  # Make a copy of keys
            
            for name in geometry_names:
                if self.API_remove_geometry(scene, name):
                    removed_count += 1
            
            console.info(f"Cleared {removed_count} geometry objects from scene")
            return True
            
        except Exception as e:
            console.error(f"Failed to clear all geometries: {e}")
            return False

    def API_list_geometries(self) -> list:
        """
        Get list of all geometry names in the container
        
        Returns:
            list: List of geometry names
        """
        return list(self.geometry_container.keys())