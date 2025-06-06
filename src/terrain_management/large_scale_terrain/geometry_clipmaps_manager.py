__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from src.terrain_management.large_scale_terrain.geometry_clipmaps import (
    GeometryClipmapConf,
    GeoClipmap

)
from dataclasses import dataclass, field
from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops
from pxr import UsdGeom, Sdf, Usd, Gf
from typing import Tuple
import numpy as np
import warp as wp
import omni
import logging
import matplotlib.pyplot as plt
import pandas as pd


full_debugging = False  # Set to True to enable full debugging features, such as visualizing points as spheres


@dataclass
class GeoClipmapManagerConf:
    """
    Args:
        root_path (str): path to the root of the clipmap.
        geo_clipmap_specs (GeometryClipmapConf): specifications for the clipmap.
        mesh_position (tuple): position of the mesh.
        mesh_orientation (tuple): orientation of the mesh.
        mesh_scale (tuple): scale of the mesh.
    """

    root_path: str = "/World"
    geo_clipmap_specs: GeometryClipmapConf = field(default_factory=dict)
    mesh_position: tuple = (0.0, 0.0, 0.0)
    mesh_orientation: tuple = (0.0, 0.0, 0.0, 1.0)
    mesh_scale: tuple = (1.0, 1.0, 1.0)
    debug_roi_center_world_init: Tuple[float, float] = (0.0, 0.0)


class GeoClipmapManager:
    """
    Class to manage the geometry clipmap.
    """

    def __init__(
        self,
        cfg: GeoClipmapManagerConf,
        interpolation_method: str = "bilinear",
        acceleration_mode: str = "hybrid",
        name_prefix: str = "",
        profiling: bool = False,
        stage: Usd.Stage = None,
    ) -> None:
        """
        Args:
            cfg (GeoClipmapManagerConf): configuration for the clipmap.
            interpolation_method (str): method to use for interpolation.
            acceleration_mode (str): mode to use for acceleration.
            name_prefix (str): prefix to add to the mesh name.
        """

        self._geo_clipmap = GeoClipmap(
            cfg.geo_clipmap_specs,
            interpolation_method=interpolation_method,
            acceleration_mode=acceleration_mode,
            profiling=profiling,
        )
        self._root_path = cfg.root_path
        self.profiling = profiling
        self._stage = stage

        self._mesh_pos = cfg.mesh_position
        self._mesh_rot = cfg.mesh_orientation
        self._mesh_scale = cfg.mesh_scale

        self._og_mesh_path = self._root_path + "/Terrain/terrain_mesh" + name_prefix
        self._mesh_path = self._root_path + "/Terrain/terrain_mesh" + name_prefix

        #For debugging, from Yannic
        self._name_prefix = name_prefix
        self._debug_spheres_root_path_str = ""
        self._debug_sphere_prims = []
        self._visualize_points_sphere_radius = 0.1 
        self._visualize_points_sphere_color = Gf.Vec3f(1.0, 0.0, 0.0) # Red color for spheres
         # ROI for debug spheres (world coordinates)
        self._debug_roi_center_world = Gf.Vec2d(cfg.debug_roi_center_world_init[0], cfg.debug_roi_center_world_init[1])
        self._debug_roi_radius = 20.0  # e.g., 2 * block_size (2 * 20 = 40)
        # end of debugging


        self.create_Xforms()
        self.update_topology = True

    # For debugging, from Yannic
    def _visualize_mesh_points_as_spheres(self, points_to_visualize: np.ndarray):
        """
        Visualizes given points as small spheres once, using default material.
        Does not update or remove them later. For coarse mesh debug only.
        """

        output_path = "/workspace/omnilrs/Debugging_files"  # Replace with your desired path



        if not self._stage or self._debug_spheres_root_path_str:
            return  # Skip if no stage or already initialized

        if points_to_visualize is None or len(points_to_visualize) == 0:
            return  # Nothing to visualize

        root_path = "/World" + "/DebugVertexSpheres"
        root_prim = self._stage.GetPrimAtPath(root_path)
        if not root_prim:
            root_prim = self._stage.DefinePrim(root_path, "Xform")
        else:
            # Clear old children (if re-used prim)
            for child in root_prim.GetChildren():
                self._stage.RemovePrim(child.GetPath())
        debug_xform_path = root_path + "/debugging_xform"
        robot_root_prim = self._stage.GetPrimAtPath(debug_xform_path)
        if not robot_root_prim:
            robot_root_prim = self._stage.DefinePrim(debug_xform_path, "Xform")
        else:
            # Clear old children (if re-used prim)
            for child in robot_root_prim.GetChildren():
                self._stage.RemovePrim(child.GetPath())

        spheres_added = 0

        robot_x_global = 480
        robot_y_global = -580


        if spheres_added == 0:
            # Add a sphere at the robot position
            spheres_added += 1
            robot_sphere_path = debug_xform_path + "/robot_sphere"
            origin_sphere_path = debug_xform_path + "/origin_sphere"
            # Create a sphere at the robot position

            sphere_prim = UsdGeom.Sphere.Define(self._stage, Sdf.Path(robot_sphere_path)).GetPrim()
            sphere_geom = UsdGeom.Sphere(sphere_prim)
            sphere_geom.GetRadiusAttr().Set(self._visualize_points_sphere_radius)

            # Apply only transform (no material)
            xformable = UsdGeom.Xformable(sphere_prim)
            translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(Gf.Vec3d(robot_x_global, robot_y_global, 0))



            #Create a sphere at the origin position
            spheres_added += 1
            sphere_prim = UsdGeom.Sphere.Define(self._stage, Sdf.Path(origin_sphere_path)).GetPrim()
            sphere_geom = UsdGeom.Sphere(sphere_prim)
            sphere_geom.GetRadiusAttr().Set(self._visualize_points_sphere_radius)
            # Apply only transform (no material)
            xformable = UsdGeom.Xformable(sphere_prim)
            translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            origin_position_from_robot_coming = Gf.Vec3d(self._debug_roi_center_world[0], self._debug_roi_center_world[1], 0)
            translate_op.Set(origin_position_from_robot_coming)  # Origin sphere at (0, 0, 0)
            logging.info(f"CoordinateSystem ROI center of world, which should be starting position of robot from largescale setting is {self._debug_roi_center_world} with radius {self._debug_roi_radius}")

            #create a sphere of the first point in the points_to_visualize
            spheres_added+= 1
            first_point = points_to_visualize[0]
            sphere_path = debug_xform_path + f"/point_sphere_test_single"
            sphere_prim = UsdGeom.Sphere.Define(self._stage, Sdf.Path(sphere_path)).GetPrim()
            sphere_geom = UsdGeom.Sphere(sphere_prim)
            sphere_geom.GetRadiusAttr().Set(self._visualize_points_sphere_radius)
            # Apply only transform (no material)
            xformable = UsdGeom.Xformable(sphere_prim)
            translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(Gf.Vec3d(*map(float, first_point)))  # First point sphere
            logging.info(f"CoordinateSystem Visualizing first point sphere at {first_point} with path {sphere_path}")
           

        region_around_pit = points_to_visualize[points_to_visualize[:, 2] == -1620]
        if len(region_around_pit) > 0:
            min_x_pit = np.min(region_around_pit[:, 0])
            max_x_pit = np.max(region_around_pit[:, 0])
            min_y_pit = np.min(region_around_pit[:, 1])
            max_y_pit = np.max(region_around_pit[:, 1])

            # Step 2: Extend bounds (e.g., 50 units in each direction)
            extension = 300  # adjust as needed

            min_x_ext = min_x_pit - extension
            max_x_ext = max_x_pit + extension
            min_y_ext = min_y_pit - extension
            max_y_ext = max_y_pit + extension

            # Step 3: Select all points within the extended region
            extended_region_around_pit = points_to_visualize[
                (points_to_visualize[:, 0] >= min_x_ext) &
                (points_to_visualize[:, 0] <= max_x_ext) &
                (points_to_visualize[:, 1] >= min_y_ext) &
                (points_to_visualize[:, 1] <= max_y_ext)
            ]

            df = pd.DataFrame({
                'x': extended_region_around_pit[:, 0],
                'y': extended_region_around_pit[:, 1],
                'z': extended_region_around_pit[:, 2],
            })
            z_grid = df.pivot(index='y', columns='x', values='z')


                    # Plot all points
            #plot only edged region around pit

            fig = plt.figure(figsize=(16, 8))  # Adjusted figure size for side-by-side plots

            # 3D scatter plot
            ax1 = fig.add_subplot(121, projection='3d')  # Left plot
            sc = ax1.scatter(
                extended_region_around_pit[:, 0],
                extended_region_around_pit[:, 1],
                extended_region_around_pit[:, 2],
                c=extended_region_around_pit[:, 2],   # color by depth
                cmap='viridis',                       # choose colormap
                s=1,
                label='Pit Region Extended'
            )
            # Set labels and title for 3D scatter plot
            ax1.set_xlabel('X')
            ax1.set_ylabel('Y')
            ax1.set_zlabel('Z (Height)')
            ax1.set_title('Point Cloud with Pit Region Highlighted')
            ax1.legend()

            # Plot heightmap
            ax2 = fig.add_subplot(122)
            heightmap = ax2.imshow(
                z_grid.values,
                cmap='viridis',
                origin='lower',  # or 'upper' depending on convention
                aspect='auto'
            )
            ax2.set_title('Heightmap (Top-Down View)')
            ax2.set_xlabel('X')
            ax2.set_ylabel('Y')
            cbar = plt.colorbar(heightmap, ax=ax2, pad=0.1)
            cbar.set_label('Height (Z)')

            # Save the combined plot
            plt.tight_layout()
            name = "pit_region_combined_plot" + self._name_prefix
            output_path = name + ".png"
            plt.savefig(output_path, dpi=300)
            plt.close()
            # Optional: add colorbar to show depth scale
            cbar = plt.colorbar(sc, ax=ax1, pad=0.1)
            cbar.set_label('Height (Z)')

            # Highlight pit region if it exists
            if len(region_around_pit) > 0:
                ax1.scatter(region_around_pit[:, 0], region_around_pit[:, 1], region_around_pit[:, 2],
                        c='red', s=10, label='Pit Region')




        if full_debugging:

            center_point_x_pit = 10
            center_point_y_pit = 40
            

            if self._name_prefix == "_coarse":
                self._debug_roi_radius = 1
            else:
                self._debug_roi_radius = 1    

            for i, point in enumerate(points_to_visualize):
                if np.sqrt((point[0] - center_point_x_pit)**2 + (point[1] - center_point_y_pit)**2) > self._debug_roi_radius:
                    continue  # Skip points outside radius

                sphere_path = root_path + f"/point_sphere_{spheres_added}_{self._name_prefix}"
                spheres_added += 1

                sphere_prim = UsdGeom.Sphere.Define(self._stage, Sdf.Path(sphere_path)).GetPrim()
                sphere_geom = UsdGeom.Sphere(sphere_prim)
                sphere_geom.GetRadiusAttr().Set(self._visualize_points_sphere_radius)

                # Apply only transform (no material)
                xformable = UsdGeom.Xformable(sphere_prim)
                translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
                translate_op.Set(Gf.Vec3d(*map(float, point)))

            self._debug_spheres_root_path_str = root_path  # Mark as done

    # end of debugging






    def build(self, dem: np.ndarray, dem_shape: Tuple[int, int], dem_center: Tuple[float, float] = None) -> None:
        """
        Builds the clipmap with the given dem.

        Args:
            dem (np.ndarray): dem to use for the clipmap.
            dem_shape (Tuple[int, int]): shape of the dem.
            dem_center (Tuple[float, float]): center of the dem in meters. Note that this is not necessarily the center
                of the true center but rather where the [0,0], or the top left of the center block is.
        """

        self._geo_clipmap.build(dem, dem_shape, dem_center=dem_center)

    def update_DEM_buffer(self) -> None:
        """
        Updates the DEM buffer of the clipmap.
        """

        self._geo_clipmap.update_DEM_buffer()

    def update_geoclipmap(self, position: np.ndarray, mesh_position: np.ndarray) -> None:
        """
        Updates the clipmap with the given position and mesh position.

        Args:
            position (np.ndarray): position to use for the update (in meters).
            mesh_position (np.ndarray): position of the mesh (in meters).
        """
        with wp.ScopedTimer("complete update loop", active=self.profiling):
            self._geo_clipmap.update_elevation(position)
            #For debugging, from Yannic
            if full_debugging:
                # Add sphere visualization for coarse mesh
                if self._name_prefix == "_coarse":
                    self._visualize_mesh_points_as_spheres(self._geo_clipmap.points)
                if self._name_prefix == "_fine":
                    self._visualize_mesh_points_as_spheres(self._geo_clipmap.points)
            #end of debugging

            with wp.ScopedTimer("mesh update", active=self.profiling):
                self.render_mesh(
                    self._geo_clipmap.points,
                    self._geo_clipmap.indices,
                    self._geo_clipmap.uvs,
                    update_topology=self.update_topology,
                )
        self.move_mesh(mesh_position)
        self.update_topology = False

    def move_mesh(self, position: np.ndarray) -> None:
        """
        Moves the mesh to the given position.

        Args:
            position (np.ndarray): position to move the mesh to (in meters).
        """

        self._mesh_pos = position
        mesh = UsdGeom.Mesh.Get(self._stage, self._mesh_path)
        if mesh:
            set_xform_ops(mesh, Gf.Vec3d(self._mesh_pos[0], self._mesh_pos[1], self._mesh_pos[2]))

    def create_Xforms(self) -> None:
        """
        Creates the xforms for the clipmap.
        """

        if not self._stage.GetPrimAtPath(self._root_path):
            self._stage.DefinePrim(self._root_path, "Xform")
        if not self._stage.GetPrimAtPath(self._root_path + "/Terrain"):
            self._stage.DefinePrim(self._root_path + "/Terrain", "Xform")
        self._stage.DefinePrim(self._mesh_path)

    def render_mesh(
        self,
        points: np.ndarray,
        indices: np.ndarray,
        uvs: np.ndarray,
        update_topology: bool = False,
    ) -> None:
        """
        Creates or updates a mesh prim with the given points and indices.

        Args:
            points (np.ndarray): array of points to set as the mesh vertices.
            indices (np.ndarray): array of indices to set as the mesh indices.
            uvs (np.ndarray): array of uvs to set as the mesh uvs.
            colors (np.ndarray): array of colors to set as the mesh colors.
            update_topology (bool): whether to update the mesh topology.
        """

        mesh = UsdGeom.Mesh.Get(self._stage, self._mesh_path)
        self._mesh_path = self._og_mesh_path
        if not mesh:
            mesh = UsdGeom.Mesh.Define(self._stage, self._mesh_path)
            set_xform_ops(mesh, Gf.Vec3d(0.0, 0.0, 0.0), Gf.Quatd(1.0, (0.0, 0.0, 0.0)), Gf.Vec3d(1.0, 1.0, 1.0))

            # force topology update on first update
            update_topology = True

        mesh.GetPointsAttr().Set(points)

        if update_topology:
            idxs = np.array(indices).reshape(-1, 3)
            mesh.GetFaceVertexIndicesAttr().Set(idxs)
            mesh.GetFaceVertexCountsAttr().Set([3] * len(idxs))
            pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.Float2Array)
            pv.Set(uvs)
            pv.SetInterpolation("faceVarying")

        set_xform_ops(
            mesh,
            Gf.Vec3d(self._mesh_pos[0], self._mesh_pos[1], self._mesh_pos[2]),
            Gf.Quatd(self._mesh_rot[-1], (self._mesh_rot[0], self._mesh_rot[1], self._mesh_rot[2])),
            Gf.Vec3d(self._mesh_scale[0], self._mesh_scale[1], self._mesh_scale[2]),
        )

    def get_height_and_random_orientation(
        self,
        x: np.ndarray,
        y: np.ndarray,
        map_coordinates: Tuple[float, float],
        seed: int = 0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Gets the height and random orientation at the given x and y.

        Args:
            x (np.ndarray): x value.
            y (np.ndarray): y value.
            map_coordinates (Tuple[float, float]): coordinates of the map.
            seed (int): seed to use for the random orientation.

        Returns:
            Tuple[np.ndarray, np.ndarray]: height and random orientation.
        """

        return self._geo_clipmap.get_height_and_random_orientation(x, y, map_coordinates, seed=seed)
