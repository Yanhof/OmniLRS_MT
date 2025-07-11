__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from scipy.spatial.transform import Rotation as SSTR
from typing import List, Tuple
import numpy as np
import math
import os

from omni.isaac.core.utils.stage import add_reference_to_stage
import omni

from pxr import UsdLux, Gf, Usd, Sdf

from src.terrain_management.large_scale_terrain_manager import LargeScaleTerrainManager
from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops
from src.configurations.stellar_engine_confs import StellarEngineConf, SunConf
from src.configurations.environments import LargeScaleTerrainConf
from src.stellar.stellar_engine import StellarEngine
from src.environments.base_env import BaseEnv
from src.robots.robot import RobotManager
from assets import get_assets_path


class LargeScaleController(BaseEnv):
    """
    This class is used to control the environment's interactive elements.
    """

    def __init__(
        self,
        large_scale_terrain: LargeScaleTerrainConf = None,
        stellar_engine_settings: StellarEngineConf = None,
        sun_settings: SunConf = None,
        is_simulation_alive: callable = lambda: True,
        **kwargs,
    ) -> None:
        """
        Initializes the env controller. This class is used to control the env interactive elements.
        Including:
            - Sun position, intensity, radius, color.

        Args:
            env_settings (LargeScaleTerrainConf): The settings of the lab.
            stellar_engine_settings (StellarEngineConf): The settings of the stellar engine.
            sun_settings (SunConf): The settings of the sun.
            is_simulation_alive (callable): function to check if the simulation is alive.
            **kwargs: Arbitrary keyword arguments.
        """

        super().__init__(**kwargs)
        self.stage_settings = large_scale_terrain
        self.sun_settings = sun_settings
        self.is_simulation_alive = is_simulation_alive
        if stellar_engine_settings is not None:
            self.SE = StellarEngine(stellar_engine_settings)
            self.enable_stellar_engine = True
        else:
            self.enable_stellar_engine = False
        self.dem = None
        self.mask = None
        self.scene_name = "/LargeScaleLunar"

    def build_scene(self) -> None:
        """
        Builds the scene.
        """

        # Creates an empty xform with the name lunaryard
        large_scale = self.stage.DefinePrim(self.scene_name, "Xform")

        # Creates the sun
        self._sun_prim = self.stage.DefinePrim(os.path.join(self.scene_name, "Sun"), "Xform")
        self._sun_lux: UsdLux.DistantLight = UsdLux.DistantLight.Define(
            self.stage, os.path.join(self.scene_name, "Sun", "sun")
        )
        self._sun_lux.CreateIntensityAttr(self.sun_settings.intensity)
        self._sun_lux.CreateAngleAttr(self.sun_settings.angle)
        self._sun_lux.CreateDiffuseAttr(self.sun_settings.diffuse_multiplier)
        self._sun_lux.CreateSpecularAttr(self.sun_settings.specular_multiplier)
        self._sun_lux.CreateColorAttr(
            Gf.Vec3f(self.sun_settings.color[0], self.sun_settings.color[1], self.sun_settings.color[2])
        )
        self._sun_lux.CreateColorTemperatureAttr(self.sun_settings.temperature)
        enable_temp_attr = self._sun_lux.GetPrim().CreateAttribute(
            "inputs:enableColorTemperature", Sdf.ValueTypeNames.Bool
        )
        enable_temp_attr.Set(True)
        x, y, z, w = SSTR.from_euler(
            "xyz", [0, self.sun_settings.elevation, self.sun_settings.azimuth - 90], degrees=True
        ).as_quat()
#        self._sun_lux.Create
        set_xform_ops(
            self._sun_lux.GetPrim(), Gf.Vec3d(0, 0, 0), Gf.Quatd(0.5, Gf.Vec3d(0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1)
        )
        set_xform_ops(self._sun_prim.GetPrim(), Gf.Vec3d(0, 0, 0), Gf.Quatd(w, Gf.Vec3d(x, y, z)), Gf.Vec3d(1, 1, 1))

        # Creates the earth
        self._earth_prim = self.stage.DefinePrim(os.path.join(self.scene_name, "Earth"), "Xform")
        self._earth_prim.GetReferences().AddReference(self.stage_settings.earth_usd_path)
        dist = self.stage_settings.earth_distance * self.stage_settings.earth_scale
        px = math.cos(math.radians(self.stage_settings.earth_azimuth)) * dist
        py = math.sin(math.radians(self.stage_settings.earth_azimuth)) * dist
        pz = math.sin(math.radians(self.stage_settings.earth_elevation)) * dist
        set_xform_ops(self._earth_prim, Gf.Vec3d(px, py, pz), Gf.Quatd(0, 0, 0, 1))
    
    def _place_static_asset(
        self,
        prim_path: str,
        usd_path: str,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    ) -> Usd.Prim:
        """
        Creates an Xform prim and adds a reference to a static USD asset.

        Args:
            prim_path (str): The path for the new prim (e.g., "/LargeScaleLunar/Lander").
            usd_path (str): The path to the asset USD file to reference.
            position (Tuple[float, float, float]): The (x, y, z) position to place the asset.
            orientation (Tuple[float, float, float, float], optional): The (w, x,y, z) orientation. Defaults to identity.

        Returns:
            Usd.Prim: The created Xform prim.
        """
        # Define an Xform prim at the specified path.
        xform_prim = self.stage.DefinePrim(prim_path, "Xform")

        # Add a reference to the lander's USD file.
        xform_prim.GetReferences().AddReference(assetPath=usd_path)

        # Set the transform.
        w, x, y, z = orientation
        set_xform_ops(xform_prim, translate=Gf.Vec3d(position), orient=Gf.Quatd(w, Gf.Vec3d(x, y, z)))
        return xform_prim
    
    def instantiate_scene(self) -> None:
        """
        Instantiates the scene. Applies any operations that need to be done after the scene is built and
        the renderer has been stepped.
        """

        pass

    def reset(self) -> None:
        """
        Resets the environment. Implement the logic to reset the environment.
        """

        pass

    def update(self) -> None:
        """
        Updates the environment.
        """

        # print position of prim to track
        p, _ = self.pose_tracker()
        self.LSTM.update_visual_mesh((p[0], p[1]))

    def monitor_thread_is_alive(self) -> None:
        return self.LSTM.map_manager.hr_dem_gen.monitor_thread.thread.is_alive()

    def get_wait_for_threads(self) -> None:
        return [self.LSTM.map_manager.hr_dem_gen.monitor_thread.thread.join]

    def load(self) -> None:
        """
        Loads the lab interactive elements in the stage.
        Creates the instancer for the rocks, and generates the terrain.
        """

        self.build_scene()
        # Instantiates the terrain manager
        self.LSTM = LargeScaleTerrainManager(self.stage_settings, is_simulation_alive=self.is_simulation_alive)
        self.LSTM.build()

        # --- Place static assets that depend on the terrain ---
        # TODO: These values should ideally come from the configuration (self.stage_settings)
        lander_usd_path = "/workspace/omnilrs/assets/USD_Assets/Lander/blueMoonLander.usd"  
        lander_prim_path = os.path.join(self.scene_name, "Lander")
        lander_position = (0, 0, -1500)
        lander_orientation = (0.70711, 0.70711, 0.0, 0.0)   #upright orientation


        self._place_static_asset(
            prim_path=lander_prim_path, usd_path=lander_usd_path, position=lander_position, orientation=lander_orientation
        )
        # ----------------------------------------------------



        # Sets the sun using the stellar engine if enabled
        if self.enable_stellar_engine:
            self.SE.set_lat_lon(*self.LSTM.get_lat_lon())

    def add_robot_manager(self, robotManager: RobotManager) -> None:
        """
        Adds the robot manager to the environment.

        Args:
            robotManager (RobotManager): The robot manager to be added.
        """

        self.robotManager = robotManager
        self.pose_tracker = list(self.robotManager.robots.values())[0].get_pose

    # ==============================================================================
    # Stellar engine control
    # ==============================================================================

    def set_coordinates(self, latlong: Tuple[float, float] = (0.0, 0.0)) -> None:
        """
        Sets the coordinates of the lab.

        Args:
            latlong (Tuple[float,float]): The latitude and longitude of the scene on the moon.
        """

        if self.enable_stellar_engine:
            self.SE.set_lat_lon(latlong[1], latlong[0])

    def set_time(self, time: float = 0.0) -> None:
        """
        Sets the time of the stellar engine.

        Args:
            time (float): The time in seconds.
        """

        if self.enable_stellar_engine:
            self.SE.set_time(time)

    def set_time_scale(self, scale: float = 1.0) -> None:
        """
        Sets the time scale of the stellar engine.

        Args:
            scale (float): The time scale.
        """

        if self.enable_stellar_engine:
            self.SE.set_time_scale(scale)

    def update_stellar_engine(self, dt: float = 0.0) -> None:
        """
        Updates the sun and earth pose.

        Args:
            dt (float): The time step.
        """

        if self.enable_stellar_engine:
            update = self.SE.update(dt)
            if update:
                earth_pos = self.SE.get_local_position("earth")
                alt, az, _ = self.SE.get_alt_az("sun")
                quat = self.SE.convert_alt_az_to_quat(alt, az)

                self.set_sun_pose((0, 0, 0), quat)
                self.set_earth_pose(earth_pos, (0, 0, 0, 1))

    # ==============================================================================
    # Earth control
    # ==============================================================================

    def set_earth_pose(
        self,
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    ) -> None:
        """
        Sets the pose of the earth.

        Args:
            position (Tuple[float,float,float]): The position of the earth. In meters. (x,y,z)
            orientation (Tuple[float,float,float,float]): The orientation of the earth. (w,x,y,z)
        """

        w, x, y, z = (orientation[0], orientation[1], orientation[2], orientation[3])
        px, py, pz = (
            position[0] * self.stage_settings.earth_scale,
            position[1] * self.stage_settings.earth_scale,
            position[2] * self.stage_settings.earth_scale,
        )
        set_xform_ops(self._earth_prim, translate=Gf.Vec3d(px, py, pz), orient=Gf.Quatd(w, Gf.Vec3d(x, y, z)))

    # ==============================================================================
    # Sun control
    # ==============================================================================

    def set_sun_pose(
        self,
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    ) -> None:
        """
        Sets the pose of the sun.

        Args:
            position (Tuple[float,float,float]): The position of the sun. In meters. (x,y,z)
            orientation (Tuple[float,float,float,float]): The orientation of the sun. (w,x,y,z)
        """

        w, x, y, z = (orientation[0], orientation[1], orientation[2], orientation[3])
        set_xform_ops(self._sun_prim, orient=Gf.Quatd(w, Gf.Vec3d(x, y, z)))

    def set_sun_intensity(self, intensity: float) -> None:
        """
        Sets the intensity of the sun.

        Args:
            intensity (float): The intensity of the projector (arbitrary unit).
        """

        self._sun_lux.GetIntensityAttr().Set(intensity)

    def set_sun_color(self, color: List[float]) -> None:
        """
        Sets the color of the projector.

        Args:
            color (List[float]): The color of the projector (RGB).
        """

        color = Gf.Vec3d(color[0], color[1], color[2])
        self._sun_lux.GetColorAttr().Set(color)

    def set_sun_color_temperature(self, temperature: float = 6500.0) -> None:
        """
        Sets the color temperature of the projector.

        Args:
            temperature (float): The color temperature of the projector in Kelvin.
        """

        self._sun_lux.GetColorTemperatureAttr().Set(temperature)

    def set_sun_angle(self, angle: float = 0.53) -> None:
        """
        Sets the angle of the sun. Larger values make the sun larger, and soften the shadows.

        Args:
            angle (float): The angle of the projector.
        """

        self._sun_lux.GetAngleAttr().Set(angle)

    # ==============================================================================
    # Terrain info
    # ==============================================================================

    def get_height_and_normal(
        self, position: Tuple[float, float, float]
    ) -> Tuple[float, Tuple[float, float, float, float]]:
        """
        Gets the height and normal of the terrain at a given position.

        Args:
            position (Tuple[float,float,float]): The position in meters.

        Returns:
            Tuple[float, Tuple[float,float,float,float]]: The height and normal of the terrain at the given position.
                                                          (height, (x,y,z,w))
        """

        normal_vector = self.LSTM.get_normal_local(position)
        heading_vector = np.array([1.0, 0.0, 0.0])
        heading_vector = heading_vector / np.linalg.norm(heading_vector)
        heading_vector = np.cross(normal_vector, heading_vector)
        heading_vector = heading_vector / np.linalg.norm(heading_vector)
        heading_vector_2 = np.cross(normal_vector, heading_vector)
        RNorm = np.array([heading_vector, heading_vector_2, normal_vector]).T
        RM = SSTR.from_matrix(RNorm)

        return (self.LSTM.get_height_local(position), RM.as_quat())

    def deform_derrain(self) -> None:
        """
        Deforms the terrain.
        Args:
            world_poses (np.ndarray): The world poses of the contact points.
            contact_forces (np.ndarray): The contact forces in local frame reported by rigidprimview.
        """
        pass

    def apply_terramechanics(self) -> None:
        pass
