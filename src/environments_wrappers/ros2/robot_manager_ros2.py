__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple
import scripts.GNC.change_cameras as cam_utils

from pxr import Gf

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from src.robots.robot import RobotManager


class ROS_RobotManager(Node):
    """
    ROS2 node that manages the robots.
    """

    def __init__(self, RM_conf: dict) -> None:
        super().__init__("Robot_spawn_manager_node")
        self.RM = RobotManager(RM_conf)


        # --- Add Camera ROS 2 Subscribers ---
        # Topic to set a specific camera MODEL on all cameras
        self.create_subscription(
            String,
            '/lunar_leaper/camera/set_model', # Expects String message data = model name (e.g., "3DPlus8mm")
            self._set_camera_model_cb,
            1 # Queue size
        )

        # Topic to apply ALL predefined camera POSES from CAMERA_POSITIONS
        # Can use String (data ignored) or Empty for simple trigger
        self.create_subscription(
            String, # Use String for consistency, data can be anything
            '/lunar_leaper/camera/apply_predefined_poses', # Trigger to apply all poses from CAMERA_POSITIONS
            self._apply_predefined_poses_cb,
            1 # Queue size
        )
        # --- End Camera ROS 2 Subscribers ---

        self.create_subscription(PoseStamped, "/OmniLRS/Robots/Spawn", self.spawn_robot, 1)
        self.create_subscription(PoseStamped, "/OmniLRS/Robots/Teleport", self.teleport_robot, 1)
        self.create_subscription(String, "/OmniLRS/Robots/Reset", self.reset_robot, 1)
        self.create_subscription(Empty, "/OmniLRS/Robots/ResetAll", self.reset_robots, 1)

        self.domain_id = 0
        self.modifications: List[Tuple[callable, dict]] = []

    # --- New Camera ROS 2 Callback Methods ---
    def _set_camera_model_cb(self, msg: String) -> None:
        """
        Callback for the /lunar_leaper/camera/set_model topic.
        Applies the camera model specified in the message data to all cameras.
        Message data should be the model name (e.g., "3DPlus8mm").
        Calls lunar_leaper_camera_utils.apply_camera_model_to_all.
        """
        model_name = msg.data.strip()

        # Call the utility function from the imported module
        cam_utils.apply_camera_model_to_all(
            model_name,
            cam_utils.CAMERA_PRIM_PATHS,
            cam_utils.RENDER_PRODUCT_PRIM_PATHS
        )


    def _apply_predefined_poses_cb(self, msg: String) -> None: # Changed msg type to String for consistency
        """
        Callback for the /lunar_leaper/camera/apply_predefined_poses topic.
        Triggers the application of ALL predefined poses from CAMERA_POSITIONS
        to their respective cameras. Message data is ignored.
        Calls lunar_leaper_camera_utils.change_all_camera_positions_and_orientations.
        """
 
        # Call the utility function from the imported module to apply all poses
        # Pass the full CAMERA_POSITIONS dictionary to change_all_camera_positions_and_orientations
        cam_utils.change_all_camera_positions_and_orientations(
            cam_utils.CAMERA_PRIM_PATHS,
            cam_utils.CAMERA_POSITIONS # Pass the dictionary containing poses for FR and FL
        )



    # --- End New Camera ROS 2 Callback Methods ---


    def reset(self) -> None:
        """
        Resets the robots to their initial state.
        """

        self.clear_modifications()
        self.reset_robots(Empty())

    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab.
        """

        self.modifications: List[Tuple[callable, dict]] = []

    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab.
        """

        for mod in self.modifications:
            mod[0](**mod[1])
        self.clear_modifications()

    def spawn_robot(self, data: PoseStamped) -> None:
        """
        Spawns a robot.

        Args:
            data (String): Name and path of the robot to spawn.
                           Must be in the format: robot_name:usd_path
        """

        assert len(data.header.frame_id.split(":")) == 2, "The data should be in the format: robot_name:usd_path"
        robot_name, usd_path = data.header.frame_id.split(":")
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z]
        self.modifications.append(
            [
                self.RM.add_robot,
                {"usd_path": usd_path, "robot_name": robot_name, "p": p, "q": q, "domain_id": self.domain_id},
            ]
        )

    def teleport_robot(self, data: PoseStamped) -> None:
        """
        Teleports a robot.

        Args:
            data (Pose): Pose in ROS2 Pose format.
        """

        robot_name = data.header.frame_id
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.modifications.append([self.RM.teleport_robot, {"robot_name": robot_name, "position": p, "orientation": q}])

    def reset_robot(self, data: String) -> None:
        """
        Resets a robot.

        Args:
            data (String): Name of the robot to reset.
        """

        robot_name = data.data
        self.modifications.append([self.RM.reset_robot, {"robot_name": robot_name}])

    def reset_robots(self, data: Empty) -> None:
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument.
        """

        self.modifications.append([self.RM.reset_robots, {}])

    def cleanRobots(self) -> None:
        """
        Cleans the robots."""

        self.destroy_node()
