import logging
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import UsdGeom, Usd
import omni.kit.viewport.utility
from omni.isaac.core.utils.viewports import create_viewport_for_camera
from omni.isaac.sensor.scripts.camera import Camera


#For now only really setup for front cameras

#known issue:
# cannot set the p and s attributes of the camera for a fisheye camera

#parameters for camera position and orientation
# use x front, y left, z up
#is on top of exisiting which is in front 0.15 from center, 0.04m outwards and 0.2 [m] above ground
front_x = 0.0
front_y = 0.0
front_z = 0.0

front_inward_rotation = 0 # positive is inwards for FR, is y axis
front_upward_rotation = 0 #positive is upwards for FR, is x axis


# -------- Camera Prim Paths --------
CAMERA_PRIM_PATHS = {
    "FR": "/Robots/LL/base_link/FR_cam/owl/FR_cam",
    "FL": "/Robots/LL/base_link/FL_cam/owl/FL_cam"
}

#used for ROS2 publishing
RENDER_PRODUCT_PRIM_PATHS = {
    "FR": "/Robots/LL/ROS2_Graph/CameraGraph/isaac_create_render_product_02", 
    "FL": "/Robots/LL/ROS2_Graph/CameraGraph/isaac_create_render_product_03"

}



# -------- Camera Model Class --------
class CameraModel:
    def __init__(
        self, focal_length, focus_distance, fstop,
        horizontal_aperture, vertical_aperture,
        horizontal_aperture_offset, vertical_aperture_offset,
        clipping_rangeX, clipping_rangeY,
        projectionType, NominalWidth, NominalHeight,
        OpticalCenterX, OpticalCenterY, MaxFOV,
        polyK=None, p=None, s=None
    ):
        self.focal_length = focal_length
        self.focus_distance = focus_distance
        self.fStop = fstop
        self.horizontal_aperture = horizontal_aperture
        self.vertical_aperture = vertical_aperture
        self.horizontal_aperture_offset = horizontal_aperture_offset
        self.vertical_aperture_offset = vertical_aperture_offset
        self.ClippingRangeX = clipping_rangeX
        self.ClippingRangeY = clipping_rangeY
        self.cameraProjectionType = projectionType
        self.NominalWidth = NominalWidth
        self.NominalHeight = NominalHeight
        self.OpticalCenterX = OpticalCenterX
        self.OpticalCenterY = OpticalCenterY
        self.fthetaMaxFov = MaxFOV

        self.polyK = polyK or [0.0]*6
        self.p = p or [0.0, 0.0]
        self.s = s or [0.0]*4

    def _set_attr_safe(self, prim, attr_name, value):
        """
        Set an attribute on a prim safely, checking if it exists first.
        :param prim: The prim to set the attribute on.
        :param attr_name: The name of the attribute to set.
        :param value: The value to set the attribute to.
        """
        if prim.HasAttribute(attr_name):
            prim.GetAttribute(attr_name).Set(value)
        else:
            logging.warning(f"[WARN] Attribute '{attr_name}' not found on prim {prim.GetPath()}")

    def apply_camera_model_to_prim_path(self, camera_prim_path: str):
        """
        Apply the camera model to a given camera prim path.
        :param camera_prim_path: The path to the camera prim.
        """
        prim = get_prim_at_path(camera_prim_path)
        if prim is None:
            logging.error(f"[ERROR] No prim at path {camera_prim_path}")
            return

        camera = UsdGeom.Camera(prim)
        if not camera:
            logging.error(f"[ERROR] Failed to initialize USD Camera at {camera_prim_path}")
            return

        # Standard attributes
        camera.GetFocalLengthAttr().Set(self.focal_length)
        camera.GetFocusDistanceAttr().Set(self.focus_distance)
        camera.GetFStopAttr().Set(self.fStop)
        camera.GetHorizontalApertureAttr().Set(self.horizontal_aperture)
        camera.GetVerticalApertureAttr().Set(self.vertical_aperture)
        camera.GetHorizontalApertureOffsetAttr().Set(self.horizontal_aperture_offset)
        camera.GetVerticalApertureOffsetAttr().Set(self.vertical_aperture_offset)
        camera.GetClippingRangeAttr().Set((self.ClippingRangeX, self.ClippingRangeY))

        # Fisheye attributes
        self._set_attr_safe(prim, "cameraProjectionType", self.cameraProjectionType)
        self._set_attr_safe(prim, "fthetaWidth", float(self.NominalWidth))
        self._set_attr_safe(prim, "fthetaHeight", float(self.NominalHeight))
        self._set_attr_safe(prim, "fthetaCx", float(self.OpticalCenterX))
        self._set_attr_safe(prim, "fthetaCy", float(self.OpticalCenterY))
        self._set_attr_safe(prim, "fthetaMaxFov", float(self.fthetaMaxFov))

        #these fisyeye cannot and do no need to be set if do not have distortion coefficients
        '''
        for i in range(6):
            self._set_attr_safe(prim, f"fthetaPoly{chr(65 + i)}", self.polyK[i])
        for i in range(2):
            self._set_attr_safe(prim, f"p{i}", self.p[i])
        for i in range(4):
            self._set_attr_safe(prim, f"s{i}", self.s[i])
        '''
        logging.info(f"[INFO] Applied camera model to {camera_prim_path}")

    def __str__(self):
        return (f"Focal Length: {self.focal_length}, Horizontal Aperture: {self.horizontal_aperture}, "
                f"Vertical Aperture: {self.vertical_aperture}, H. Offset: {self.horizontal_aperture_offset}, "
                f"V. Offset: {self.vertical_aperture_offset}")

    def change_camera_position_and_orientation(camera_prim_path: str, position: tuple, orientation: tuple):
        """
        Change the position and orientation of a camera.
        :param camera_prim_path: The path to the camera prim.
        :param position: A tuple (x, y, z) representing the new position.
        :param orientation: A tuple (roll, pitch, yaw) representing the new orientation.
        """
        camera_prim = get_prim_at_path(camera_prim_path)
        owl_prim = camera_prim.GetParent()
        if camera_prim is None:
            logging.error(f"[ERROR] No camera found at path {owl_prim}")
            return

        owl_prim.GetAttribute("xformOp:translate").Set(value=position)
        owl_prim.GetAttribute("xformOp:rotateXYZ").Set(value=orientation)
        logging.info(f"[INFO] Changed position and orientation for camera at path {owl_prim}")

# -------- Predefined Models --------
# p and s attributes cannot be set currently
CAMERA_MODELS = {
    # This is the camera which already existed, all others are adapted versions of this one, the way it is right now are teh fisheye parameters missing (p,s)
    "OWL": CameraModel(
    focal_length=1.36461,
    focus_distance=0.6, 
    fstop=180,
    horizontal_aperture=5.76,
    vertical_aperture=3.6,
    horizontal_aperture_offset=0.0,
    vertical_aperture_offset=0.0,
    clipping_rangeX=0.076,
    clipping_rangeY=100000.0,
    projectionType="fisheyePolynomial",
    NominalWidth=1920.0,
    NominalHeight=1200.0,
    OpticalCenterX=943.99463,
    OpticalCenterY=602.3111,
    MaxFOV=235, #unknown
    ),

    "KissCamv2": CameraModel(
        #pixsel size is 3.75 mikrometers
        focal_length=2.1,
        focus_distance=5, #unknown
        fstop=1.9,
        horizontal_aperture=4.8,
        vertical_aperture=3.6,
        horizontal_aperture_offset=0.0,
        vertical_aperture_offset=0.0,
        clipping_rangeX=0.05,
        clipping_rangeY=100000.0,
        projectionType="fisheyeEquidistant",
        NominalWidth=1280.0,
        NominalHeight=960.0,
        OpticalCenterX=640,
        OpticalCenterY=480,
        MaxFOV=135, #unknown
    ),
    "IM20016mm": CameraModel(
        #different versions exist
        focal_length=16,
        focus_distance=5, #unknown
        fstop=1.2,
        horizontal_aperture=7.2, #unknown, asked - would need NDA
        vertical_aperture=3.6, #unknown, asked
        horizontal_aperture_offset=0.0,
        vertical_aperture_offset=0.0,
        clipping_rangeX=0.05,
        clipping_rangeY=100000.0,
        projectionType="pinhole", #not yet calculated which has best fit
        NominalWidth=2048.0,
        NominalHeight=1944.0,
        OpticalCenterX=1024,
        OpticalCenterY=972,
        MaxFOV=135, #unknown
    ),
    "SCS": CameraModel(
        #pixel size us 1.67 mikrometers
        focal_length=3.8,
        focus_distance=5, #unknown
        fstop=2.8,
        horizontal_aperture=6.11888,
        vertical_aperture=4.58916,
        horizontal_aperture_offset=0.0,
        vertical_aperture_offset=0.0,
        clipping_rangeX=0.05,
        clipping_rangeY=100000.0,
        projectionType="pinhole", #should have best fit but is still blurry
        NominalWidth=3664,
        NominalHeight=2748,
        OpticalCenterX=1832,
        OpticalCenterY=1374,
        MaxFOV=87.6, 
    ),
        "4m8mm": CameraModel(
        #pixel size us 5.5 mikrometers
        focal_length=7.9,
        focus_distance=5, #unknown
        fstop=8.0,
        horizontal_aperture=11.264,
        vertical_aperture=11.264,
        horizontal_aperture_offset=0.0,
        vertical_aperture_offset=0.0,
        clipping_rangeX=0.05,
        clipping_rangeY=100000.0,
        projectionType="fisheyeEquidistant",
        NominalWidth=2048,
        NominalHeight=2048,
        OpticalCenterX=1024,
        OpticalCenterY=1024,
        MaxFOV=99.8, #datasheet says 80x80 (horizontal,vertical) but here is diagonal FOV, also when calculating it do I get 71, this can be corrected by using fisheye model
    ),
        "4m11mm": CameraModel(
        #pixel size us 5.5 mikrometers
        focal_length=11.6,
        focus_distance=5, #unknown
        fstop=9.0,
        horizontal_aperture=11.264,
        vertical_aperture=11.264,
        horizontal_aperture_offset=0.0,
        vertical_aperture_offset=0.0,
        clipping_rangeX=0.05,
        clipping_rangeY=100000.0,
        projectionType="fisheyeEquisolid",
        NominalWidth=2048,
        NominalHeight=2048,
        OpticalCenterX=1024,
        OpticalCenterY=1024,
        MaxFOV=73.9,
        ),
        "4m25mm": CameraModel(
        #pixel size us 5.5 mikrometers
        focal_length=25,
        focus_distance=5, #unknown
        fstop=6.0,
        horizontal_aperture=11.264,
        vertical_aperture=11.264,
        horizontal_aperture_offset=0.0,
        vertical_aperture_offset=0.0,
        clipping_rangeX=0.05,
        clipping_rangeY=100000.0,
        projectionType="fisheyeOrthographic",
        NominalWidth=2048,
        NominalHeight=2048,
        OpticalCenterX=1024,
        OpticalCenterY=1024,
        MaxFOV=38.9,
        ),
        "4m50mm": CameraModel(
        #pixel size us 5.5 mikrometers
        focal_length=50,
        focus_distance=5, #unknown
        fstop=4.0,
        horizontal_aperture=11.264,
        vertical_aperture=11.264,
        horizontal_aperture_offset=0.0,
        vertical_aperture_offset=0.0,
        clipping_rangeX=0.05,
        clipping_rangeY=100000.0,
        projectionType="pinhole",
        NominalWidth=2048,
        NominalHeight=2048,
        OpticalCenterX=1024,
        OpticalCenterY=1024,
        MaxFOV=17.6,
        )
 }

# predefined positions
CAMERA_POSITIONS = {
    "FR": {
        "position": (front_y, front_z , -front_x),
        "orientation": (front_upward_rotation,front_inward_rotation, 0.0)
    },
    "FL": {
        "position": (-front_y, front_z, -front_x),
        "orientation": (front_upward_rotation,-front_inward_rotation, 0.0)
    }
    # Add more camera positions here as needed
}


# -------- Utility Functions --------
def apply_camera_model_to_all(model_name: str, camera_paths: dict, render_product_paths: dict):
    """
    Apply a camera model to all cameras in the given dictionary.
    :param model_name: Name of the camera model to apply.
    :param camera_paths: Dictionary of camera names and their corresponding paths.
    :param render_product_paths: Dictionary of render product names and their corresponding paths, used for ROS2 topics.
    """
    logging.info(f"[INFO] Applying camera model '{model_name}' to all cameras")
    if model_name not in CAMERA_MODELS:
        logging.error(f"[ERROR] Unknown model name '{model_name}'")
        return
    model = CAMERA_MODELS[model_name]
    logging.info(f"[INFO] Applying camera model '{model_name}' → {model}")
    for idx, (name, cam_path) in enumerate(camera_paths.items()):
        logging.info(f"[INFO] Processing camera '{name}' at path {cam_path}")
        model.apply_camera_model_to_prim_path(cam_path) # Applies UsdGeom.Camera and fisheye attributes to the camera prim

        if name in render_product_paths:
            rp_path = render_product_paths[name]
            render_product_prim = get_prim_at_path(rp_path)
            if render_product_prim and render_product_prim.IsValid():
                logging.info(f"[INFO] Setting render product dimensions for '{name}' at path {rp_path}")
                _set_prim_attribute_if_exists(render_product_prim, "inputs:width", int(model.NominalWidth))
                _set_prim_attribute_if_exists(render_product_prim, "inputs:height", int(model.NominalHeight))
            else:
                logging.warning(f"[WARN] Render product prim not found or invalid at path {rp_path} for camera '{name}'")
        else:
            logging.warning(f"[WARN] No render product path specified for camera '{name}'. Skipping render dimension settings.")          
        # This would work to change the viewport resolution, however it crashes the simulation        
        '''
        try:
            from omni.kit.viewport.window import get_viewport_window_instances
            width = int(model.NominalWidth)
            height = int(model.NominalHeight)
            found = False
            for window in get_viewport_window_instances():
                logging.debug(f"[DEBUG] Checking viewport '{window.title}'")
                window.viewport_api.set_texture_resolution((width, height))
                logging.info(f"[INFO] Set resolution of viewport '{window.title}' to {width}x{height}")
                found = True
                break
            if not found:
                logging.info(f"[INFO] No existing viewport named found. Skipping viewport resolution change.")
        except Exception as e:
            logging.error(f"[ERROR] Failed to set resolution for viewport: {e}")
        '''
        print_camera_parameters(cam_path)  # Print camera parameters after applying the model



def _set_prim_attribute_if_exists(prim, attr_name: str, value):
    """Helper function to set an attribute on a prim if it exists."""
    if prim.HasAttribute(attr_name):
        try:
            prim.GetAttribute(attr_name).Set(value)
            logging.debug(f"[DEBUG] Set attribute '{attr_name}' to '{value}' on prim {prim.GetPath()}")
        except Exception as e:
            logging.error(f"[ERROR] Failed to set attribute '{attr_name}' on prim {prim.GetPath()}: {e}")
    else:
        logging.warning(f"[WARN] Attribute '{attr_name}' not found on prim {prim.GetPath()}")


def change_all_camera_positions_and_orientations(camera_paths: dict, positions: dict):
    """
    Change the position and orientation of all cameras in the given dictionary.
    :param camera_paths: Dictionary of camera names and their corresponding paths.
    :param positions: Dictionary of camera names and their corresponding positions and orientations.
    """
    logging.info(f"[INFO] Changing camera positions and orientations")
    for name, cam_path in camera_paths.items():
        if name in positions:
            position = positions[name]["position"]
            orientation = positions[name]["orientation"]
            logging.info(f"[INFO] Changing position and orientation for camera '{name}' at path {cam_path}")
            CameraModel.change_camera_position_and_orientation(cam_path, position, orientation)
        else:
            logging.warning(f"[WARN] No position/orientation specified for camera '{name}'. Skipping.")


def print_camera_parameters(camera_prim_path: str):
    """
    Print the camera parameters for a given camera prim path.
    :param camera_prim_path: The path to the camera prim.
    """
    camera_prim = get_prim_at_path(camera_prim_path)
    if camera_prim is None:
        logging.error(f"[ERROR] No camera found at path {camera_prim_path}")
        return

    camera = UsdGeom.Camera(camera_prim)
    logging.info(f"[INFO] Camera parameters for {camera_prim_path}:")
    logging.info(f"  Focal Length: {camera.GetFocalLengthAttr().Get()}")
    logging.info(f"  Horizontal Aperture: {camera.GetHorizontalApertureAttr().Get()}")
    logging.info(f"  Vertical Aperture: {camera.GetVerticalApertureAttr().Get()}")
    logging.info(f"  H. Aperture Offset: {camera.GetHorizontalApertureOffsetAttr().Get()}")
    logging.info(f"  V. Aperture Offset: {camera.GetVerticalApertureOffsetAttr().Get()}")

