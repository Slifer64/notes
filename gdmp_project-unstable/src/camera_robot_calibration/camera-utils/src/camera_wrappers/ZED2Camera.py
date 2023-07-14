import pyzed.sl as sl
import numpy as np
from .BaseCamera import BaseCamera





class ZED2Camera(BaseCamera):

    def __init__(self, resolution=720, pcl_enabled=False):
        self.init_params = sl.InitParameters()

        if resolution==720:
            BaseCamera.__init__(self, "ZED2", 1280, 720, 1280, 720, True)
            self.init_params.camera_resolution = sl.RESOLUTION.HD720
        elif resolution==1080:
            BaseCamera.__init__(self, "ZED2", 1920, 1080, 1280, 720, True)
            self.init_params.camera_resolution = sl.RESOLUTION.HD1080
            self.init_params.camera_fps = 15
        else:
            raise AssertionError("[ZED2CAMERA]: invalid input resolution: "+ str(resolution))

        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        self.init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        self.init_params.depth_mode = sl.DEPTH_MODE.QUALITY

        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL  # Use STANDARD sensing mode
        # Setting the depth confidence parameters
        self.runtime_parameters.confidence_threshold = 100
        self.runtime_parameters.textureness_confidence_threshold = 100

        self.pcl_enabled = pcl_enabled



        # PRIVATE MEMBERS
        self.image_ = sl.Mat()
        self.depth_ = sl.Mat()
        self.deth_for_view_ = sl.Mat()
        self.pcl_ = sl.Mat()
        # self.point_cloud_ = sl.Mat() # TODO


        # # First radial distortion coefficient
        # k1 = calibration_params.left_cam.disto[0]
        # # Translation between left and right eye on z-axis
        # tz = calibration_params.T.z
        # # Horizontal field of view of the left eye in degrees
        # h_fov = calibration_params.left_cam.h_fov
    def start(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
           print("[ZED2CAMERA][ERROR]: Camera couldn't start: Zed2 error code: " + str(err))
        print("[ZED2CAMERA]: Camera has Started.")
        calibration_params = self.zed.get_camera_information().calibration_parameters
        print("|    Resolution: ", self.zed.get_camera_information().camera_resolution.width, " , ", self.zed.get_camera_information().camera_resolution.height)
        print("|    Field Of View (V): ", calibration_params.left_cam.v_fov)
        print("|    Field Of View (H): ", calibration_params.left_cam.h_fov)

    def stop(self):
        self.zed.close()
        print("ZED2CAMERA: Camera has Stopped")


    def run(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            self.zed.retrieve_image(self.image_, sl.VIEW.LEFT)
            # Retrieve depth map. Depth is aligned on the left image
            self.zed.retrieve_measure(self.depth_, sl.MEASURE.DEPTH) # already in meters

            # if self.pcl_enabled:
            #     self.zed.retrieve_measure(self.pcl_, sl.MEASURE.XYZRGBA)

            temp = self.image_.get_data()
            temp = np.array(temp[:,:,:3])
            self.cv_rgb_image = temp
            self.cv_depth_image = self.depth_.get_data()
            self.zed.retrieve_image(self.deth_for_view_, sl.VIEW.DEPTH)

        else:
            print("[ZED2CAMERA][WARNING]: Dropped Frame")


    def get_pointcloud(self):
        return self.zed.retrieve_measure(self.pcl_, sl.MEASURE.XYZRGBA)


    def get_depth_frame_for_view(self):
        return self.deth_for_view_.get_data()




    def get_intrisincs(self):
        calibration_params = self.zed.get_camera_information().calibration_parameters
        # Depth is already alligned to te LEFT camera
        fx = calibration_params.left_cam.fx
        fy = calibration_params.left_cam.fy
        cx = calibration_params.left_cam.cx
        cy = calibration_params.left_cam.cy
        # Lensdistortion: k1, k2, k3, p1, p2. (?)
        camera_params = [fx, fy, cx, cy]
        return camera_params


    def get_reverse_projection(self, x, y):
        camera_params = self.get_intrisincs()
        fx, fy, cx, cy = [c for c in camera_params]
        u, v = np.int(np.ceil(x)), np.int(np.ceil(y))
        if (u >= 0 and u < self.cv_depth_image.shape[1]) and (v >= 0 and v < self.cv_depth_image.shape[0]):
            err, Z = self.depth_.get_value(u,v) # already in meters
            X = (x - cx) * Z / fx
            Y = (y - cy) * Z / fy
        else:
            X = None
            Y = None
            Z = None
            # print('ZED2CAMERA: REV-PROJ: OUT OF BOUNDS')
        return X, Y, Z