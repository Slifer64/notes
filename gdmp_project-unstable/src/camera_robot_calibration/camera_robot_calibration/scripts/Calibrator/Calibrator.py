#   Aristotle Univercisty of Thessaloniki
#   Robotics and Automation Lab
#
#   Author: Savvas Sampaziotis
#   e-mail: savvas.sampaziotis@gmail.com
#
import numpy as np
import cv2
from dt_apriltags import apriltags as ap
from apriltag_utils import apriltag_utils as ap_utils
import rospy


#   Calibrator Class:
#
#       Contains the AprilTag Detector, and handles the detection, cross referencing and estimation of the
#       Transform R|t between the camera and the robot world frame
#
class Calibrator:

    def __init__(self, TAG_SIZE, camera, CALIBRATION_MODE="corners"):
        self.TAG_SIZE = TAG_SIZE
        self.camera = camera
        self.CALIBRATION_MODE = CALIBRATION_MODE
        self.detector = ap.Detector()
        self.cv_image_with_tags = "No Image passed to April Tag Detector"
        self.apriltag_results = None

        self.tag_list = []
        for i in range(0,20):
            if rospy.has_param("Calibration/tags/tag_" + str(i)):
                self.tag_list.append(i)

        print("***************************************")
        print("[CALIBRATOR]: Calibration Mode = " + self.CALIBRATION_MODE)
        print("***************************************")


    #
    #   Detects the april tags in input picture.
    #   OUTPUT:
    #       + a "results" list
    #       + a helpful cv_image with markers on tag corners
    #
    def detect_april_tags(self):
        cv_image = self.camera.get_color_frame()
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        return results, cv_image



    #
    #   Performs Calibration of camera position in respect to the robot.
    #
    # The method first detects april tags in input cv_image, and extracts the 3D coordinates of the april tag corners
    # in repsect to eh camera frame {C}.
    #
    # Then, using the 3D coordinates of the same corners defined in repsct to the ROBOT base frame {O}, it estimates
    # the homogenous transform |R,t| between the two sets of coordinates
    #
    # OUTPUT:
    #   + the rotation Matrix R, the translation vector t of the camera frame in respect to the robot frame
    #   + the MSE of the estimation
    #
    def calibrate(self):

        # WARNING: detector has bugs and messes things up if twice in same cycle.
        # This if-segment avoids that specific scenario: In case Calibrator.run_viewer() is called,
        #   the apriltag_results are updated internally, and there is no need to onvoke the detector again.
        if self.apriltag_results is None:
            # april detector has not been called externally via th eviewer
            self.apriltag_results = self.detect_april_tags()
        else:
            pass # do nothing, self.apriltag_results is already set.

        # filter out tags that are not in the config file
        filtered_apriltag_results = [r for r in self.apriltag_results if r.tag_id in self.tag_list]

        print("[CALIBRATOR]:    Tags in config: " + str(self.tag_list))
        print("[CALIBRATOR]:    Tags Detected:  " + str([r.tag_id for r in self.apriltag_results]))
        print("[CALIBRATOR]:    Tags Used in Estimation:  " + str([r.tag_id for r in filtered_apriltag_results]))

        if not filtered_apriltag_results: # list is empty
            print("[CALIBRATOR]: No april tags detected in image, that are also in config file:" + str(self.tag_list))
            return None, None, None

        # get the points in form of lists of 3x1 coords
        if self.CALIBRATION_MODE=="corners":
            camera_points_list = self._get_apriltag_corners_depth(filtered_apriltag_results)
            robot_points_list = self._get_robot_corners(filtered_apriltag_results)
        elif self.CALIBRATION_MODE == "centers":
            camera_points_list = self._get_apriltag_centers_depth(filtered_apriltag_results)
            robot_points_list = self._get_robot_centers(filtered_apriltag_results)
        else:
            raise AssertionError("[CALIBRATOR]: uknown calibration mode given: "+str(self.CALIBRATION_MODE))

        # self._print_point_list(robot_points_list, camera_points_list)

        # Estimate
        R, t, MSE = self.estimate_transform_(camera_points_list, robot_points_list)
        if MSE > 0.001:
            print( "[CALIBRATOR]: WARNING: Estimation Error too high: MSE = " + str(MSE))
        self._print_estimation_results( R, t, MSE)

        return R, t, MSE


    #
    #   Calculates the [R|t] transform between a list of 3D points "A" and "B"
    #
    #             p_b = R*p_a + t
    #
    #   INPUT:
    #       * Point List "A" contains the april tag positions in respect to the CAMERA frame
    #       * Point List "B" contains the april tag positions in respect to the ROBOT frame
    #
    #   Point Lists should be a list of 3x1 numpy arrays [np.array([]).T,[]]
    #
    #   OUTPUT:
    #       * the rotation matrix R
    #       * translation vector t
    #
    def estimate_transform_(self, camera_points_list, robot_points_list):
        # transform the list of 3x1 vec to a compact 3xN array
        A = np.hstack(camera_points_list)
        B = np.hstack(robot_points_list)

        # find the centroids 3x1
        A_centroid = np.mean(A, axis=1)
        B_centroid = np.mean(B, axis=1)

        # Estimate Rotation Matrix using SVD
        #
        A_n, B_n = (A.T - A_centroid.T).T, (B.T - B_centroid.T).T
        # normalise points: P' = P - centroid
        H = np.dot(A_n, B_n.T)
        u, s, v = np.linalg.svd(H)

        R = np.dot(v, u.T)
        if np.linalg.det(R) < 0:  # special case: Reflection
            print( "[CALIBRATOR]: special case detected: det(R) = " + str(np.linalg.det(R)) + " < 0")
            print( "[CALIBRATOR]: Rotation Matrix is recalculated")
            v[:, 2] = -v[:, 2]
            R = np.dot(v, u.T)

        # Estimate Translation vector
        #
        t = B_centroid - np.dot(R, A_centroid)

        A = np.hstack(camera_points_list)
        B = np.hstack(robot_points_list)

        # Estimation Error:
        e_ = np.dot(R, A).T + t.T - B.T
        MSE = sum(np.linalg.norm(e_, axis=1) ** 2) / len(camera_points_list)
        return R, t, MSE





    # Loads the april-tag corners (in respect to the Robot Frame {O}) from the config file.
    #
    #   TODO: this function uses rospy functionality. It d be better if ROS utilities remain separate, and the
    #
    # The method load the ABCD coordinates of each "Calibration/tags/tag_<>ID" entry in the config file.
    #
    # Only tags that have been already detected are loaded.
    #
    # OUTPUT: a list of 3x1 vectors (np arrays)
    #
    def _get_robot_corners(self, apriltag_results):
        # list must contain 3x1 vectors. Each tag sorted with corners order "A,B,C,D"
        robot_point_list = []
        for r in apriltag_results:
            tag_id = "tag_" + str(r.tag_id)
            if rospy.has_param("Calibration/tags/" + tag_id):
                for corner_letter in "ABCD":
                    temp_ = np.array(rospy.get_param("Calibration/tags/" + tag_id + "/" + corner_letter)).reshape(3, 1)
                    # temp_ = temp_/1000 # scale from mm to m
                    robot_point_list.append(temp_)
            else:
                raise RuntimeError("[CALIBRATOR]: Tag [" + tag_id + "] detected that does not exist on the config file")
        return robot_point_list

    def _get_apriltag_corners_depth(self, apriltag_results):
        camera_point_list = []
        for r in apriltag_results:
            for c in r.corners:
                x, y = c[0], c[1]
                X,Y,Z = self.camera.get_reverse_projection(x,y)
                camera_point_list.append(np.array([X,Y,Z]).reshape(3,1))
        return  camera_point_list


    # Loads the april-tag centers (in respect to the Robot Frame {O}) from the config file.
    #
    #   TODO: this function uses rospy functionality. It d be better if ROS utilities remain separate, and the
    #
    # The method load the center coordinates of each "Calibration/tags/tag_<>ID" entry in the config file.
    #
    # Only tags that have already been detected are loaded.
    #
    # OUTPUT: a list of 3x1 vectors (np arrays)
    #
    def _get_robot_centers(self, apriltag_results):
        # list must contain 3x1 vectors. Each tag
        robot_point_list = []
        for r in apriltag_results:
            tag_id = "tag_" + str(r.tag_id)
            if rospy.has_param("Calibration/tags/" + tag_id):
                temp_ = np.array(rospy.get_param("Calibration/tags/" + tag_id + "/center")).reshape(3, 1)
                # temp_ = temp_/1000
                robot_point_list.append(temp_)
            else:
                raise RuntimeError("[CALIBRATOR]: Tag [" + tag_id + "] detected that does not exist on the config file")
        return robot_point_list

    def _get_apriltag_centers_depth(self, apriltag_results):
        camera_point_list = []
        for r in apriltag_results:
            x, y = r.center[0], r.center[1]
            X,Y,Z = self.camera.get_reverse_projection(x,y)
            camera_point_list.append(np.array([X, Y, Z]).reshape(3, 1))
        return camera_point_list





    #
    #   Display Utilities
    #
    #

    # diplays the image used in calibration, so as to validate the results
    #
    #
    def show_image_with_apriltags(self):
        cv2.imshow("", self.cv_image_with_tags)
        print( "[CALIBRATOR]: Transform estimated: Press any key to continue...")
        cv2.waitKey(25000)


    def run_viewer(self):
        results, cv_image = self.detect_april_tags()
        for r in results:
            cv_image = ap_utils.draw_apriltag(r, cv_image)
        self.cv_image_with_tags = cv_image
        self.apriltag_results = results
        return cv_image

    def _print_point_list(self, robot_list, camera_list):
        print("Points Detected:")
        print("_________________________________________")
        print("          Robot      |       Camera      ")
        for i in range(0,len(robot_list)):
            print("P_" + str(i) + "     " + str(robot_list[i]) + "     " + str(camera_list[i]))
        print("_________________________________________")


    def _print_estimation_results(self, R, t, err):
        print("[CALIBRATOR]:")
        print("R = ")
        print(R)
        print("t = ")
        print(t)
        print("error = ")
        print(err)