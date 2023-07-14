#!/usr/bin/env python

#   Aristotle Univercisty of Thessaloniki
#   Robotics and Automation Lab
#
#   Author: Savvas Sampaziotis
#   e-mail: savvas.sampaziotis@gmail.com
#
import numpy as np
import rospy, rospkg

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField


from Calibrator import Calibrator

import apriltag_utils.apriltag_utils as ap_utils
from dt_apriltags import apriltags as ap
import cv2

from camera_wrappers.RealSenseCamera import RealSenseCamera
# from camera_wrappers.ZED2Camera import ZED2Camera



# #
# def toROS(R,t, base_frame, tag):
#     static_transformStamped = geometry_msgs.msg.TransformStamped()
#     static_transformStamped.header.stamp = rospy.Time.now()
#     static_transformStamped.header.frame_id = base_frame
#     static_transformStamped.child_frame_id = tag

#     static_transformStamped.transform.translation.x = t[0]
#     static_transformStamped.transform.translation.y = t[1]
#     static_transformStamped.transform.translation.z = t[2]

#     R_temp = np.hstack([np.vstack([R, np.zeros(3)]), np.array([0, 0, 0, 1]).reshape(4, 1)])
#     quat = tf.transformations.quaternion_from_matrix(R_temp)
#     static_transformStamped.transform.rotation.x = quat[0]
#     static_transformStamped.transform.rotation.y = quat[1]
#     static_transformStamped.transform.rotation.z = quat[2]
#     static_transformStamped.transform.rotation.w = quat[3]

#     return static_transformStamped



import threading

user_input = threading.Event()

def get_input():
    temp = raw_input("Press [ENTER] Key to calibrate") # Something akin to this    user_flag.append( True)
    user_input.set()

#
#
#   MAIN
#
#
if __name__ == '__main__':
    rospy.init_node('Camera_Calibrator_Node', anonymous=True)

    print(" ++++++++++++++++ CALIBRATION NODE +++++++++++++++++++++++++++")
    print(" ")
    print("                   (`.                  ")
    print("                    \\ `.               ")
    print("                     )  `._..---._      ")
    print("   \\`.       __...---`         o  )    ")
    print("    \\ `._,--'           ,    ___,'     ")
    print("     ) ,-._          \\  )   _,-'       ")
    print("    /,'    ``--.._____\\/--''           ")
    print(" ")
    print("Setup your Camera and Press [ENTER] to calibrate")
    print(" +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

    TAG_SIZE = rospy.get_param("Calibration/TagSize")
    if rospy.has_param("Calibration/CalibrationMode"):
        CALIBRATION_MODE = rospy.get_param("Calibration/CalibrationMode")
    else: # default values
        CALIBRATION_MODE = "corners"
    output_filename = rospy.get_param("Calibration/output_filename")


    #
    #    Open the RS camera
    #    
    print("[CALIBRATION NODE]: No 'Camera' param found in config file. Opening default Realsense Camera")
    camera = RealSenseCamera("RS_CAMERA", None,
                                color_width=1920, color_height=1080, depth_width=1280, depth_height=720, frame_rate=15)  


    # StaticBroadcaster = tf2_ros.StaticTransformBroadcaster()
    pub = rospy.Publisher("calibration/camera/image_with_tags", Image, queue_size=5)
    pub_depth = rospy.Publisher("calibration/camera/depth", Image, queue_size=5)
    pub_pcl = rospy.Publisher("calibration/camera/pcl", PointCloud2, queue_size=1)
    cvBridge = CvBridge()

    #
    #   START CAMER0.00, 0.00, 0.0000 ]A
    #
    camera.start()
    # do *NOT* change this: get_intrisincs() must be called after start()
    camera_params = camera.get_intrisincs()
    calibrator = Calibrator.Calibrator(TAG_SIZE, camera, CALIBRATION_MODE)


    rate = rospy.Rate(15)

    user_flag = []
    input_thread = threading.Thread(target=get_input) #,  daemon=True)
    input_thread.start()

    startTime = rospy.get_rostime().to_sec()
    while not rospy.is_shutdown():
        camera.run()
        # cv_image = camera.get_color_frame()
        cv_image = calibrator.run_viewer() # get cv_omage from Calibrator, so as to viz the tags
        cv_depth = camera.get_depth_frame()
        # TODO implement and use get_depth_for_view instead



        #
        # When user presses key, Calibrate
        #
        if user_input.isSet():

            print("[CALIBRATION NODE]: Estimating Transfrom..... ")
            R, t, err = calibrator.calibrate()

            if err is None:
                print("CALIBRATION NODE: FAILED. EXITING....")
            else:
                # add axis to image before publishing
                cv_image = ap_utils.draw_axis(calibrator.cv_image_with_tags,
                                              R.T, np.dot(R.T, -t), camera_params,
                                              TAG_SIZE * 60)
                # save results
                rospack = rospkg.RosPack()
                path_ = rospack.get_path('camera_robot_calibration')
                fname_ = path_ + "/output/" + output_filename
                g = np.hstack([R, t.reshape(3, 1)])
                np.savetxt(fname_, g)
                print("CALIBRATION SAVED IN OUTPUT FOLDER: " + fname_)
            break # EXIT LOOP

        pub.publish(cvBridge.cv2_to_imgmsg(cv_image))
        pub_depth.publish(cvBridge.cv2_to_imgmsg(cv_depth, encoding="mono16"))
        rate.sleep()
    # END WHILE

    # static_transformStamped = toROS(R.T, np.dot(R.T, -t), "camera_color_optical_frame", "world")
    # StaticBroadcaster.sendTransform(static_transformStamped)

    # Verify detection - Publish Cloud and Static Tf


    # Vizualise PCL
    # pcl = camera.get_pointcloud_ROSmsg()
    # pub_pcl.publish(pcl)
    # vert, tex = rs_camera.get_pointcloud_VT()
    # for i in range(0,vert.size()):
    #     if vert[i][2] is not None:
    # glVertex3fv(vertices[i]);
    # glTexCoord2fv(tex_coords[i]);

    camera.stop()

    print("++++++++++++++++++++++++++++ END +++++++++++++++++++++++++++++++++++")




