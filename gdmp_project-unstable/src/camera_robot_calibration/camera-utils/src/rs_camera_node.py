#!/usr/bin/env python


import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from camera_wrappers.RealSenseCamera import RealSenseCamera



#
#   MAIN
#
#
if __name__ == '__main__':


    rospy.init_node('Camera_Calibrator_Node', anonymous=True)
    print("----------- START ------------")

    rs_camera = RealSenseCamera("Camera_D415", "013422062273",
                                color_width=640, color_height=480,
                                depth_width=640, depth_height=480,
                                frame_rate=30)  # D415

    # rs_camera = RealSenseCamera("Camera_D435i", "047322070390",
    #                             color_width=1920, color_height=1080,
    #                             depth_width=1280, depth_height=720,
    #                             frame_rate=30)  # D435i

    pub_rgb = rospy.Publisher("camera/colour/image_raw", Image, queue_size=5)
    pub_depth = rospy.Publisher("camera/aligned_depth/image_raw", Image, queue_size=5)
    cv_bridge = CvBridge()

    rs_camera.start()
    camera_params = rs_camera.get_intrisincs()

    print( "---------- INTRISINCS ---------------")
    print( "")
    print( " fx, fy, cx, cy")
    print (camera_params)
    print( "")
    print( "-------------------------------------")


    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rs_camera.run()

        # Depth is already alinged and hole-filled.
        # ** NO DECIMATION ** is implemented: depth is at original specified size
        color_msg = cv_bridge.cv2_to_imgmsg(rs_camera.get_color_frame())
        depth_msg = cv_bridge.cv2_to_imgmsg(rs_camera.get_depth_frame())

        pub_depth.publish(depth_msg)
        pub_rgb.publish(color_msg)

        rate.sleep()

    rs_camera.stop()
    print("----------- STOP ------------")
