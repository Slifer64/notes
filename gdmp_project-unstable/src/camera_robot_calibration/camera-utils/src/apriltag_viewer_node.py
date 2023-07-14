#!/usr/bin/env python


#   Aristotle Univercisty of Thessaloniki
#   Robotics and Automation Lab
#
#   Author: Savvas Sampaziotis
#   e-mail: savvas.sampaziotis@gmail.com
#


import rospy

from .apriltag_utils import apriltag_viewer


if __name__ == '__main__':

    rospy.init_node('Camera_Calibrator_Node', anonymous=True)

    sub_topic_str = "camera/color/image_raw"
    pub_topic_str = "apriltag_viewer/rgb_image"

    print "********************************************************"
    print " ****** APRIL TAG VIEWER HAS STARTED ******** "
    print " "
    print "                   (`.              "
    print "                    \\ `.           "
    print "                     )  `._..---._"
    print "   \\`.       __...---`         o  )"
    print "    \\ `._,--'           ,    ___,'"
    print "     ) ,-._          \\  )   _,-' "
    print "    /,'    ``--.._____\\/--''  "
    print " "
    print "********************************************************"
    AprilTagViewer(sub_topic_str, pub_topic_str)
    rospy.spin()





