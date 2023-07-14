#!/usr/bin/env python
import rospy, rospkg

import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np

def read_calibration_file():
    rospack = rospkg.RosPack()
    path_ = rospack.get_path('camera_robot_calibration')
    fname_ = path_ + "/output/conveyor_belt_calibration.dat"
    print "CALIBRATION FILE: " + fname_
    g = np.loadtxt(fname_)

    R = g[:,0:3]
    t = g[:,3]
    t = t.reshape(3,1)
    return R,t


if __name__ == '__main__':

        rospy.init_node('static_tf2_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        # broadcaster = tf2_ros.TransformBroadcaster()

        R,t = read_calibration_file()

        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "conveyor_belt"
        static_transformStamped.child_frame_id = "camera_color_optical_frame"

        static_transformStamped.transform.translation.x = t[0]
        static_transformStamped.transform.translation.y = t[1]
        static_transformStamped.transform.translation.z = t[2]

        R_temp = np.hstack([np.vstack([R, np.zeros(3)]), np.array([0, 0, 0, 1]).reshape(4, 1)])

        quat = tf.transformations.quaternion_from_matrix(R_temp)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
        rospy.spin()