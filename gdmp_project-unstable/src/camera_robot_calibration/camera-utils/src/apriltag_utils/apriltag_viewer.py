
import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from sensor_msgs.msg import Image

import numpy as np

from cv_bridge import CvBridge, CvBridgeError
import cv2

import dt_apriltags as ap

from . import apriltag_utils as ap_utils

class AprilTagViewer:

    def __init__(self, sub_topic_str, pub_topic_str, TagSize):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(sub_topic_str, Image, self.callback)
        self.pub = rospy.Publisher(pub_topic_str, Image, queue_size=10)
        self.detector = ap.Detector()
        self.TagSize = TagSize
        self.broadcaster = tf2_ros.TransformBroadcaster()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        camera_params = (642.364, 637.862, 642.364, 357.083)
        results = self.detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=self.TagSize)
        for r in results:
            cv_image = ap_utils.draw_apriltag(r, cv_image)
            cv_image = ap_utils.draw_axis(cv_image, r.pose_R, r.pose_t, camera_params, 0.02 * 10)
            tf_msg = self.toROS(r.pose_R, r.pose_t, "tag_" + str(r.tag_id))
            self.broadcaster.sendTransform(tf_msg)

        self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image))

    def toROS(self, R, t, tag):
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "camera_color_optical_frame"
        static_transformStamped.child_frame_id = tag

        static_transformStamped.transform.translation.x = t[0]
        static_transformStamped.transform.translation.y = t[1]
        static_transformStamped.transform.translation.z = t[2]

        R_temp = np.hstack([np.vstack([R, np.zeros(3)]), np.array([0, 0, 0, 1]).reshape(4, 1)])
        quat = tf.transformations.quaternion_from_matrix(R_temp)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        return static_transformStamped
