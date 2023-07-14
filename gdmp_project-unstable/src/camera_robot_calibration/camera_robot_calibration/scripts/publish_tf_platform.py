#!/usr/bin/env python
import rospy, rospkg

import tf
import tf2_ros

import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation 

def read_calibration_file(calib_file):
    rospack = rospkg.RosPack()
    path_ = rospack.get_path('camera_robot_calibration')
    fname_ = path_ + "/output/" + calib_file 
    print "CALIBRATION FILE: " + fname_
    g = np.loadtxt(fname_)

    R = g[:,0:3]
    t = g[:,3]
    t = t.reshape(3,1)
    return R,t

class Quaternion:
    def __init__(self, w=1, x=0, y=0, z=0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __copy__(self):
        return Quaternion(w=self.w, x=self.x, y=self.y, z=self.z)

    def copy(self):
        return self.__copy__()

    def as_vector(self, convention='wxyz'):
        if convention == 'wxyz':
            return np.array([self.w, self.x, self.y, self.z])
        elif convention == 'xyzw':
            return np.array([self.x, self.y, self.z, self.w])
        else:
            raise RuntimeError

    def normalize(self):
        q = self.as_vector()
        q = q / np.linalg.norm(q)
        self.w = q[0]
        self.x = q[1]
        self.y = q[2]
        self.z = q[3]

    def vec(self):
        return np.array([self.x, self.y, self.z])

    def error(self, quat_desired):
        return - self.w * quat_desired.vec() + quat_desired.w * self.vec() + np.matmul(skew_symmetric(quat_desired.vec()), self.vec())

    def rotation_matrix(self):
        """
        Transforms a quaternion to a rotation matrix.
        """
        n  = self.w
        ex = self.x
        ey = self.y
        ez = self.z

        R = np.eye(3)

        R[0, 0] = 2 * (n * n + ex * ex) - 1
        R[0, 1] = 2 * (ex * ey - n * ez)
        R[0, 2] = 2 * (ex * ez + n * ey)

        R[1, 0] = 2 * (ex * ey + n * ez)
        R[1, 1] = 2 * (n * n + ey * ey) - 1
        R[1, 2] = 2 * (ey * ez - n * ex)

        R[2, 0] = 2 * (ex * ez - n * ey)
        R[2, 1] = 2 * (ey * ez + n * ex)
        R[2, 2] = 2 * (n * n + ez * ez) - 1

        return R;

    @classmethod
    def from_vector(cls, vector, order='wxyz'):
        if order == 'wxyz':
            return cls(w=vector[0], x=vector[1], y=vector[2], z=vector[3])
        elif order == 'xyzw':
            return cls(w=vector[3], x=vector[0], y=vector[1], z=vector[2])
        else:
            raise ValueError('Order is not supported.')

    @classmethod
    def from_rotation_matrix(cls, R):
        """
        Transforms a rotation matrix to a quaternion.
        """

        q = [None] * 4

        tr = R[0, 0] + R[1, 1] + R[2, 2]

        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2  # S=4*qwh
            q[0] = 0.25 * S
            q[1] = (R[2, 1] - R[1, 2]) / S
            q[2] = (R[0, 2] - R[2, 0]) / S
            q[3] = (R[1, 0] - R[0, 1]) / S

        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
          S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*qx
          q[0] = (R[2, 1] - R[1, 2]) / S
          q[1] = 0.25 * S
          q[2] = (R[0, 1] + R[1, 0]) / S
          q[3] = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
          S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*qy
          q[0] = (R[0, 2] - R[2, 0]) / S
          q[1] = (R[0, 1] + R[1, 0]) / S
          q[2] = 0.25 * S
          q[3] = (R[1, 2] + R[2, 1]) / S
        else:
          S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*qz
          q[0] = (R[1, 0] - R[0, 1]) / S
          q[1] = (R[0, 2] + R[2, 0]) / S
          q[2] = (R[1, 2] + R[2, 1]) / S
          q[3] = 0.25 * S

        result = q / np.linalg.norm(q);
        return cls(w=result[0], x=result[1], y=result[2], z=result[3])

    @classmethod
    def from_dict(cls, quat):
        return cls(w=quat['w'], x=quat['x'], y=quat['y'], z=quat['z'])

    def __str__(self):
        return str(self.w) + " + " + str(self.x) + "i +" + str(self.y) + "j + " + str(self.z) + "k"

    def rot_z(self, theta):
        mat = self.rotation_matrix()
        mat =  np.matmul(mat, rot_z(theta))
        new = Quaternion.from_rotation_matrix(mat)
        self.w = new.w
        self.x = new.x
        self.y = new.y
        self.z = new.z
        return self

    def log(self):
        if abs(self.w - 1) < 1e-12:
            return np.zero(3)
        vec_norm = lin.norm(self.vec())
        return math.atan2(vec_norm, self.w) * self.vec() / vec_norm;

    def mul(self, second):
        result = Quaternion()
        result.w = self.w * second.w - np.dot(self.vec(), second.vec())
        vec = self.w * second.vec() + second.w * self.vec() + np.cross(self.vec(), second.vec())
        result.x = vec[0]
        result.y = vec[1]
        result.z = vec[2]
        return result

    def inverse(self):
        result = Quaternion();
        temp = pow(self.w, 2) + pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2)
        result.w = self.w / temp
        result.x = - self.x / temp
        result.y = - self.y / temp
        result.z = - self.z / temp
        return result

    def log_error(self, desired):
        diff = self.mul(desired.inverse())
        if (diff.w < 0):
            diff.x = - diff.x
            diff.y = - diff.y
            diff.z = - diff.z
            diff.w = - diff.w
        return 2.0 * diff.log()


if __name__ == '__main__':

        rospy.init_node('camera_tf_broadcaster')
        rate = rospy.Rate(5.0)
        broadcaster = tf2_ros.TransformBroadcaster()
        broadcaster2 = tf2_ros.TransformBroadcaster()
        broadcaster3 = tf2_ros.TransformBroadcaster()
        broadcaster4 = tf2_ros.TransformBroadcaster()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
       
        
      

        R,t = read_calibration_file("realsense_link.dat")
        t.reshape(3)
        camera_link_to_optical = np.vstack((np.hstack((R, t)), [0, 0, 0 ,1]))             

        R,t = read_calibration_file("conveyor_belt_calibration.dat")
        
        R = np.linalg.inv(R)
        t = -R.dot(t)         

        optical_to_conveyor = np.vstack((np.hstack((R, t)), [0, 0, 0 ,1]))
      
      
        R,t = read_calibration_file("UR5e_platform_realsense.dat")
        t.reshape(3)
        base_to_optical = np.vstack((np.hstack((R, t)), [0, 0, 0 ,1]))   

        R,t = read_calibration_file("UR5e_platform_zed2.dat")
        t.reshape(3)
        base_to_human_track = np.vstack((np.hstack((R, t)), [0, 0, 0 ,1])) 
       
         #   base - > came_link =   base->cam_color  *  (camlink -> cam_color )^-1     

        base_to_camera_link = np.matmul ( base_to_optical , np.linalg.inv(camera_link_to_optical)   )
        print(base_to_camera_link)


        #  cam_link -> conveyor

        camera_link_to_conveyor = np.matmul ( camera_link_to_optical , optical_to_conveyor )
        print(camera_link_to_conveyor)

        base_to_camera_link_Stamped = geometry_msgs.msg.TransformStamped()
        base_to_camera_link_Stamped.header.stamp = rospy.Time.now()
        base_to_camera_link_Stamped.header.frame_id = "world"
        base_to_camera_link_Stamped.child_frame_id = "camera_link"

        q = Quaternion.from_rotation_matrix(base_to_camera_link[0:3,0:3])
        base_to_camera_link_Stamped.transform.rotation.x = q.x
        base_to_camera_link_Stamped.transform.rotation.y = q.y
        base_to_camera_link_Stamped.transform.rotation.z = q.z
        base_to_camera_link_Stamped.transform.rotation.w = q.w

        base_to_camera_link_Stamped.transform.translation.x = base_to_camera_link[0,3]
        base_to_camera_link_Stamped.transform.translation.y = base_to_camera_link[1,3]
        base_to_camera_link_Stamped.transform.translation.z = base_to_camera_link[2,3]
        print(base_to_camera_link_Stamped.transform.translation)


        camera_link_to_conveyor_Stamped = geometry_msgs.msg.TransformStamped()
        camera_link_to_conveyor_Stamped.header.stamp = rospy.Time.now()
        camera_link_to_conveyor_Stamped.header.frame_id = "camera_link"
        camera_link_to_conveyor_Stamped.child_frame_id = "conveyor"

        q = Quaternion.from_rotation_matrix(camera_link_to_conveyor[0:3,0:3])
        camera_link_to_conveyor_Stamped.transform.rotation.x = q.x
        camera_link_to_conveyor_Stamped.transform.rotation.y = q.y
        camera_link_to_conveyor_Stamped.transform.rotation.z = q.z
        camera_link_to_conveyor_Stamped.transform.rotation.w = q.w

        camera_link_to_conveyor_Stamped.transform.translation.x = camera_link_to_conveyor[0,3]
        camera_link_to_conveyor_Stamped.transform.translation.y = camera_link_to_conveyor[1,3]
        camera_link_to_conveyor_Stamped.transform.translation.z = camera_link_to_conveyor[2,3]  

        camera_link_to_camera_optical_Stamped = geometry_msgs.msg.TransformStamped()
        camera_link_to_camera_optical_Stamped.header.stamp = rospy.Time.now()
        camera_link_to_camera_optical_Stamped.header.frame_id = "camera_link"
        camera_link_to_camera_optical_Stamped.child_frame_id = "camera_color_optical_frame"

        q = Quaternion.from_rotation_matrix(camera_link_to_optical[0:3,0:3])
        camera_link_to_camera_optical_Stamped.transform.rotation.x = q.x
        camera_link_to_camera_optical_Stamped.transform.rotation.y = q.y
        camera_link_to_camera_optical_Stamped.transform.rotation.z = q.z
        camera_link_to_camera_optical_Stamped.transform.rotation.w = q.w

        camera_link_to_camera_optical_Stamped.transform.translation.x = camera_link_to_optical[0,3]
        camera_link_to_camera_optical_Stamped.transform.translation.y = camera_link_to_optical[1,3]
        camera_link_to_camera_optical_Stamped.transform.translation.z = camera_link_to_optical[2,3]

        base_to_human_tracker_Stamped = geometry_msgs.msg.TransformStamped()
        base_to_human_tracker_Stamped.header.stamp = rospy.Time.now()
        base_to_human_tracker_Stamped.header.frame_id = "world"
        base_to_human_tracker_Stamped.child_frame_id = "camera_link_human_tracker"

        q = Quaternion.from_rotation_matrix(base_to_human_track[0:3,0:3])
        base_to_human_tracker_Stamped.transform.rotation.x = q.x
        base_to_human_tracker_Stamped.transform.rotation.y = q.y
        base_to_human_tracker_Stamped.transform.rotation.z = q.z
        base_to_human_tracker_Stamped.transform.rotation.w = q.w

        base_to_human_tracker_Stamped.transform.translation.x = base_to_human_track[0,3]
        base_to_human_tracker_Stamped.transform.translation.y = base_to_human_track[1,3]
        base_to_human_tracker_Stamped.transform.translation.z = base_to_human_track[2,3]  


        while not rospy.is_shutdown():
            base_to_camera_link_Stamped.header.stamp = rospy.Time.now()
            camera_link_to_conveyor_Stamped.header.stamp = rospy.Time.now()
            camera_link_to_camera_optical_Stamped.header.stamp = rospy.Time.now()
            base_to_human_tracker_Stamped.header.stamp = rospy.Time.now()
            broadcaster.sendTransform(base_to_camera_link_Stamped)
            broadcaster2.sendTransform(camera_link_to_conveyor_Stamped)
            broadcaster3.sendTransform(camera_link_to_camera_optical_Stamped)
            broadcaster4.sendTransform(base_to_human_tracker_Stamped)

            rate.sleep()

        rospy.spin()
        

