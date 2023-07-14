#!/usr/bin/env python

import numpy as np

from .BaseCamera import BaseCamera

import pyrealsense2 as rs
import rospy
from std_msgs import msg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import  PointCloud2, PointField
from std_msgs import msg
from sensor_msgs import point_cloud2
import struct


def create_D415_camera_obj():
    return RealSenseCamera("Camera_D415", "013422062273")

def create_D435i_camera_obj():
    return RealSenseCamera("Camera_D435i", "047322070390")


class RealSenseCamera(BaseCamera):

    def __init__(self, camera_ID, cameraSerialNum=None, color_width=1280, color_height=720, depth_width=1280, depth_height = 720, enable_depth=True, frame_rate=30):
        BaseCamera.__init__(self, camera_ID, color_width, color_height, depth_width, depth_height, enable_depth)
        try:
            # TODO: pass "device id" as arg and create appropriate pipe. This is necessary for multiple RS cameras.
            self.pipeline = rs.pipeline()
        except:
            raise "[REALSENSE_CAMERA]: REALSENSE CAMERA "+ camera_ID + " couldn't establish pipeline"

        self.config = rs.config()
        if cameraSerialNum is not None:
            self.config.enable_device(cameraSerialNum)
        self.config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, frame_rate)
        self.config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, frame_rate)
        self.align_filter = rs.align(rs.stream.color)
        self.hole_filter = rs.hole_filling_filter()
        self.spatial_filter = rs.spatial_filter()
        # for pointcloud only
        if color_height == 1080:
            self.dec_filter = rs.decimation_filter(5)
        elif color_height == 720:
            self.dec_filter = rs.decimation_filter(3)
        else:
            self.dec_filter = rs.decimation_filter(2)

        self.profile = None
        self.timestamp = None

    def start(self):
        self.profile = self.pipeline.start(self.config)
        print("REALSENSE_CAMERA: Camera has Started...")

    def stop(self):
        self.pipeline.stop()
        print("REALSENSE_CAMERA: Camera is Stopped")


    def run(self):
        # poll for frames maybe?
        self.frames = self.pipeline.wait_for_frames()
        self.timestamp = self.frames.get_timestamp()

        # align depth
        if self.enable_depth:
            self.frames = self.align_filter.process(self.frames)

        color_frame = self.frames.get_color_frame()
        self.cv_rgb_image = np.asanyarray(color_frame.get_data())

        if self.enable_depth:
            depth_frame = self.frames.get_depth_frame()
            filtered = depth_frame
            # include other filters as well...
            # filtered = self.spatial_filter.process(filtered)
            filtered = self.hole_filter.process(filtered)

            self.cv_depth_image = np.asanyarray(filtered.get_data())
        else:
            self.cv_depth_image = None

    def get_timestamp_msec(self):
        return self.timestamp

    #
    #   returns camera_params = (fx,fy, cx,cy)
    #
    def get_intrisincs(self):
        intrisincs = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        camera_params = (intrisincs.fx, intrisincs.fy, intrisincs.ppx, intrisincs.ppy)
        return camera_params

    def get_reverse_projection(self, x, y):
        camera_params = self.get_intrisincs()
        fx, fy, cx, cy = [c for c in camera_params]
        u, v = np.int(np.ceil(x)), np.int(np.ceil(y))
        if (u >= 0 and u < self.cv_depth_image.shape[1]) and (v >= 0 and v < self.cv_depth_image.shape[0]):
            dist = self.cv_depth_image[v, u]
            Z = np.float(dist) / 1000
            X = (x - cx) * Z / fx
            Y = (y - cy) * Z / fy
        else:
            X = None
            Y = None
            Z = None
        return X,Y,Z


    def get_pointcloud_ROSmsg(self, decimation = True):
        pc = rs.pointcloud()

        color = self.frames.get_color_frame()
        depth = self.frames.get_depth_frame()


        if decimation:
            color = self.dec_filter.process(color)
            depth = self.dec_filter.process(depth)
        depth = self.spatial_filter.process(depth)
        depth = self.hole_filter.process(depth)
        pc.map_to(color)
        points = pc.calculate(depth)
        h = msg.Header()
        h.frame_id = "camera_color_optical_frame"
        h.stamp = rospy.Time.now()

        N = points.size()
        vert = np.asanyarray(points.get_vertices())
        text = np.asanyarray(points.get_texture_coordinates())
        color_map = np.asanyarray(color.get_data())
        dims = color_map.shape
        P = []
        for i in range(0, N):
            x = np.float(vert[i][0])
            y = np.float(vert[i][1])
            z = np.float(vert[i][2])
            u,v = text[i][0], text[i][1]
            # print u,v
            u,v = np.int(u * dims[1]), np.int(v * dims[0])
            if 0<u and u<dims[1] and 0<v and v<dims[0]:
                b,g,r =  color_map[v,u,0:3]
                a = 255
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                P.append([x,y,z, rgb])

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1),
                  ]
        pc2 = point_cloud2.create_cloud(h, fields, P)

        return pc2


