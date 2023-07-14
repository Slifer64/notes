



# import rospy


from camera_wrappers.ZED2Camera  import ZED2Camera
import cv2
import math
import numpy as np
import sys
import pyzed.sl as sl


#
#   MAIN
#
#
if __name__ == '__main__':
    print("-----CAMERA DEMO------")

    camera = ZED2Camera()

    camera.start()
    try:
        while True:
            camera.run()
            camera.get_intrisincs()
            # cv2.imshow("", camera.get_color_frame())
            # cv2.imshow("", np.hstack([camera.get_color_frame(), ]))
            depth_im = camera.get_depth_frame_for_view()
            depth_colormap_im = cv2.applyColorMap(cv2.cvtColor(depth_im, cv2.COLOR_RGB2GRAY), cv2.COLORMAP_JET)

            x,y =1280/2, 720/2
            color_im = camera.get_color_frame()
            color_im = cv2.drawMarker(color_im, (640,360), (255,0,255), thickness=20)
            added_image = cv2.addWeighted(color_im[:,:,:3], 1, depth_colormap_im, 0.6, 0)

            # print(camera.get_reverse_projection(x,y))

            cv2.imshow("", np.hstack([added_image, depth_im[:,:,:3]]))
            cv2.waitKey(5)
    finally:
        camera.stop()









