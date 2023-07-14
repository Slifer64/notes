import numpy as np
import torch
import matplotlib.pyplot as plt
from my_pkg.util.cv_tools import draw_orient_trajectory_on_image
from my_pkg.models import *

import rospy
import rospkg
import actionlib
from cv_bridge import CvBridge, CvBridgeError
import grasp_obj_controller.msg

import cv2

plt.ion()
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

if __name__ == '__main__':

    rospy.init_node("temp")

    model = load_model('models/rn18_dmp.bin')
    # model = load_model('models/rn18_dmp_ptr.bin')
    model.to(device)
    model.eval()

    img_path = rospkg.RosPack().get_path('dummy_') + '/data/dummy_camera/rgb.png'

    rgb_img0 = cv2.cvtColor(cv2.imread(img_path, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)

    fig, ax = plt.subplots()
    ax.axis('off')
    ax.imshow(rgb_img0)
    plt.pause(0.001)

    # convert to torch.tensor, permute channels to (C, H, W), add batch dim
    rgb_img = torch.tensor(rgb_img0, dtype=torch.float32, device=device).permute(2, 0, 1)[None, ...]
    img_size = rgb_img.shape[2:4]

    with torch.no_grad():
        out = model(model.input_transform(rgb_img))
        traj = model.output_to_traj(out, img_size=img_size)

    print(out['mp_weights'].shape)

    # vizualize result
    traj = traj[0].cpu().detach().numpy()
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.axis('off')
    image = draw_orient_trajectory_on_image(rgb_img0, traj=traj[:2, :], theta=traj[2, :]+np.pi/2, traj_color=(50, 255, 255), linewidth=6, quiver_color=(150, 255, 255))
    ax.imshow(image)
    plt.pause(0.001)

    input()

    