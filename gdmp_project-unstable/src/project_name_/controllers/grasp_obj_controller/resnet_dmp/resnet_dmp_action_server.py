import numpy as np
import torch
import cv2
import matplotlib.pyplot as plt
from my_pkg.util.cv_tools import draw_orient_trajectory_on_image
from my_pkg.models import *

import rospy
import actionlib
from cv_bridge import CvBridge, CvBridgeError
import grasp_obj_controller.msg


plt.ion()
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# models = {
#     'rn18_dmp': 'models/rn18_dmp.bin',
#     'rn18_dmp_ptr': 'models/rn18_dmp_ptr.bin',
#     'rn18_mddmp_ptr': 'models/rn18_mddmp_ptr.bin',
#     'vimednet_ptr': 'models/vimednet_ptr.bin',
# }

def parse_args():

    args = {}
    
    # import argparse
    # parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # parser.add_argument('--action_name', required=True, type=str, 
    #                     help='The action name')
    # args = vars(parser.parse_args())

    import yaml
    with open("../config/params.yaml", "r") as stream:
        try:
            ctrl_params = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            raise RuntimeError('\33[1;31mERROR loading "params.yaml" file: \33[0m' + str(exc))
    args['action_name'] = ctrl_params['resnet_action_name']

    return args


class ResnetDMPAction(object):
    # create messages that are used to publish feedback/result
    _feedback = grasp_obj_controller.msg.ImageToMPFeedback()
    _result = grasp_obj_controller.msg.ImageToMPResult()

    def __init__(self, name):
        
        self.cv_br = CvBridge()

        self.fig = None

        # load models
        print('\33[1;34mLoading models...\33[0m')

        import yaml
        with open("../config/models.yaml", "r") as stream:
            try:
                models = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                raise RuntimeError('\33[1;31mERROR loading models from yaml file: \33[0m' + str(exc))

        self._models_dict = {}
        for k, v in models.items():
            model = load_model(v)
            model.to(device)
            model.eval()
            self._models_dict[k] = model

        self.roi_detect_model = None
        self.roi_detect_model = ResnetSegmentation.load('models/resnet18_seg.bin')
        self.roi_detect_model.to(device)
        self.roi_detect_model.eval()


        print('\33[1;34mCreating action server...\33[0m')
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, grasp_obj_controller.msg.ImageToMPAction, execute_cb=self.execute_cb, auto_start = False)
        
        self._as.start()
        print('\33[1;34mAction server is running!\33[0m')
    
    def get_roi_mask(self, rgb_img: torch.Tensor, obj_id: int):

        if rgb_img.ndim == 3: rgb_img = rgb_img[None, ...]  # add batch dim

        with torch.no_grad():
            pred_class, prob = self.roi_detect_model.output(rgb_img, return_prob=True)
            pred_class[prob < 0.85] = 0

        pred_class = pred_class[0].detach().cpu().numpy()
        mask = ((pred_class == obj_id) * 255).astype(np.uint8)
        
        contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        if not contours:
            return None

        max_contour = contours[0]
        for contour in contours:
            if cv2.contourArea(contour) > cv2.contourArea(max_contour):
                max_contour = contour

        (cx, cy), radius = cv2.minEnclosingCircle(max_contour)
        cx, cy, radius = map(lambda x: int(x), (cx, cy, radius))
        radius = int(0.99*radius + 0.5)

        img_size = rgb_img.shape[2:4]
        roi_mask = np.zeros((*img_size, 3), dtype=np.uint8)
        cv2.circle(roi_mask, (cx, cy), radius, (255, 255, 255), -1)
        roi_mask = roi_mask[..., 0] # keep only one channel

        return torch.tensor(roi_mask, dtype=torch.uint8, device=device)


    def execute_cb(self, goal: grasp_obj_controller.msg.ImageToMPGoal):

        if self.fig is not None:
            plt.close(self.fig)

        # goal.rgb
        # goal.model_name

        # load model
        try:
            model = self._models_dict[goal.model_name]
        except KeyError:
            self._result.success = False
            self._result.msg = "Model " + goal.model_name + " does not exist!"
            self._as.set_succeeded(self._result)
            return

        # convert image msg
        try:
            rgb_img0 = np.asarray(self.cv_br.imgmsg_to_cv2(goal.rgb, "rgb8"), dtype=np.float32) / 255.
        except CvBridgeError as e:
            self._result.success = False
            self._result.msg = str(e)
            self._as.set_succeeded(self._result)
            return

        # convert to torch.tensor, permute channels to (C, H, W), add batch dim
        rgb_img = torch.tensor(rgb_img0, dtype=torch.float32, device=device).permute(2, 0, 1)[None, ...]
        img_size = rgb_img.shape[2:4]

        roi_mask = None

        with torch.no_grad():
            inputs = {'rgb': rgb_img}

            if isinstance(model, ImgMaskDMP_net):
                roi_mask = self.get_roi_mask(rgb_img, goal.target_obj_id)
                if roi_mask is None:
                    self._result.success = False
                    self._result.msg = "Failed to find the target object with id" + str(goal.target_obj_id)
                    self._as.set_succeeded(self._result)
                    return
                inputs['mask'] = roi_mask[None, ...]

            out = model(model.input_transform(inputs))
            traj = model.output_to_traj(out, img_size=img_size)

        # assign the mp weights to the result msg
        W = out['mp_weights'][0]
        self._result.mp_x_weights = W[0, :].tolist()
        self._result.mp_y_weights = W[1, :].tolist()
        self._result.mp_theta_weights = W[2, :].tolist()

        # vizualize result
        traj = traj[0].cpu().detach().numpy()
        self.fig, ax = plt.subplots(figsize=(4,3))
        ax.axis('off')
        image = rgb_img0.copy()
        # image = draw_orient_trajectory_on_image(image, traj=traj[:2, :], theta=traj[2, :]+np.pi/2, traj_color=(50, 255, 50), linewidth=6, quiver_color=(255, 50, 50))
        if roi_mask is not None:
            mask = np.repeat(roi_mask.squeeze().detach().cpu().numpy()[..., None], 3, axis=2).astype(np.float32) / 255.0
            masked_image = cv2.addWeighted(image, 0.55, mask, 0.45, 0)
            # image = np.hstack([image, np.ones((image.shape[0], 30, 3)),  masked_image])
            image = draw_orient_trajectory_on_image(masked_image, traj=traj[:2, :], theta=traj[2, :]+np.pi/2, traj_color=(50, 255, 50), linewidth=6, quiver_color=(255, 50, 50))
        ax.imshow(image)
        plt.pause(0.001)

        # finish
        self._result.success = True
        self._result.msg = "Calculated mp weigts!"
        self._as.set_succeeded(self._result)


if __name__ == '__main__':

    rospy.init_node("resnet_dmp_node")

    args = parse_args()

    server = ResnetDMPAction(args['action_name'])

    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.1)

    # rospy.spin()

    