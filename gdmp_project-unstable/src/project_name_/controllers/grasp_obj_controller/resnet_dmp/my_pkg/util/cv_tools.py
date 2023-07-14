import numpy as np
import cv2
import time
import math
from threading import Thread, Lock
from typing import Union, Tuple, List


class DrawOnImage:

    def __init__(self, image_height=480, image_width=640, draw_type='trajectory',
                 record_stops=True, logger_Ts=0.02, dist_thres=-1, point_radius=5):

        self.win_name = 'Draw Path'

        cv2.namedWindow(winname=self.win_name)

        self.dist_thres = dist_thres  # register a new point if the  L2 distance in pixels from the last point is greater than this threshold
        self.point_radius = point_radius
        self.record_stops = record_stops
        self.logger_Ts = logger_Ts

        # trajectory params
        self.Time = []  # timestamp of each recorded point
        self.points = []

        self.color = (255, 0, 0)
        self.drawing = False
        self.img0 = np.zeros((image_height, image_width, 3), np.uint8)
        self.img = self.img0.copy()

        self.t0 = time.time()

        self.new_point = []
        self.lock = Lock()
        self.run_logger = True

        self.draw_type = draw_type.lower()

        if self.draw_type == 'line':
            self.points = [(-1, -1), (-1, -1)]
            cv2.setMouseCallback(self.win_name, DrawOnImage.__draw_line_callback, self)
        else:
            cv2.setMouseCallback(self.win_name, DrawOnImage.__draw_trajectory_callback, self)

    def __del__(self):
        cv2.destroyWindow(self.win_name)

    def __call__(self, img: np.array) -> Tuple[np.array, np.array, np.array]:

        if self.record_stops:
            self.run_logger = True
            logger_thread = Thread(target=self.logger_thread_fun)
            logger_thread.start()

        self.img0 = img.copy()
        self.__reset_image()
        while True:
            cv2.imshow(self.win_name, cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(1)

            if key == 27:  # Esc (focus must be on cv window)
                self.__reset_image()

            elif key == 13:  # 'enter'
                self.__reset_image()
                break

            elif key in (ord('u'), ord('U')):  # reset image
                self.__reset_image()

            elif key in (ord('l'), ord('L')):
                self.draw_type = 'line'
                cv2.setMouseCallback(self.win_name, DrawOnImage.__draw_line_callback, self)

            elif key in (ord('t'), ord('T')):
                self.draw_type = 'trajectory'
                cv2.setMouseCallback(self.win_name, DrawOnImage.__draw_trajectory_callback, self)

            elif key in (ord('p'), ord('P')):
                print('Draw type:', self.draw_type)

            elif key == 3: # Ctrl + C
                break

        if self.record_stops:
            self.run_logger = False
            logger_thread.join()

        x_data, y_data = map(np.array, zip(*self.points))
        # y_data = [-y for y in y_data]

        return x_data, y_data, np.array(self.Time)

    def __reset_image(self):
        self.img = self.img0.copy()

    def logger_thread_fun(self):

        while self.run_logger:
            if self.drawing:
                self.lock.acquire()
                self.points.append(self.new_point)
                self.Time.append(time.time() - self.t0)
                self.lock.release()
            time.sleep(self.logger_Ts)

    @staticmethod
    def __draw_line_callback(event, x, y, flags, self):

        if self.drawing:
            self.points[1] = (x, y)

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.points[0] = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                self.__reset_image()
                cv2.line(self.img, pt1=self.points[0], pt2=self.points[1], color=self.color, thickness=4)

        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.__reset_image()
            cv2.line(self.img, pt1=self.points[0], pt2=self.points[1], color=self.color, thickness=4)

    @staticmethod
    def __draw_trajectory_callback(event, x, y, flags, self):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.__reset_image()
            self.drawing = True

            self.lock.acquire()
            new_point = (x, y)
            self.points = [new_point]
            self.t0 = time.time()
            self.Time = [0.]
            cv2.circle(self.img, new_point, self.point_radius, self.color, thickness=-1)
            self.new_point = new_point
            self.lock.release()

        elif event == cv2.EVENT_LBUTTONUP:  # must be before the next elif, otherwise it will never stop!
            self.drawing = False

        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            last_point = self.points[-1]
            new_point = (x, y)
            if np.linalg.norm(np.array(last_point) - np.array(new_point)) > self.dist_thres:
                self.lock.acquire()
                self.new_point = new_point
                self.lock.release()
                if not self.record_stops:
                    self.points.append(new_point)
                    self.Time.append(time.time() - self.t0)
                cv2.circle(self.img, new_point, self.point_radius, self.color, thickness=-1)

def draw_orient_trajectory_on_image(image: np.array, traj: np.array, theta: np.array, traj_color=(255, 0, 0), linewidth=6, quiver_color=(0, 255, 0), draw_quiver_dist=20, quiver_width=2, quiver_lenth=50):
    image = draw_trajectory_on_image(image, traj, color=traj_color, linewidth=linewidth)
    image = draw_quiver_on_image(image, theta, traj, dist_thres=draw_quiver_dist, color=quiver_color, linewidth=quiver_width, arrow_length=quiver_lenth)
    return image


def draw_trajectory_on_image(image: np.array, action_traj: np.array, color=(255, 0, 0), linewidth=6) -> np.array:

    # if not image.data.contiguous:
    image = image.copy()

    is_float_ = False
    if str(image.dtype) == 'float32':
        image = (image * 255).astype(np.uint8)
        is_float_ = True

    for j in range(action_traj.shape[1]-1):
        point1 = (int(action_traj[0, j]), int(action_traj[1, j]))
        point2 = (int(action_traj[0, j+1]), int(action_traj[1, j+1]))
        cv2.line(img=image, pt1=point1, pt2=point2, color=color, thickness=linewidth)
    cv2.circle(img=image, center=(int(action_traj[0, -1]), int(action_traj[1, -1])), color=color,
               radius=2*linewidth, thickness=linewidth)

    if is_float_:
        image = image.astype(np.float32) / 255.

    return image



def draw_quiver_on_image(image: np.array, theta: np.array, traj: np.array, dist_thres=20, color=(0, 255, 0), linewidth=2, arrow_length=50) -> np.array:

    def directionVec(theta):
        c = math.cos(theta)
        s = math.sin(theta)
        R = np.array([[c, -s], [s,  c]])
        
        v = np.matmul(R, np.array([1, 0]).reshape(-1, 1))
        v[1] = -v[1] # because y faces downwards in the image
        return v

    # if not image.data.contiguous:
    image = image.copy()

    is_float_ = False
    if str(image.dtype) == 'float32':
        image = (image * 255).astype(np.uint8)
        is_float_ = True


    dist = 0.0
    p_data = [traj[:,0].reshape(-1,1)]
    v_data = [directionVec(theta[0])]
    n_points = traj.shape[1]
    for j in range(2, n_points):
        dist = dist + np.linalg.norm(traj[:,j] - traj[:,j-1])
        if dist > dist_thres:
            dist = 0.0
            p_data.append(traj[:,j].reshape(-1,1))
            v_data.append(directionVec(theta[j]))
    
    # alway include the last point, if it was included on the last iteration
    if dist > 0.0:
        p_data.append(traj[:,-1].reshape(-1,1))
        v_data.append(directionVec(theta[-1]))

    p1_data = np.hstack(p_data)
    v_data = np.hstack(v_data)
    p2_data = p1_data + arrow_length*v_data

    for j in range(len(p_data)):
        point1 = (int(p1_data[0,j]), int(p1_data[1,j]))
        point2 = (int(p2_data[0,j]), int(p2_data[1,j]))
        image = cv2.arrowedLine(image, pt1=point1, pt2=point2, color=color, thickness=linewidth) 

    if is_float_:
        image = image.astype(np.float32) / 255.

    return image



def batch_imshow(images: Union[np.array, List[np.array]], figsize=(12, 9)):

    import math
    import matplotlib.pyplot as plt

    if isinstance(images, list):
        images = np.stack(images, dim=0)

    n_images = images.shape[0]
    if n_images < 4:
        n_rows = 1
        n_cols = n_images
    else:
        n_rows = int(math.sqrt(n_images) + 0.5)
        n_cols = int(n_images / n_rows + 0.5)

    fig, ax = plt.subplots(n_rows, n_cols, figsize=figsize)

    if n_rows == 1:
        ax = np.array([ax])
    if n_cols == 1:
        ax = ax[..., None]

    for row in ax:
        for ax_ in row:
            ax_.axis('off')
    for k in range(n_images):
        image = images[k]
        i, j = np.unravel_index(k, (n_rows, n_cols))  # k // n_cols, k % n_cols
        ax[i, j].imshow(np.clip(image, 0, 1))
    plt.pause(0.001)

    return fig
