import os.path

import PIL.Image
import numpy as np
import pickle
import math
import abc
import torch
import cv2
from typing import List, Tuple, Dict, Union
from my_pkg.util.gmp import MovementPrimitive
import torchvision.transforms.functional as torchvision_F
import torchvision.transforms as torchvision_T

__all__ = [
    'BaseDataType',
    'MpTrajectory',
    'MP_2D_OrientTraj',
    'SegmentationMask',
    'InputImage',
    'BeforeImage',
    'AfterImage',
    'SegImage',
    'DepthImage',
    'ROIMask',
]


class BaseDataType:

    def __init__(self):
        self.transforms = {}

    def __call__(self, transform, *args, **kwargs) -> 'BaseDataType':
        """
        Applies a transform to the underlying data.

        Arguments:
        transform -- string with name of the transform
        *args, **kwargs -- arguments to be passed to the transform
        """

        # if the transform is not supported (implemented) skip it
        if transform not in self.transforms:
            return self
        # else apply the transform
        else:
            return self.transforms[transform](*args, **kwargs)

    def get_type(self) -> str:
        """
        Returns the class type as a string.
        """
        return self.__class__.__name__

    @abc.abstractmethod
    def numpy(self) -> np.array:
        """
        Returns the underlying raw data as an np.array.
        """
        pass

    @abc.abstractmethod
    def torch_tensor(self) -> torch.Tensor:
        """
        Returns the underlying raw data as a torch.Tensor.
        """
        pass

    @classmethod
    @abc.abstractmethod
    def load(cls, path):
        """
        Loads the data from the specified path. The filename will be chosen automatically based on the data-type,
        unless specified explicitly from additional input arguments.

        Arguments:
        path -- string, path where the file is located.
        """
        pass

    @abc.abstractmethod
    def save(self, path):
        """
        Loads the data from the specified path.

        Arguments:
        path -- string, path where the file is located.
        """
        pass


class MP_2D_OrientTraj(BaseDataType):

    data_name = 'mp_weights'
    filename = 'mp_weights.bin'

    def __init__(self, weights: np.array, im_height=480, im_width=640):

        super().__init__()

        if weights.shape[0] != 3:
            raise RuntimeError('weights must have 3 rows (DoFs)')

        self.im_height = im_height
        self.im_width = im_width

        weights = np.asarray(weights)

        n_dofs, n_kernels = weights.shape[0:2]
        self.mp = MovementPrimitive(n_dofs=n_dofs, n_kernels=n_kernels)
        self.mp.weights = weights

        self.im_height = im_height
        self.im_width = im_width

        self.transforms = {
            'hflip': self.hflip,
            'rotate': self.rotate,
            'scale': self.scale,
            'translate': self.translate,
            'perspective': self.perspective,
        }

    @classmethod
    def load(cls, path) -> 'MpTrajectory':
        filename = os.path.join(path, cls.filename)
        if not os.path.exists(filename):
            return None
        kwargs = pickle.load(open(filename, 'rb'))
        return cls(**kwargs)
        # return cls(np.loadtxt(filename))

    def save(self, path):
        kwargs = {'weights': self.mp.weights, 'im_height':self.im_height, 'im_width': self.im_width}
        pickle.dump(kwargs, open(os.path.join(path, self.filename), 'wb'))
        # np.savetxt(os.path.join(path, self.filename), self.mp.weights, delimiter=' ')

    def numpy(self) -> np.array:
        return self.mp.weights

    def torch_tensor(self) -> torch.Tensor:
        return torch.tensor(self.mp.weights)

    def get_trajectory(self, duration=5.0, time_step=0.05, img_size=None) -> np.array:
        """
        Generates the MP's trajectory, scaled to the desired img_size

        Arguments:
        duration -- float, the time duration of the generated trajectory.
        time_step -- float, the time-step between consevutive trajectory points. 
        img_size -- Tuple[int, int] with the (height, width) of the image. (default=None, uses the default values)

        Returns:
        np.array, the trajectory with DoFs along rows and time-steps alongs columns.
        """
        if img_size is None:
            img_size = [self.im_height, self.im_width]
        h, w = img_size
        traj = np.hstack([self.mp.get_pos(s) for s in np.linspace(0, 1, int(duration / time_step + 0.5))])
        traj[0:2, :] = traj[0:2, :] * np.array([w, h]).reshape(2, 1)
        return traj

    # =========== Transforms ==============

    def hflip(self) -> 'MP_2D_OrientTraj':
        s_data, pos_data = self.__generate_data()
        pos_data[0, :] = 1.0 - pos_data[0, :]
        pos_data[2, :] = -pos_data[2, :] # notice that my x-axis is the images y-axis!
        self.mp.train(s_data, pos_data)
        return self

    def rotate(self, angle: float) -> 'MP_2D_OrientTraj':
        s_data, pos_data = self.__generate_data()
        height, width = float(self.im_height) / self.im_width, 1.0
        pos_data[0:2, :] = pos_data[0:2, :] * np.array([width, height]).reshape(-1, 1)
        angle = angle * math.pi / 180.
        rotm = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        c = np.array([width, height]).reshape(-1, 1) / 2.0
        pos_data[0:2, :] = np.matmul(rotm, pos_data[0:2, :] - c) + c
        pos_data[0:2, :] = pos_data[0:2, :] / np.array([width, height]).reshape(-1, 1)
        pos_data[2, :] -= angle # because z in image points inside, and I assume an opposite z, the angle for me has minus sign
        self.mp.train(s_data, pos_data)
        return self

    def scale(self, scale: float) -> 'MP_2D_OrientTraj':
        s_data, pos_data = self.__generate_data()
        pos_data[0, :] = scale * (pos_data[0, :] - 0.5) + 0.5
        pos_data[1, :] = scale * (pos_data[1, :] - 0.5) + 0.5
        # theta remains same, since the direction vector remains the same
        self.mp.train(s_data, pos_data)
        return self

    def translate(self, translate: List[int]) -> 'MP_2D_OrientTraj':
        tx = float(translate[0]) / self.im_width
        ty = float(translate[1]) / self.im_height
        s_data, pos_data = self.__generate_data()
        pos_data[0:2, :] = pos_data[0:2, :] + np.array([tx, ty]).reshape(-1, 1)
        # theta remains same, since the direction vector remains the same
        self.mp.train(s_data, pos_data)
        return self

    def perspective(self, startpoints: List[List[float]], endpoints) -> 'MP_2D_OrientTraj':
        """ Applies perspective transformation to 'self'.

        Args:
            startpoints: List[List[float]], normalized in [0, 1) corner start-points.
            endpoints: List[List[float]], normalized in [0, 1) corner end-points.

        Returns:
            self
        """

        # scale_points = lambda points: [temp.int().tolist() for temp in torch.as_tensor(points) * torch.tensor([[self.im_width, self.im_height]])]
        # startpoints, endpoints = map(scale_points, (startpoints, endpoints))

        # (x, y) -> ( (ax + by + c) / (gx + hy + 1), (dx + ey + f) / (gx + hy + 1) )
        a, b, c, d, e, f, g, h = self._get_perspective_coeffs(startpoints, endpoints)
        s_data, pos_data = self.__generate_data()

        def thetaTodir(theta):
            c = math.cos(theta)
            s = math.sin(theta)
            R = np.array([[c, -s], [s,  c]])
            
            v = np.matmul(R, np.array([1, 0]).reshape(-1, 1))
            v[1] = -v[1] # because y faces downwards in the image
            return v
        
        def dirToTheta(v):
            return math.atan2(-v[1], v[0])  # because y faces downwards in the image

        # theta_data = pos_data[2, :]
        # v_data = np.hstack([thetaTodir(theta) for theta in theta_data])
        # nom = np.dot(np.array([[a, b], [d, e]]), v_data) + np.array([c, f]).reshape(2, 1)
        # denom = np.dot(np.array([g, h]).reshape(1, 2), v_data) + 1
        # v_data = nom / denom
        # pos_data[2, :] = np.hstack([dirToTheta(v_data[:, j]) for j in range(v_data.shape[1])])

        nom = np.dot(np.array([[a, b], [d, e]]), pos_data[0:2, :]) + np.array([c, f]).reshape(2, 1)
        denom = np.dot(np.array([g, h]).reshape(1, 2), pos_data[0:2, :]) + 1
        pos_data[0:2, :] = nom / denom

        self.mp.train(s_data, pos_data)
        return self

    # =========== Private utility functions ==============

    @staticmethod
    def _get_perspective_coeffs(startpoints: List[List[int]], endpoints: List[List[int]]) -> List[float]:
        """ Calculates the Perspective Transform coefficienys
        For each pixel (x, y) the transform is
        (x, y) -> ( (ax + by + c) / (gx + hy + 1), (dx + ey + f) / (gx + hy + 1) )
        Returns:
            octuple (a, b, c, d, e, f, g, h) for transforming each pixel.
        """
        a_matrix = torch.zeros(2 * len(startpoints), 8, dtype=torch.float)

        for i, (p1, p2) in enumerate(zip(startpoints, endpoints)):
            a_matrix[2 * i, :] = torch.tensor([p1[0], p1[1], 1, 0, 0, 0, -p2[0] * p1[0], -p2[0] * p1[1]])
            a_matrix[2 * i + 1, :] = torch.tensor([0, 0, 0, p1[0], p1[1], 1, -p2[1] * p1[0], -p2[1] * p1[1]])

        b_matrix = torch.tensor(endpoints, dtype=torch.float).view(8)
        res = torch.linalg.lstsq(a_matrix, b_matrix, driver='gels').solution

        output: List[float] = res.tolist()
        return output

    def __generate_data(self, n_points=200):
        s_data = np.linspace(0, 1, n_points)
        pos_data = np.hstack([self.mp.get_pos(s) for s in s_data])
        return s_data, pos_data


class MpTrajectory(BaseDataType):

    data_name = 'mp_weights'
    filename = 'mp_weights.bin'

    def __init__(self, weights: np.array, im_height=480, im_width=640):

        super().__init__()

        weights = np.asarray(weights)
        weights = weights.reshape(2, -1)
        n_kernels = weights.shape[1]
        self.mp = MovementPrimitive(n_dofs=2, n_kernels=n_kernels)
        self.mp.weights = weights

        self.im_height = im_height
        self.im_width = im_width

        self.transforms = {
            'hflip': self.hflip,
            'rotate': self.rotate,
            'scale': self.scale,
            'translate': self.translate,
            'perspective': self.perspective,
        }

    @classmethod
    def load(cls, path) -> 'MpTrajectory':
        filename = os.path.join(path, cls.filename)
        if not os.path.exists(filename):
            return None
        return cls(pickle.load(open(filename, 'rb')))

    def save(self, path):
        mp_weights = np.asarray(self.mp.weights).reshape(-1)
        pickle.dump(mp_weights, open(os.path.join(path, self.filename), 'wb'))

    def numpy(self) -> np.array:
        return self.torch_tensor().numpy()

    def torch_tensor(self) -> torch.Tensor:
        return torch.tensor(self.mp.weights).view(2, -1)

    def get_trajectory(self, duration=5.0, time_step=0.05, img_size=[1, 1]) -> np.array:
        """
        Generates the MP's trajectory, scaled to the desired img_size

        Arguments:
        duration -- float, the time duration of the generated trajectory.
        time_step -- float, the time-step between consevutive trajectory points. 
        img_size -- Tuple[int, int] with the (height, width) of the image. (default=[1, 1], i.e. no scaling)

        Returns:
        np.array, the trajectory with DoFs along rows and time-steps alongs columns.
        """
        h, w = img_size
        traj = np.hstack([self.mp.get_pos(s) for s in np.linspace(0, 1, int(duration / time_step + 0.5))]) * np.array([w, h]).reshape(2, 1)
        return traj

    def __getitem__(self, item: str):
        if item.lower() == 'weights':
            return self.mp.weights.copy()
        elif item.lower() == 'start_pixel':
            return self.mp.get_pos(0).reshape(-1).tolist()
        elif item.lower() == 'final_pixel':
            return self.mp.get_pos(1).reshape(-1).tolist()
        else:
            raise ValueError(f'Unsupported item "{item}"...')

    # =========== Transforms ==============

    def hflip(self) -> 'MpTrajectory':
        s_data, pos_data = self.__generate_data()
        pos_data[0, :] = 1.0 - pos_data[0, :]
        self.mp.train(s_data, pos_data)
        return self

    def rotate(self, angle: float) -> 'MpTrajectory':
        s_data, pos_data = self.__generate_data()
        height, width = float(self.im_height) / self.im_width, 1.0
        pos_data = pos_data * np.array([width, height]).reshape(-1, 1)
        angle = angle * math.pi / 180.
        rotm = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        c = np.array([width, height]).reshape(-1, 1) / 2.0
        pos_data = np.matmul(rotm, pos_data - c) + c
        pos_data = pos_data / np.array([width, height]).reshape(-1, 1)
        self.mp.train(s_data, pos_data)
        return self

    def scale(self, scale: float) -> 'MpTrajectory':
        s_data, pos_data = self.__generate_data()
        pos_data[0, :] = scale * (pos_data[0, :] - 0.5) + 0.5
        pos_data[1, :] = scale * (pos_data[1, :] - 0.5) + 0.5
        self.mp.train(s_data, pos_data)
        return self

    def translate(self, translate: List[int]) -> 'MpTrajectory':
        tx = float(translate[0]) / self.im_width
        ty = float(translate[1]) / self.im_height
        s_data, pos_data = self.__generate_data()
        pos_data = pos_data + np.array([tx, ty]).reshape(-1, 1)
        self.mp.train(s_data, pos_data)
        return self

    def perspective(self, startpoints: List[List[float]], endpoints) -> 'MpTrajectory':
        """ Applies perspective transformation to 'self'.

        Args:
            startpoints: List[List[float]], normalized in [0, 1) corner start-points.
            endpoints: List[List[float]], normalized in [0, 1) corner end-points.

        Returns:
            self
        """

        # scale_points = lambda points: [temp.int().tolist() for temp in torch.as_tensor(points) * torch.tensor([[self.im_width, self.im_height]])]
        # startpoints, endpoints = map(scale_points, (startpoints, endpoints))

        # (x, y) -> ( (ax + by + c) / (gx + hy + 1), (dx + ey + f) / (gx + hy + 1) )
        a, b, c, d, e, f, g, h = self._get_perspective_coeffs(startpoints, endpoints)
        s_data, pos_data = self.__generate_data()
        # pos_data = pos_data * np.array([self.im_width, self.im_height]).reshape(2,1)
        new_pos_data = np.dot(np.array([[a, b], [d, e]]), pos_data) + np.array([c, f]).reshape(2, 1)
        denom = np.dot(np.array([g, h]).reshape(1, 2), pos_data) + 1
        pos_data = new_pos_data / denom
        # pos_data = pos_data / np.array([self.im_width, self.im_height]).reshape(2,1)
        self.mp.train(s_data, pos_data)
        return self

    # =========== Private utility functions ==============

    @staticmethod
    def _get_perspective_coeffs(startpoints: List[List[int]], endpoints: List[List[int]]) -> List[float]:
        """ Calculates the Perspective Transform coefficienys
        For each pixel (x, y) the transform is
        (x, y) -> ( (ax + by + c) / (gx + hy + 1), (dx + ey + f) / (gx + hy + 1) )
        Returns:
            octuple (a, b, c, d, e, f, g, h) for transforming each pixel.
        """
        a_matrix = torch.zeros(2 * len(startpoints), 8, dtype=torch.float)

        for i, (p1, p2) in enumerate(zip(startpoints, endpoints)):
            a_matrix[2 * i, :] = torch.tensor([p1[0], p1[1], 1, 0, 0, 0, -p2[0] * p1[0], -p2[0] * p1[1]])
            a_matrix[2 * i + 1, :] = torch.tensor([0, 0, 0, p1[0], p1[1], 1, -p2[1] * p1[0], -p2[1] * p1[1]])

        b_matrix = torch.tensor(endpoints, dtype=torch.float).view(8)
        res = torch.linalg.lstsq(a_matrix, b_matrix, driver='gels').solution

        output: List[float] = res.tolist()
        return output

    def __generate_data(self, n_points=200):
        s_data = np.linspace(0, 1, n_points)
        pos_data = np.hstack([self.mp.get_pos(s) for s in s_data])
        return s_data, pos_data

    def __get_new_trajectory(self, s_data, pos_data) -> 'MpTrajectory':
        mp2 = self.mp.deep_copy()
        mp2.train(s_data, pos_data)
        return MpTrajectory(mp2.weights)


class SegmentationMask(BaseDataType):

    data_name = 'seg_mask'
    filename = 'seg_mask.png'

    def __init__(self, seg_mask: Union[np.array, PIL.Image.Image]):
        super().__init__()
        self.data = torch.tensor(np.asarray(seg_mask), dtype=torch.long)[None, ...] # add 1 channel

        self.transforms = {
            'hflip': self.hflip,
            'center_crop': self.center_crop,
            'resize': self.resize,
            'pad': self.pad,
            'rotate': self.rotate,
            'translate': self.translate,
            'scale': self.scale,
            'perspective': self.perspective,
        }

    @classmethod
    def empty(cls, size):
        return cls(-np.ones(size))

    @classmethod
    def load(cls, path, filename=None) -> 'SegmentationMask':
        if filename is None:
            filename = cls.filename
        filename = os.path.join(path, filename)
        if not os.path.exists(filename):
            return None
        return cls(PIL.Image.open(filename))

    def save(self, path, filename=None):
        if filename is None:
            filename = self.filename
        torchvision_F.to_pil_image(self.data.type(torch.int)).save(os.path.join(path, filename), format='png')
    
    def numpy(self) -> np.array:
        return self.torch_tensor().numpy()

    def torch_tensor(self) -> torch.Tensor:
        return self.data[0]

    def size(self) -> List[int]:
        return list(self.data.shape)

    # =========== Transforms ==============

    def hflip(self) -> 'SegmentationMask':
        self.data = torchvision_F.hflip(self.data)
        return self

    def center_crop(self, size) -> 'SegmentationMask':
        self.data = torchvision_F.center_crop(self.data, size)
        return self

    def resize(self, size, not_used) -> 'SegmentationMask':
        self.data = torchvision_F.resize(self.data, size, torchvision_T.InterpolationMode.NEAREST)
        return self

    def pad(self, padding, fill=0) -> 'SegmentationMask':
        self.data = torchvision_F.pad(self.data, padding, fill)
        return self

    def rotate(self, angle) -> 'SegmentationMask':
        self.data = torchvision_F.affine(img=self.data, angle=angle, translate=[0, 0], scale=1.0, shear=0, fill=0.0,
                                   interpolation=torchvision_T.InterpolationMode.NEAREST)
        return self

    def translate(self, translate) -> 'SegmentationMask':
        self.data = torchvision_F.affine(img=self.data, angle=0, translate=translate, scale=1.0, shear=0, fill=0.0,
                                   interpolation=torchvision_T.InterpolationMode.NEAREST)
        return self

    def scale(self, scale) -> 'SegmentationMask':
        self.data = torchvision_F.affine(img=self.data, angle=0, translate=[0, 0], scale=scale, shear=0, fill=0.0,
                                   interpolation=torchvision_T.InterpolationMode.NEAREST)
        return self

    def perspective(self, startpoints: List[List[float]], endpoints: List[List[float]], fill=0) -> 'SegmentationMask':
        """ Applies perspective transformation to 'self'.

        Args:
            startpoints: List[List[float]], normalized in [0, 1) corner start-points.
            endpoints: List[List[float]], normalized in [0, 1) corner end-points.

        Returns:
            self
        """
        h, w = self.size()[-2:]
        scale_points = lambda points: [temp.int().tolist() for temp in torch.as_tensor(points) * torch.tensor([[w, h]])]
        startpoints, endpoints = map(scale_points, (startpoints, endpoints))
        self.data = torchvision_F.perspective(self.data, startpoints, endpoints, torchvision_T.InterpolationMode.NEAREST, fill=fill)
        return self


class ROIMask(SegmentationMask):

    data_name = 'mask'
    filename = 'mask.png'

    def __init__(self, roi_mask: Union[np.array, PIL.Image.Image]):
        super().__init__([])
        self.data = torch.tensor(np.asarray(roi_mask), dtype=torch.uint8)[None, ...] # add 1 channel
    
    @classmethod
    def load(cls, path, filename=None) -> 'ROIMask':
        if filename is None:
            filename = cls.filename
        filename = os.path.join(path, filename)
        if not os.path.exists(filename):
            return None
        mask = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        if mask.ndim == 3: mask = mask[..., 0]
        return cls(mask)

    def save(self, path, filename=None):
        if filename is None:
            filename = self.filename
        cv2.imwrite(os.path.join(path, filename), self.data[0].numpy())


class DepthImage(BaseDataType):

    data_name = 'depth'
    filename = 'depth.png'

    def __init__(self, depth_img: Union[np.array, PIL.Image.Image]):
        super().__init__()
        self.data = torch.tensor(np.asarray(depth_img), dtype=torch.long)[None, ...] # add 1 channel

        self.transforms = {
            'hflip': self.hflip,
            'center_crop': self.center_crop,
            'resize': self.resize,
            'pad': self.pad,
            'rotate': self.rotate,
            'translate': self.translate,
            'scale': self.scale,
            # 'perspective': self.perspective,
        }

    @classmethod
    def load(cls, path, filename=None) -> 'DepthImage':
        if filename is None:
            filename = cls.filename
        filename = os.path.join(path, filename)
        if not os.path.exists(filename):
            return None
        # return cls(PIL.Image.open(filename))
        depth_img = cv2.imread(filename, cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000
        return cls(depth_img)

    def save(self, path, filename=None):
        if filename is None:
            filename = self.filename
            depth_img = (self.numpy() * 1000).astype(np.uint16)
        if not cv2.imwrite(os.path.join(path, filename), depth_img):
            raise RuntimeError('Failed to save DepthImage...')
    
    def numpy(self) -> np.array:
        return self.torch_tensor().numpy()

    def torch_tensor(self) -> torch.Tensor:
        return self.data[0]

    def size(self) -> List[int]:
        return list(self.data.shape)

    # =========== Transforms ==============

    def hflip(self) -> 'DepthImage':
        self.data = torchvision_F.hflip(self.data)
        return self

    def center_crop(self, size) -> 'DepthImage':
        self.data = torchvision_F.center_crop(self.data, size)
        return self

    def resize(self, size) -> 'DepthImage':
        self.data = torchvision_F.resize(self.data, size, torchvision_T.InterpolationMode.NEAREST)
        return self

    def pad(self, padding) -> 'DepthImage':
        self.data = torchvision_F.pad(self.data, padding, fill=0)
        return self

    def rotate(self, angle) -> 'DepthImage':
        self.data = torchvision_F.affine(img=self.data, angle=angle, translate=[0, 0], scale=1.0, shear=0, fill=0.0,
                                   interpolation=torchvision_T.InterpolationMode.NEAREST)
        return self

    def translate(self, translate) -> 'DepthImage':
        self.data = torchvision_F.affine(img=self.data, angle=0, translate=translate, scale=1.0, shear=0, fill=0.0,
                                   interpolation=torchvision_T.InterpolationMode.NEAREST)
        return self

    def scale(self, scale) -> 'DepthImage':
        self.data = torchvision_F.affine(img=self.data, angle=0, translate=[0, 0], scale=scale, shear=0, fill=0.0,
                                   interpolation=torchvision_T.InterpolationMode.NEAREST)
        return self


class BaseRGBimage(BaseDataType):

    data_name = None
    filename = None

    def __init__(self, img: Union[np.array, PIL.Image.Image]):
        super().__init__()
        self.data = torchvision_F.to_tensor(img)

        # supported (implemented) transforms
        self.transforms = {
            'hflip': self.hflip,
            'center_crop': self.center_crop,
            'resize': self.resize,
            'pad': self.pad,
            'rotate': self.rotate,
            'translate': self.translate,
            'scale': self.scale,
            'perspective': self.perspective,
        }

    @classmethod
    def load(cls, path, filename=None) -> 'BaseRGBimage':
        if filename is None:
            filename = cls.filename
        filename = os.path.join(path, filename)
        if not os.path.exists(filename):
            return None
        return cls(PIL.Image.open(filename))

    def save(self, path, filename=None):
        if filename is None:
            filename = self.filename
        torchvision_F.to_pil_image(self.data).save(os.path.join(path, filename), format='png')

    def numpy(self) -> np.array:
        return self.data.permute(1, 2, 0).numpy()

    def torch_tensor(self) -> torch.Tensor:
        return self.data

    def size(self) -> List[int]:
        return list(self.data.shape)

    # =========== Transforms ==============

    def hflip(self) -> 'BaseRGBimage':
        self.data = torchvision_F.hflip(self.data)
        return self

    def center_crop(self, size) -> 'BaseRGBimage':
        self.data = torchvision_F.center_crop(self.data, size)
        return self

    def resize(self, size, interpolation) -> 'BaseRGBimage':
        self.data = torchvision_F.resize(self.data, size, interpolation)
        return self

    def pad(self, padding, fill) -> 'BaseRGBimage':
        self.data = torchvision_F.pad(self.data, padding, fill)
        return self

    def rotate(self, angle, fill=0.0) -> 'BaseRGBimage':
        self.data = torchvision_F.affine(img=self.data, angle=angle, translate=[0, 0], scale=1.0, shear=0, fill=fill,
                                   interpolation=torchvision_T.InterpolationMode.NEAREST)
        return self

    def translate(self, translate, fill=0.0) -> 'BaseRGBimage':
        self.data = torchvision_F.affine(img=self.data, angle=0, translate=translate, scale=1.0, shear=0, fill=fill,
                                   interpolation=torchvision_T.InterpolationMode.NEAREST)
        return self

    def scale(self, scale, fill=0.0) -> 'BaseRGBimage':
        self.data = torchvision_F.affine(img=self.data, angle=0, translate=[0, 0], scale=scale, shear=0, fill=fill,
                                   interpolation=torchvision_T.InterpolationMode.NEAREST)
        return self

    def perspective(self, startpoints: List[List[float]], endpoints: List[List[float]], fill=0.0) -> 'BaseRGBimage':
        """ Applies perspective transformation to 'self'.

        Args:
            startpoints: List[List[float]], normalized in [0, 1) corner start-points.
            endpoints: List[List[float]], normalized in [0, 1) corner end-points.

        Returns:
            self
        """
        h, w = self.size()[-2:]
        scale_points = lambda points: [temp.int().tolist() for temp in torch.as_tensor(points) * torch.tensor([[w, h]])]
        startpoints, endpoints = map(scale_points, (startpoints, endpoints))
        self.data = torchvision_F.perspective(self.data, startpoints, endpoints, torchvision_T.InterpolationMode.NEAREST, fill=fill)
        return self

# ========= Specific Types ===========

class InputImage(BaseRGBimage):
    data_name = 'rgb'
    filename = 'rgb.png'

    def __init__(self, img: Union[np.array, PIL.Image.Image]):
        img = np.array(img)[..., 0:3] # make sure only 3 channels are read
        super().__init__(img)

        # add extra supported transforms
        self.transforms.update({
            'grayscale': self.grayscale,
            'grayscale_to_rgb': self.grayscale_to_rgb,
            'normalize': self.normalize,
            'color_jitter': self.color_jitter,
            'gaussian_noise': self.gaussian_noise,
        })

    def normalize(self, mean, std) -> 'InputImage':
        self.data = torchvision_F.normalize(self.data, mean=mean, std=std)
        return self

    def grayscale(self) -> 'InputImage':
        self.data = torchvision_T.Grayscale()(self.data)
        return self

    def grayscale_to_rgb(self) -> 'InputImage':
        self.data = self.data.repeat(3, 1, 1)
        return self

    def color_jitter(self, brightness=0, contrast=0, saturation=0, hue=0):
        self.data = torchvision_T.ColorJitter(brightness=brightness, contrast=contrast,
                                              saturation=saturation, hue=hue)(self.data)
        return self

    def gaussian_noise(self, mean, std) -> 'InputImage':
        self.data = self.data + torch.randn(self.data.size()) * std + mean
        self.data = torch.clip(self.data, 0.0, 1.0)
        return self


class BeforeImage(BaseRGBimage):
    data_name = 'before_rgb'
    filename = 'before.png'


class AfterImage(BaseRGBimage):
    data_name = 'after_rgb'
    filename = 'after.png'


class SegImage(BaseRGBimage):
    data_name = 'seg_image'
    filename = 'seg_image.png'


