import torchvision.utils
import cv2
from typing import Dict, List, Union, Callable
import copy
import os
import torchvision.transforms as torchvision_T
import my_pkg.data_transforms as data_tf
from my_pkg.data_types import *
from my_pkg.util.gmp import MovementPrimitive

import torch
import torchvision
import numpy as np
import pickle
import abc
from typing import List

__all__ = [
    "ResNetDMP",
    "ResNetDMP_loss",
    "ResNetDMP_2D_OrientTraj_loss",
    "ImgMaskDMP_net",
    "ImgMaskDMP_net_loss",
    "VIMEDNet",
    "VIMEDNet_loss",
    "ResNetMDDMP",
    "ResNetMDDMP_loss",
    "load_model",
    "mp_weights_to_traj",
    "ResnetSegmentation",
]


_tf_params = {
    'crop': 480,
    'resize': 224,
    'mean': np.array([0.0, 0.0, 0.0]),  # np.array([0.485, 0.456, 0.406])
    'std': np.array([1.0, 1.0, 1.0]),  # np.array([0.229, 0.224, 0.225])
    'fill': 0.9,
}
_tf_params['pad'] = (640 - _tf_params['crop']) / 2


def mp_weights_to_traj(W: torch.Tensor, duration: float, time_step: float, img_size=[1., 1.]) -> torch.Tensor:
    
    device = W.device
    dtype = W.dtype
    n_dofs = W.shape[1]
    n_kernels = W.shape[2]

    mp = MovementPrimitive(n_dofs=1, n_kernels=n_kernels)
    s_data = np.linspace(0, 1, int(duration / time_step + 0.5))
    Phi_data = torch.hstack([torch.tensor(mp.regress_vec(s)) for s in s_data]).to(device=device, dtype=dtype)

    # traj = torch.matmul(W.view(-1, n_kernels), Phi_data).view(W.shape[0], n_dofs, len(s_data)) # (batch_size, n_dofs, n_points)
    traj = torch.matmul(W, Phi_data) # (batch_size, n_dofs, n_points)

    h, w = img_size
    if h != 1. or w != 1.:
        traj[:, 0:2, :] = traj[:, 0:2, :] * torch.tensor([w, h], dtype=dtype , device=device).view(1, 2, 1)

    return traj


# =====================================
# ============  MDN Head  =============
# =====================================

class MDNHead(torch.nn.Module):

    def __init__(self, n_gaussians, n_in: int, n_out: int, weights_init: Callable = None,
                sigma_gating: torch.nn.Module = torch.nn.Softplus()) -> None:
        super().__init__()

        self.w_init_fun = weights_init

        self.n_gaussians = n_gaussians
        self.n_out = n_out

        self.frac_net = torch.nn.Sequential(
            torch.nn.Linear(in_features=n_in, out_features=n_gaussians, bias=True),
            torch.nn.Softmax(dim=1)
        )
        self.init_weights(self.frac_net)

        self.mu_net = torch.nn.Linear(in_features=n_in, out_features=n_out*n_gaussians, bias=True)
        self.init_weights(self.mu_net)

        self.sigma_net = torch.nn.Sequential(
            torch.nn.Linear(in_features=n_in, out_features=n_gaussians, bias=True),
            sigma_gating # torch.nn.Softplus(), torch.nn.ELU(), torch.exp
        )
        self.init_weights(self.sigma_net)
        
    def forward(self, x: torch.Tensor) -> Dict[str, torch.Tensor]:

        return {'fracs': self.frac_net(x), #.view(-1, 1, self.n_gaussians),
                'means': self.mu_net(x).view(-1, self.n_gaussians, self.n_out), 
                'sigma': self.sigma_net(x)} #.view(-1, 1, self.n_gaussians)}

    def init_weights(self, m: torch.nn.Module):
        
        if isinstance(m, torch.nn.Sequential):
            for m_ in m:
                self.init_weights(m_)

        if isinstance(m, torch.nn.Linear):
            if self.w_init_fun is not None:
               m.weight = self.w_init_fun(m.weight)
            # torch.nn.init.xavier_normal_(m.bias)
    
    @staticmethod
    def loss_fun(gmm_params: Dict[str, torch.Tensor], y: torch.Tensor) -> torch.Tensor:
        fracs, means, sigma = gmm_params['fracs'], gmm_params['means'], gmm_params['sigma']

        dist = (y.unsqueeze(dim=1) - means) / sigma[..., None, None] #.view(means.shape[0], -1, 1)
        squared_dist =  - 0.5 * torch.sum(torch.square(dist), dim=[2, 3], keepdim=False)
        log_sum_exp = -torch.logsumexp(torch.log(fracs)+squared_dist-0.5*torch.log(2*np.pi*torch.square(sigma)), dim=1)
        loss = torch.mean(log_sum_exp) + 0.1*torch.mean(torch.square(sigma))

        # print('dist: ', dist.shape)
        # print('squared_dist: ', squared_dist.shape)
        # print('log_sum_exp: ', log_sum_exp.shape)
        # input('')

        if torch.any(torch.isnan(loss)):
            print('\33[1;33m==== Nan detected! ====\33[0m')
            # print('fracs: ', fracs.shape, ' = ', fracs)
            # print('sigma: ', sigma.shape, ' = ', sigma)
            # print('loss: ', loss.shape, ' = ', loss)
            # print('squared_dist: ', squared_dist.shape, ' = ', squared_dist)
            # print('log_sum_exp: ', log_sum_exp.shape, ' = ', log_sum_exp)
            # input('')

        return loss


# =======================================
# ============  Base Model  =============
# =======================================

class BaseModel(torch.nn.Module):

    def __init__(self):
        super().__init__()
        self._config = {}
        self.train_history = []

    def config_state_dict(self):
        return {'config': self._config, 'state_dict': self.state_dict(),
                'class_name': self.__class__.__name__,
                'train_history': self.train_history}

    @classmethod
    def from_config_state_dict(cls, s):
        class_name = s.pop('class_name', None)
        if not class_name:
            raise RuntimeError('Failed to load class name...')
        if class_name != cls.__name__:
            raise RuntimeError(f"Loaded class {class_name} != from called class {cls.__name__}")

        model = cls(**s['config'])
        model.load_state_dict(s['state_dict'])
        model.train_history = s['train_history']
        return model

    def save(self, filename: str):
        path = '/'.join(filename.split('/')[:-1])
        if path and not os.path.exists(path): os.makedirs(path)
        pickle.dump(self.config_state_dict(), open(filename, 'wb'))

    @classmethod
    def load(cls, filename):
        return cls.from_config_state_dict(pickle.load(open(filename, 'rb')))

    def _init_config(self, locals_dict):
        self._config = locals_dict
        self._config.pop('self')
        self._config.pop('__class__')


    @staticmethod
    @abc.abstractmethod
    def output_to_traj(net_output: Dict[str, torch.Tensor], duration=5.0, time_step=0.05, img_size=[1., 1.]) -> torch.Tensor:
        """
        Converts the networks output to a trajectory. The trajectory axes are scaled according to the img_size.

        Arguments:
        net_ouput -- torch.Tensor(batch_size, ...), the network's output.
        duration -- float, the time duration of the generated trajectory.
        time_step -- float, the time-step between consevutive trajectory points. 
        img_size -- Tuple[int, int] with the (height, width) of the image. (default=[1., 1.], i.e. no scaling)

        Returns:
        torch.Tensor(batch_size, n_dofs, n_points), the generated trajectory.
        """
        pass

    @staticmethod
    @abc.abstractmethod
    def input_transform(x):
        pass

    @staticmethod
    @abc.abstractmethod
    def inv_input_transform(x):
        pass


# --------- Base Loss --------

class MPTraj_loss(torch.nn.Module):

    def __init__(self) -> None:
        super().__init__()
        self._Phi_data = None
        self._Phi_target_data = None

    def _set_phi(self, n_kernels: int, target_n_kernels: int, device=None, dtype=None):

        if self._Phi_data is None or self._Phi_data.shape[0] != n_kernels:
            mp = MovementPrimitive(n_dofs=1, n_kernels=n_kernels)
            self._Phi_data = torch.hstack([torch.tensor(mp.regress_vec(s)) for s in np.linspace(0, 1, 50)]).to(dtype=torch.float32)

        if self._Phi_target_data is None or self._Phi_target_data.shape[0] != n_kernels:
            mp = MovementPrimitive(n_dofs=1, n_kernels=target_n_kernels)
            self._Phi_target_data = torch.hstack([torch.tensor(mp.regress_vec(s)) for s in np.linspace(0, 1, 50)]).to(dtype=torch.float32)

        self._Phi_data = self._Phi_data.to(device=device, dtype=dtype)
        self._Phi_target_data = self._Phi_target_data.to(device=device, dtype=dtype)

# ======================================
# ============  ResNetDMP  =============
# ======================================

class ResNetDMP(BaseModel):

    global _tf_params

    __dataset_transforms = [
        data_tf.CenterCrop(_tf_params['crop']),
        data_tf.Resize(_tf_params['resize']),
        data_tf.Normalize(_tf_params['mean'], _tf_params['std'])
    ]

    __input_transforms = torchvision.transforms.Compose([
        torchvision_T.CenterCrop(_tf_params['crop']),
        torchvision_T.Resize(_tf_params['resize']),
        torchvision_T.Normalize(_tf_params['mean'], _tf_params['std'])
    ])

    __inv_input_transforms = torchvision.transforms.Compose([
        torchvision_T.Normalize(-_tf_params['mean'] / _tf_params['std'], 1 / _tf_params['std']),
        torchvision_T.Resize(_tf_params['crop']),
        torchvision_T.Pad([_tf_params['pad'], 0], fill=_tf_params['fill']),
    ])

    def __init__(self, n_dofs: int, mp_kernels: int, resnet_type='resnet18', in_channels=3, backbone_trainable=True, pretrained=False):
        super().__init__()
        self._init_config(locals())

        self.mp_kernels = mp_kernels
        self.n_dofs = n_dofs
        self.out_params = self.n_dofs * self.mp_kernels

        if resnet_type == 'resnet18':
            create_resnet = torchvision.models.resnet18
        elif resnet_type == 'resnet50':
            create_resnet = torchvision.models.resnet50
        else:
            raise RuntimeError(f'Unsupported resnet type "{resnet_type}"...')

        self.backbone = create_resnet(pretrained=pretrained, progress=False)

        if in_channels != 3:
            w1 = self.backbone.conv1.weight.clone()
            c1 = self.backbone.conv1
            self.backbone.conv1 = torch.nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3,bias=False)
            # self.backbone.conv1 = torch.nn.Conv2d(in_channels, self.backbone.inplanes, kernel_size=c1.kernel_size,
            #                                       stride=c1.stride, padding=c1.padding, bias=c1.bias)
            with torch.no_grad():
                self.backbone.conv1.weight[:, :3] = w1
                self.backbone.conv1.weight[:, 3] = self.backbone.conv1.weight[:, 0]

        if not backbone_trainable:
            for param in self.backbone.parameters():
                param.requires_grad = False

        self.bb_features = self.backbone.fc.in_features
        self.dmp_nn = torch.nn.Linear(self.bb_features, self.out_params)
        self.backbone.fc = torch.nn.Flatten(start_dim=1)

        self.backbone_out_layers = {}
        self.backbone_out_layers['fc'] = 'dmp_features'

        from torchvision.models._utils import IntermediateLayerGetter
        self.backbone = IntermediateLayerGetter(self.backbone, return_layers=self.backbone_out_layers)

    @staticmethod
    def input_output_transforms():
        return ResNetDMP.__dataset_transforms

    @staticmethod
    def input_transform(x: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        x['rgb'] = ResNetDMP.__input_transforms(x['rgb'])
        return x

    @staticmethod
    def inv_input_transform(x: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        x['rgb'] = ResNetDMP.__inv_input_transforms(x['rgb'])
        return x

    @staticmethod
    def output_to_traj(net_output: Dict[str, torch.Tensor], duration=5.0, time_step=0.05, img_size=[1., 1.]) -> torch.Tensor:
        return mp_weights_to_traj(net_output['mp_weights'], duration, time_step, img_size)

    def forward(self, x: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        x = x['rgb']
        bb_out = self.backbone(x)
        out = {}
        out['mp_weights'] = self.dmp_nn(bb_out['dmp_features']).view(-1, self.n_dofs, self.mp_kernels)

        return out

# -------------  Loss  ----------------
class ResNetDMP_loss(MPTraj_loss):

    def __init__(self, criterion: torch.nn.SmoothL1Loss(reduction='mean', beta=0.5)) -> None:
        super().__init__()
        self.criterion = criterion

    def forward(self, net_out: Dict[str, torch.Tensor], targets: torch.Tensor):

        pred_weights = net_out['mp_weights']
        target_weights = targets['mp_weights']

        self._set_phi(n_kernels=pred_weights.shape[2], target_n_kernels=target_weights.shape[2], device=target_weights.device, dtype=target_weights.dtype)

        y_pred = torch.matmul(pred_weights, self._Phi_data) # (batch_size, n_dofs, n_points)
        y_target = torch.matmul(target_weights, self._Phi_target_data) # (batch_size, n_dofs, n_points)
        
        return self.criterion(y_pred, y_target)


class ResNetDMP_2D_OrientTraj_loss(MPTraj_loss):

    def __init__(self, criterion: torch.nn.SmoothL1Loss(reduction='mean', beta=0.5)) -> None:
        super().__init__()
        self.criterion = criterion

    def forward(self, net_out: Dict[str, torch.Tensor], targets: torch.Tensor):

        pred_weights = net_out['mp_weights']
        target_weights = targets['mp_weights']

        self._set_phi(n_kernels=pred_weights.shape[2], target_n_kernels=target_weights.shape[2], device=target_weights.device, dtype=target_weights.dtype)

        y_pred = torch.matmul(pred_weights, self._Phi_data) # (batch_size, n_dofs, n_points)
        y_target = torch.matmul(target_weights, self._Phi_target_data) # (batch_size, n_dofs, n_points)

        y_pred[:, 2, :] = y_pred[:, 2, :] / (2*np.pi) # normalize rotation
        y_target[:, 2, :] = y_target[:, 2, :] / (2*np.pi) # normalize rotation
        
        return self.criterion(y_pred, y_target)


# ========================================
# ============  ImgMaskDMP_net  =============
# ========================================

class ImgMaskDMP_net(ResNetDMP):

    __input_img_transforms = torchvision.transforms.Compose([
        torchvision_T.CenterCrop(_tf_params['crop']),
        torchvision_T.Resize(_tf_params['resize']),
        torchvision_T.Normalize(_tf_params['mean'], _tf_params['std'])
    ])

    __input_mask_transforms = torchvision.transforms.Compose([
        torchvision_T.CenterCrop(_tf_params['crop'], ),
        torchvision_T.Resize(_tf_params['resize'], interpolation=torchvision_T.InterpolationMode.NEAREST),
    ])

    def __init__(self, **kwargs):
        kwargs['in_channels'] = 4
        super().__init__(**kwargs)

    def forward(self, x: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        roi_mask = x['mask'][:, None, ...]/255.0
        masked_img =  x['rgb'] * (1 - roi_mask)
        x = torch.concat([masked_img, roi_mask], dim=1)
        return ResNetDMP.forward(self, {'rgb': x})

    
    @staticmethod
    def input_transform(x: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        x_in = {}
        x_in['rgb'] = ImgMaskDMP_net.__input_img_transforms(x['rgb'])
        x_in['mask'] = ImgMaskDMP_net.__input_mask_transforms(x['mask'])
        return x_in

# -------------  Loss  ----------------
class ImgMaskDMP_net_loss(ResNetDMP_2D_OrientTraj_loss):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)


# ========================================
# ============  ResNetMDDMP  =============
# ========================================

class ResNetMDDMP(ResNetDMP):

    def __init__(self, n_gaussians: int, **kwargs):
        super().__init__(**kwargs)
        self._config['n_gaussians'] = n_gaussians
        self.mdn = MDNHead(n_gaussians, n_in=self.bb_features, n_out=self.out_params)

    def forward(self, x: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        x = x['rgb']
        bb_out = self.backbone(x)
        mdn_out = self.mdn(bb_out['dmp_features'])
        mdn_out['means'] = mdn_out['means'].view(-1, self.mdn.n_gaussians, self.n_dofs, self.mp_kernels)
        mdn_out['mp_weights'] = self.get_kth_weights(mdn_out, k=1)

        return mdn_out

    @staticmethod
    def output_to_traj(net_output: Dict[str, torch.Tensor], duration=5.0, time_step=0.05, img_size=[1., 1.], k=1) -> torch.Tensor:

        return mp_weights_to_traj(ResNetMDDMP.get_kth_weights(net_output, k), duration, time_step, img_size)
    
    @staticmethod
    def get_kth_weights(net_output: Dict[str, torch.Tensor], k) -> torch.Tensor:
        
        # indices = torch.argmax(net_output['fracs'], dim=1, keepdim=False)
        indices = torch.kthvalue(-net_output['fracs'], k, dim=1, keepdim=False)[1]
        w_means = net_output['means']
        kth_weights = torch.stack([w_means[i, idx, :, :] for i, idx in enumerate(indices)], dim=0)
        return kth_weights


# -------------  Loss  ----------------
class ResNetMDDMP_loss(MPTraj_loss):

    def __init__(self) -> None:
        super().__init__()

    def forward(self, net_out: Dict[str, torch.Tensor], targets: Dict[str, torch.Tensor]):

        target_weights = targets['mp_weights']
        frac, w_means, w_sigma = net_out['fracs'], net_out['means'], net_out['sigma']

        self._set_phi(n_kernels=w_means.shape[3], target_n_kernels=target_weights.shape[2], device=target_weights.device, dtype=target_weights.dtype)

        y_target = torch.matmul(target_weights, self._Phi_target_data) # (batch_size, n_dofs, n_points)
        traj_means = torch.matmul(w_means, self._Phi_data) # (batch_size, n_components, n_dofs, n_points)
        traj_sigma = w_sigma
        traj_mixture_params = {'fracs': frac, 'means': traj_means, 'sigma': traj_sigma }

        # print('y_target: ', y_target.shape)
        # print('traj_means: ', traj_means.shape)
        # print('traj_sigma: ', traj_sigma.shape)

        return MDNHead.loss_fun(gmm_params=traj_mixture_params, y=y_target)

# =====================================
# ============  VIMEDNet  =============
# =====================================


class VIMEDNet(BaseModel):

    global _tf_params

    __dataset_transforms = [
        data_tf.CenterCrop(_tf_params['crop']),
        data_tf.Resize(_tf_params['resize']),
        data_tf.Grayscale()
    ]

    __input_transforms = torchvision.transforms.Compose([
        torchvision_T.CenterCrop(_tf_params['crop']),
        torchvision_T.Resize(_tf_params['resize']),
        torchvision_T.Grayscale()
    ])

    __inv_input_transforms = torchvision.transforms.Compose([
        torchvision_T.Resize(_tf_params['crop']),
        torchvision_T.Pad([_tf_params['pad'], 0], fill=_tf_params['fill']),
        torchvision_T.Lambda(lambda x: x.repear(1, 3, 1, 1)),
    ])

    def __init__(self, n_dofs: int, mp_kernels: int):
        super().__init__()
        self._init_config(locals())

        self.mp_kernels = mp_kernels
        self.n_dofs = n_dofs

        self.conv1 = torch.nn.Conv2d(in_channels=1, out_channels=5, kernel_size=5, stride=1, padding=2)
        self.maxpool1 = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)

        self.conv2 = torch.nn.Conv2d(in_channels=5, out_channels=10, kernel_size=5, stride=1, padding=2)
        self.maxpool2 = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)

        self.conv3 = torch.nn.Conv2d(in_channels=10, out_channels=20, kernel_size=5, stride=1, padding=2)
        self.maxpool3 = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)

        self.conv4 = torch.nn.Conv2d(in_channels=20, out_channels=35, kernel_size=30, stride=1, padding=14)

        self.global_max_pool = lambda x: torch.nn.functional.max_pool2d(x, kernel_size=x.size()[2:])

        # self.activ = torch.tanh
        self.activ = torch.nn.functional.relu

        self.fc1 = torch.nn.Linear(in_features=35, out_features=40)
        self.fc2 = torch.nn.Linear(in_features=40, out_features=45)
        self.fc3 = torch.nn.Linear(in_features=45, out_features=self.n_dofs * self.mp_kernels)

    @staticmethod
    def input_output_transforms():
        return VIMEDNet.__dataset_transforms

    @staticmethod
    def input_transform(x: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        x['rgb'] = VIMEDNet.__input_transforms(x['rgb'])
        return x

    @staticmethod
    def inv_input_transform(x: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        x['rgb'] = VIMEDNet.__inv_input_transforms(x['rgb'])
        return x

    @staticmethod
    def output_to_traj(net_output: Dict[str, torch.Tensor], duration=5.0, time_step=0.05, img_size=[1, 1]) -> torch.Tensor:
        return mp_weights_to_traj(net_output['mp_weights'], duration, time_step, img_size)

    def forward(self, x: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        x = x['rgb']
        x = self.activ(self.conv1(x))
        x = self.maxpool1(x)

        x = self.activ(self.conv2(x))
        x = self.maxpool2(x)

        x = self.activ(self.conv3(x))
        x = self.maxpool3(x)

        x = self.activ(self.conv4(x))

        x = torch.flatten(self.global_max_pool(x), 1)

        x = self.activ(self.fc1(x))
        x = self.activ(self.fc2(x))
        x = self.fc3(x)

        return {'mp_weights': x.view(-1, self.n_dofs, self.mp_kernels)}

# -------------  Loss  ----------------
class VIMEDNet_loss(ResNetDMP_loss):

    def __init__(self, criterion: torch.nn.SmoothL1Loss(reduction='mean', beta=0.5)) -> None:
        super().__init__(criterion)


# ================================================
# ============  Segmentation Models  =============
# ================================================

class FCNClassifier(torch.nn.Module):
    def __init__(self, in_channels, n_classes):
        super().__init__()
        self.classifier = torchvision.models.segmentation.fcn.FCNHead(in_channels, n_classes)
        self.backbone_layer = 'layer4'

    def forward(self, x):
        return self.classifier(x[self.backbone_layer])


class FCNUpsampleBlock(torch.nn.Module):

    def __init__(self, in_channels: int, out_channels: int):

        super().__init__()

        up_rate = 2
        self.upsample = torch.nn.ConvTranspose2d(
            out_channels, out_channels, kernel_size=3, stride=up_rate, padding=1, output_padding=up_rate-1)
        self.conv = torch.nn.Conv2d(
            in_channels=in_channels + out_channels, out_channels=out_channels, kernel_size=1, stride=1)

    def forward(self, x, skip_connection):
        x = self.conv(torch.concat([self.upsample(x), skip_connection], dim=1))
        return x


class FCNSegmentation(torch.nn.Module):
    def __init__(self, n_classes, type_=32, backbone_model='resnet50'):
        super().__init__()

        if type_ not in (32, 16, 8, 4):
            raise ValueError(f"Input type '{type_}' is not in (32, 16, 8, 4)")
        self.type_ = type_

        self.backbone_layers = []

        if backbone_model == 'resnet50':
            upsample_in_channels = [1024, 512, 256]
        elif backbone_model == 'resnet18':
            upsample_in_channels = [256, 128, 64]
        else:
            raise RuntimeError(f'Unsupported backbone model "{backbone_model}"...')

        if self.type_ < 32:
            self.upsample_x2 = FCNUpsampleBlock(in_channels=upsample_in_channels[0], out_channels=n_classes)
            self.backbone_layers.append("layer3")

        if self.type_ < 16:
            self.upsample_x4 = FCNUpsampleBlock(in_channels=upsample_in_channels[1], out_channels=n_classes)
            self.backbone_layers.append("layer2")

        if self.type_ < 8:
            self.upsample_x8 = FCNUpsampleBlock(in_channels=upsample_in_channels[2], out_channels=n_classes)
            self.backbone_layers.append("layer1")

    def forward(self, classifier_output, backbone_layers, input_shape):

        x = classifier_output
        if self.type_ < 32:
            x = self.upsample_x2(x, backbone_layers['layer3'])

        if self.type_ < 16:
            x = self.upsample_x4(x, backbone_layers['layer2'])

        if self.type_ < 8:
            x = self.upsample_x8(x, backbone_layers['layer1'])

        x = torch.nn.functional.interpolate(x, size=input_shape, mode="bilinear", align_corners=False)

        return x


class ResnetSegmentation(torch.nn.Module):

    def __init__(self, n_classes: int, seg_type=4, backbone_trainable=True, resnet_type='resnet50'):
        super().__init__()
        self._config = {}
        self._init_config(locals())

        self.train_history = []

        self.n_classes = n_classes

        if resnet_type == 'resnet18':
            create_resnet = torchvision.models.resnet18
        elif resnet_type == 'resnet50':
            create_resnet = torchvision.models.resnet50
        else:
            raise RuntimeError(f'Unsupported resnet type "{resnet_type}"...')

        self.backbone = create_resnet(pretrained=False, progress=False)

        if not backbone_trainable:
            for param in self.backbone.parameters():
                param.requires_grad = False

        bb_features = self.backbone.fc.in_features
        self.backbone.fc = torch.nn.Flatten(start_dim=1)  # LambdaLayer(lambda x: torch.flatten(x,1))

        backbone_out_layers = {}
        self.classifier = FCNClassifier(bb_features, n_classes)
        self.segmentation = FCNSegmentation(n_classes, type_=seg_type, backbone_model=resnet_type)
        backbone_out_layers[self.classifier.backbone_layer] = self.classifier.backbone_layer
        for layer in self.segmentation.backbone_layers:
            backbone_out_layers[layer] = layer

        from torchvision.models._utils import IntermediateLayerGetter
        self.backbone = IntermediateLayerGetter(self.backbone, return_layers=backbone_out_layers)

    def _init_config(self, locals_dict):
        self._config = locals_dict
        self._config.pop('self')
        self._config.pop('__class__')

    def config_state_dict(self):
        return {'config': copy.deepcopy(self._config), 'state_dict': copy.deepcopy(self.state_dict()),
                'class_name': self.__class__.__name__,
                'train_history': self.train_history}

    @classmethod
    def from_config_state_dict(cls, s):
        class_name = s.pop('class_name', None)
        if not class_name:
            raise RuntimeError('Failed to load class name...')
        if class_name != cls.__name__:
            raise RuntimeError(f"Loaded class {class_name} != from called class {cls.__name__}")

        model = cls(**s['config'])
        model.load_state_dict(s['state_dict'])
        model.train_history = s['train_history']
        return model

    def save(self, filename):
        import pickle
        pickle.dump(self.config_state_dict(), open(filename, 'wb'))

    @classmethod
    def load(cls, filename):
        import pickle
        return cls.from_config_state_dict(pickle.load(open(filename, 'rb')))

    def forward(self, x):
        input_shape = x.shape[-2:]

        bb_out = self.backbone(x)

        out = {}
        x = self.classifier(bb_out)
        out['seg_mask'] = self.segmentation(x, bb_out, input_shape)

        return out

    def output(self, x, return_prob=False):

        seg_mask = self.forward(x)['seg_mask']
        out = torch.argmax(seg_mask, dim=1, keepdim=False)

        if return_prob:
            prob = torch.max(torch.nn.functional.softmax(seg_mask, dim=1), dim=1, keepdim=False)[0]
            out = (out, prob)

        return out

# ==========================================
# ==========================================


def load_model(filename: str):

    s = pickle.load(open(filename, 'rb'))

    class_name = s.pop('class_name', None)
    if not class_name:
        raise RuntimeError('Failed to load class name...')

    model_dict = {
        ResNetDMP.__name__: ResNetDMP,
        VIMEDNet.__name__: VIMEDNet,
        ResNetMDDMP.__name__: ResNetMDDMP,
        ImgMaskDMP_net.__name__: ImgMaskDMP_net,
    }

    if class_name not in model_dict:
        raise RuntimeError('Unsupported model type: "' + class_name + '"...')

    return model_dict[class_name].load(filename)