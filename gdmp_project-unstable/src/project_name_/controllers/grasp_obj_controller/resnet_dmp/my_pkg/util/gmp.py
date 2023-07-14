import pickle

import numpy as np
import matplotlib.pyplot as plt
import math
import copy

from typing import Union, Tuple


class GaussianKernelsRegressor:

    def __init__(self, n_kernels: int, kernels_std_scaling: float = 1.5):
        self.__init_helper__(n_kernels, kernels_std_scaling)

    def __init_helper__(self, n_kernels: int, kernels_std_scaling: float = 1.5):
        self.n_kernels = n_kernels
        self.kernels_std_scaling = kernels_std_scaling

        self.c = np.linspace(0.0, 1.0, self.n_kernels)[:, np.newaxis]

        hi = 1.0 / (kernels_std_scaling * (self.c[1] - self.c[0])) ** 2
        self.h = np.ones((n_kernels, 1)) * hi

        zero_tol = np.nextafter(np.float32(0), np.float32(1))
        self.s_min = self.c[0] - math.sqrt(-math.log(zero_tol) / self.h[0])
        self.s_max = self.c[-1] + math.sqrt(-math.log(zero_tol) / self.h[-1])

    def regress_vec(self, s: float) -> np.array:

        # take appropriate actions when x causes phi = 0 due to finite
        # numerical precision.
        if s < self.s_min:
            psi = np.zeros((self.n_kernels, 1))
            psi[0] = 1.0
        elif s > self.s_max:
            psi = np.zeros((self.n_kernels, 1))
            psi[-1] = 1.0
        else:
            psi = self._kernel_fun(s)

        phi = psi / np.sum(psi)
        return phi

    def regress_vec_dot(self, s: float, s_dot: float) -> np.array:

        if s < self.s_min or s > self.s_max:
            return np.zeros((self.n_kernels, 1))

        psi = self._kernel_fun(s)
        psi_dot = self._kernel_fun_dot(s, s_dot)
        sum_psi = np.sum(psi)
        sum_psi_dot = np.sum(psi_dot)

        phi = psi / sum_psi
        phi_dot = (psi_dot - phi * sum_psi_dot) / sum_psi
        return phi_dot

    def regress_vec_ddot(self, s: float, s_dot: float, s_ddot: float) -> np.array:

        if s < self.s_min or s > self.s_max:
            return np.zeros((self.n_kernels, 1))

        psi = self._kernel_fun(s)
        psi_dot = self._kernel_fun_dot(s, s_dot)
        psi_ddot = self._kernel_fun_ddot(s, s_dot, s_ddot)
        sum_psi = np.sum(psi)
        sum_psi_dot = np.sum(psi_dot)
        sum_psi_ddot = np.sum(psi_ddot)

        phi = psi / sum_psi
        phi_dot = (psi_dot - phi * sum_psi_dot) / sum_psi
        phi_ddot = (psi_ddot - 2 * phi_dot * sum_psi_dot - phi * sum_psi_ddot) / sum_psi
        return phi_ddot

    def _kernel_fun(self, s: float) -> np.array:
        return np.exp(-self.h * np.power(s - self.c, 2))

    def _kernel_fun_dot(self, s: float, s_dot: float) -> np.array:
        psi = self._kernel_fun(s)
        a = (s - self.c) * s_dot
        psi_dot = -2 * self.h * (psi * a)
        return psi_dot

    def _kernel_fun_ddot(self, s: float, s_dot: float, s_ddot: float) -> np.array:

        psi = self._kernel_fun(s)
        psi_dot = self._kernel_fun_dot(s, s_dot)
        a = (s - self.c) * s_dot
        a_dot = (s - self.c) * s_ddot + s_dot ** 2
        psi_ddot = -2 * self.h * (psi_dot * a + psi * a_dot)

        return psi_ddot


class MovementPrimitive(GaussianKernelsRegressor):

    def __init__(self, n_dofs: int, n_kernels: int, kernels_std_scaling: float = 1.5):
        super().__init__(n_kernels, kernels_std_scaling)
        self._config = {}
        self._init_config(locals())

        self.n_dofs = n_dofs
        self.weights = np.zeros((n_dofs, n_kernels))

    def _init_config(self, locals_dict):
        self._config = locals_dict
        self._config.pop('self')
        self._config.pop('__class__')

    def get_pos(self, s: float) -> np.array:
        return np.matmul(self.weights, self.regress_vec(s))

    def get_vel(self, s: float, s_dot: float) -> np.array:
        return np.matmul(self.weights, self.regress_vec_dot(s, s_dot))

    def get_accel(self, s: float, s_dot: float, s_ddot: float) -> np.array:
        return np.matmul(self.weights, self.regress_vec_ddot(s, s_dot, s_ddot))

    def train(self, s: np.array, pos_data: np.array, train_method: str = 'LS', end_points_constraints=False):

        s = np.squeeze(s)

        if pos_data.ndim == 1:
            pos_data = np.expand_dims(pos_data, 0)

        if pos_data.shape[0] != self.n_dofs:
            raise AttributeError('[MovementPrimitive::train]: The training data have wrong number of DoFs...')

        if np.any(s > 1) or np.any(s < 0):
            print('\33[1m\33[33m[MovementPrimitive::train]: The training timestamps are not normalized...\33[0m')

        H = np.hstack([self.regress_vec(s[j]) for j in range(len(s))])

        if train_method.upper() == 'LWR':
            self.weights = np.matmul(pos_data, H.transpose()) / np.sum(H, axis=1).transpose()
        elif train_method.upper() == 'LS':
            self.weights = np.linalg.lstsq(H.transpose(), pos_data.transpose(), rcond=None)[0].transpose()
        else:
            print('\33[1m\33[31m[MovementPrimitive::train]: Unsupported training method...\33[0m')

        if end_points_constraints:
            Sw = np.linalg.inv(np.matmul(H, H.transpose()))

            # enforce start and end point constraints
            A = np.hstack([self.regress_vec(0), self.regress_vec(1),
                           self.regress_vec_dot(0, 1), self.regress_vec_dot(1, 1),
                           self.regress_vec_ddot(0, 1, 0), self.regress_vec_ddot(1, 1, 0)])
            b = np.stack([pos_data[:, 0], pos_data[:, -1],
                          np.zeros((self.n_dofs,)), np.zeros((self.n_dofs,)),
                          np.zeros((self.n_dofs,)), np.zeros((self.n_dofs,))],
                         axis=1)

            R = np.diag([1e-5, 1e-5, 1e-3, 1e-3, 1e-2, 1e-2])
            Sw_At = np.matmul(Sw, A)  # A is already transposed!
            K = np.matmul(Sw_At, np.linalg.inv(R + np.matmul(A.transpose(), Sw_At)))
            e = (b - np.matmul(self.weights, A)).transpose()
            self.weights = self.weights + np.matmul(K, e).transpose()

        err_data = np.matmul(self.weights, H) - pos_data
        train_err = np.linalg.norm(err_data, axis=1)

        return train_err

    def reconfig(self, n_kernels=None, kernels_std_scaling=None,
                 n_points=200, train_method='LS', end_points_constraints=False):

        if not n_kernels:
            n_kernels = self.n_kernels
        if not kernels_std_scaling:
            kernels_std_scaling = self.kernels_std_scaling

        s_data = np.linspace(0, 1, n_points)
        pos_data = np.hstack([self.get_pos(s) for s in s_data])
        # reconfigure MP
        self.__init_helper__(n_dofs=self.n_dofs, n_kernels=n_kernels, kernels_std_scaling=kernels_std_scaling)
        return self.train(s_data, pos_data, train_method=train_method, end_points_constraints=end_points_constraints)

    def to_state_dict(self):
        return {'weights': self.weights, 'config': self._config}

    @classmethod
    def from_state_dict(cls, state_dict):
        mp = cls(**state_dict['config'])
        mp.weights = state_dict['weights']
        return mp

    def save(self, filename):
        pickle.dump(self.to_state_dict(), open(filename, 'wb'))

    @staticmethod
    def load(filename):
        return MovementPrimitive.from_state_dict(pickle.load(open(filename, 'rb')))

    def deep_copy(self):
        return copy.deepcopy(self)

