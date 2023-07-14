clc;
close all;
clear;

n_dof = 3;
K = 30;
n_s = 3*n_dof;
N = 10;

H_size = (n_dof*K + n_s) *[1 1]

H_n_elem = H_size(1) * H_size(2)
