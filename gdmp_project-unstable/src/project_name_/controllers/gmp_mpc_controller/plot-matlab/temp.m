clc;
close all;
clear;

gmp_reg = GMP_regressor(30,1.0);

gmp_reg.setTruncatedKernels(1e-4);

gmp_reg.plotRegressVec(linspace(-0.2,1.2, 1000));