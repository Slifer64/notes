clc;
close all;
clear;

%% =============  Create training data  =============
y0 = 0;
yf = 10;
tf = 3;
Timed = linspace(0, 3, 1000);

[yd_data, yd_dot_data, yd_ddot_data] = get5thOrderPol(y0, yf, Timed);


% includes...
import_gmp_lib();

%% =============  Set GMP params  =============
train_method = 'LS'; % {'LS', 'LWR'}
N_kernels = 25;  % number or kernels (Gaussians)
kernels_std_scaling = 1.3; % scaling of the width of each Gaussian. The greater 
                           % the scaling the more the overlapping). A vaule 
                           % in [1 1.5] is usually good.

n_dof = size(yd_data, 1);

%% =============  Create/Train GMP  =============
gmp = GMP(n_dof, N_kernels, kernels_std_scaling);
t_start = tic;
sd_data = Timed/Timed(end);
offline_train_mse = gmp.train(train_method, sd_data, yd_data);
offline_train_mse
toc(t_start)

%% =============  Simulate GMP  =============
K = 300;
D = 80;

Time = [];
y_data = [];
y_dot_data = [];
y_ddot_data = [];

t = 0;
dt = 0.005;
% phase var
x = 0;
x_dot = 1/tf;
x_ddot = 0;
% state
y = y0;
y_dot = zeros(n_dof, 1);
y_ddot = zeros(n_dof, 1);

% simulation loop
while (t < tf)
    
    % log data
    Time = [Time t];
    y_data = [y_data y];
    y_dot_data = [y_dot_data y_dot];
    y_ddot_data = [y_ddot_data y_ddot];
    
    % DMP reference
    yx = gmp.getYd(x);
    yx_dot = gmp.getYdDot(x, x_dot);
    yx_ddot = gmp.getYdDDot(x, x_dot, x_ddot);
    
    y_ddot = yx_ddot + D*(yx_dot - y_dot) + K*(yx - y);
    
    % numerical integration
    t = t + dt;
    x = x + x_dot*dt;
    x_dot = x_dot + x_ddot*dt;
    y = y + y_dot*dt;
    y_dot = y_dot + y_ddot*dt;
    
end

%% ============= Plot results ==============
fig = figure;
fig.Position(3:4) = [640 984];
% position
subplot(3, 1, 1); hold on;
plot(Time, y_data, 'LineWidth',2, 'LineStyle','-', 'Color','magenta', 'DisplayName','DMP');
plot(Timed, yd_data, 'LineWidth',2, 'LineStyle','--', 'Color','blue', 'DisplayName','demo');
legend({}, 'fontsize',14);
ylabel('pos', 'fontsize',15);
axis tight;

% velocity
subplot(3, 1, 2); hold on;
plot(Time, y_dot_data, 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
plot(Timed, yd_dot_data, 'LineWidth',2, 'LineStyle','--', 'Color','blue');
ylabel('vel', 'fontsize',15);
axis tight;

% acceleration
subplot(3, 1, 3); hold on;
plot(Time, y_ddot_data, 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
plot(Timed, yd_ddot_data, 'LineWidth',2, 'LineStyle','--', 'Color','blue');
ylabel('accel', 'fontsize',15);
xlabel('time [s]', 'fontsize',14);
axis tight;

%% #####################################################
%% Utility functions

function [y, y_dot, y_ddot] = get5thOrderPol(y0, yf, Time)

    y0 = y0(:);
    yf = yf(:);
    Time = Time(:)';
    
    T = Time(end);
    t = Time/T;
    
    y = y0 + (yf - y0) * (10*t.^3 - 15*t.^4 + 6*t.^5 );

    if (nargout > 1)
        y_dot = (yf - y0) * (30*t.^2 - 60*t.^3 + 30*t.^4 ) / T;
    end

    if (nargout > 2)
        y_ddot = (yf - y0) * (60*t - 180*t.^2 + 120*t.^3 ) / T^2;
    end

end