clc;
close all;
clear;

import_io_lib();
import_gmp_lib();
import_math_lib();

filename = 'place_demo.bin';
save_model_as = 'place_model.bin';

% filename = 'pick_demo.bin';
% save_model_as = 'pick_model.bin';

data = FileIO(filename).readAll();

[Timed, Pd_data, Qd_data] = processData(data.t_data, data.Pd_data(1:3,:), data.Pd_data(4:7,:));

x_data = Timed/Timed(end);

% p_dist = norm(Pd_data(:,end) - Pd_data(:,1))
% o_dist = norm(math_.quatLog(math_.quatDiff(Qd_data(:,end), Qd_data(:,1))))*180/pi

%% ============  position ===========

gmp = GMP(3, 30, 1.5);
train_err = gmp.train('LS', x_data, Pd_data)

tau = Timed(end);
x_dot = 1/tau;

gmp.setY0(Pd_data(:,1));
gmp.setGoal(Pd_data(:,end));

n_data = length(x_data);
P_data = zeros(3, n_data);
for j=1:n_data, P_data(:,j) = gmp.getYd(x_data(j)); end

y_labels = {'x', 'y', 'z'};
figure;
for i=1:3
   subplot(3,1,i); hold on;
   plot(Timed, P_data(i,:), 'LineWidth',2, 'Color','magenta');
   plot(Timed, Pd_data(i,:), 'LineWidth',2, 'Color','blue', 'LineStyle',':');
   ylabel(y_labels{i}, 'fontsize',14);
   if (i==1), title('Cartesian Position', 'fontsize',16); end
   axis tight;
end

plotTrajectory(gmp);

% gmp_.write(gmp, 'obstacles_gmp_model.bin', '');

%% ============  orient ===========

gmp_o = GMPo(30, 1.5);
train_err_o = gmp_o.train('LS', x_data, Qd_data)

gmp_o.setQ0(Qd_data(:,1));
gmp_o.setQg(Qd_data(:,end));

n_data = length(x_data);
Q_data = zeros(4, n_data);
for j=1:n_data, Q_data(:,j) = gmp_o.getQd(x_data(j)); end

y_labels = {'w', 'x', 'y', 'z'};

figure;
for i=1:4
   subplot(4,1,i); hold on;
   plot(Timed, Q_data(i,:), 'LineWidth',2, 'Color','magenta');
   plot(Timed, Qd_data(i,:), 'LineWidth',2, 'Color','blue', 'LineStyle',':');
   ylabel(y_labels{i}, 'fontsize',14);
   if (i==1), title('Cartesian Orientation (Quaternion)', 'fontsize',16); end
   axis tight;
end

plotGMPoTrajectory(gmp_o);

% gmp_.write(gmp_o, 'orient_gmp_model.bin', '');


%% save models
fid = FileIO(save_model_as, bitor(FileIO.out, FileIO.trunc));
gmp_.write(gmp, fid, 'pos_');
gmp_.write(gmp_o, fid, 'orient_');


%% =================================================== 

function [Timed, Pd_data, Qd_data] = processData(Timed, Pd_data, Qd_data)
    
    N0 = size(Pd_data, 2);
    
    %% smoothing
    for k=1:15
        % use a moving average window with 3% width
        for i=1:size(Pd_data,1)
            Pd_data(i, :) = smooth(Pd_data(i, :), round(N0*0.03), 'moving');
        end
        for i=1:size(Qd_data,1)
            Qd_data(i, :) = smooth(Qd_data(i, :), round(N0*0.03), 'moving');
        end
        Qd_data = Qd_data ./ vecnorm(Qd_data, 2, 1); % normalize
    end
    
    %% trim
    p_thres = 8e-4; % trim after surpassing the distance threshold
    theta_thres = 0.01;
    
    % trim indices
    j1 = 1;
    j2 = size(Pd_data,2);
    
    % trim from start
    for j=1:j2-1
       if (norm(Pd_data(:,j+1) - Pd_data(:,1)) > p_thres || ...
           norm(math_.quatLog(math_.quatDiff(Qd_data(:,j+1), Qd_data(:,1)))) > theta_thres)
           j1 = j+1;
           break;
       end
    end
    
    % trim from end
    for j=j2:-1:j1+1
       if (norm(Pd_data(:,j-1) - Pd_data(:,end)) > p_thres || ...
           norm(math_.quatLog(math_.quatDiff(Qd_data(:,j-1), Qd_data(:,end)))) > theta_thres)
           j2 = j-1;
           break;
       end
    end

    N1 = j2 - j1 + 1;

    fprintf('n0 = %d , n1 = %d', N0, N1);

    Timed = Timed(j1:j2) - Timed(j1);
    Pd_data = Pd_data(:, j1:j2);
    Qd_data = Qd_data(:, j1:j2);

end

function plotTrajectory(gmp)
    
    c = class(gmp);
    if ( strcmpi(c, 'GMP') ), plotGMPTrajectory(gmp);
    elseif ( strcmpi(c, 'GMPo') ), plotGMPoTrajectory(gmp);
    else, error('[plotTrajectory]: Unsupported class type');
    end

end

function plotGMPTrajectory(gmp)
    
    n_data = 200;
    s_data = linspace(0, 1, n_data);
    P_data = zeros(3, n_data);
    dP_data = zeros(3, n_data);
    ddP_data = zeros(3, n_data);

    Tf = 5;

    for j=1:n_data
       s = s_data(j);
       P_data(:, j) = gmp.getYd(s);
       dP_data(:, j) = gmp.getYdDot(s, 1/Tf);
       ddP_data(:, j) = gmp.getYdDDot(s, 1/Tf, 0);
    end

    fig = figure;
    ax_ = cell(3, 3);
    for i=1:3
        for j=1:3, ax_{j, i} = subplot(3, 3, (i-1)*3+j); end
    end
    for i=1:3
        plot(s_data, P_data(i,:), 'Color','blue', 'Parent',ax_{i, 1});
        plot(s_data, dP_data(i,:), 'Color','green', 'Parent',ax_{i, 2});
        plot(s_data, ddP_data(i,:), 'Color','red', 'Parent',ax_{i, 3});
    end
    
    title('$X$', 'interpreter','latex', 'fontsize',18, 'Parent',ax_{1,1});
    title('$Y$', 'interpreter','latex', 'fontsize',18, 'Parent',ax_{2,1});
    title('$Z$', 'interpreter','latex', 'fontsize',18, 'Parent',ax_{3,1});
    
    ylabel('$p$ [$m$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax_{1,1});
    ylabel('$\dot{p}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax_{1,2});
    ylabel('$\ddot{p}$ [$m/s^2$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax_{1,3});
end

function plotGMPoTrajectory(gmp)
    
    n_data = 200;
    s_data = linspace(0, 1, n_data);
    qlog_data = zeros(3, n_data);
    rotVel_data = zeros(3, n_data);
    rotAccel_data = zeros(3, n_data);

    Tf = 5;

    for j=1:n_data
       s = s_data(j);
       qlog_data(:, j) = gmp.getYd(s);
       rotVel_data(:, j) = gmp.getVd(s, 1/Tf);
       rotAccel_data(:, j) = gmp.getVdDot(s, 1/Tf, 0);
    end

    fig = figure;
    ax_ = cell(3, 3);
    for i=1:3
        for j=1:3, ax_{j, i} = subplot(3, 3, (i-1)*3+j); end
    end
    for i=1:3
        plot(s_data, qlog_data(i,:), 'Color','blue', 'Parent',ax_{i, 1});
        plot(s_data, rotVel_data(i,:), 'Color','green', 'Parent',ax_{i, 2});
        plot(s_data, rotAccel_data(i,:), 'Color','red', 'Parent',ax_{i, 3});
    end
    
    title('$X$', 'interpreter','latex', 'fontsize',18, 'Parent',ax_{1,1});
    title('$Y$', 'interpreter','latex', 'fontsize',18, 'Parent',ax_{2,1});
    title('$Z$', 'interpreter','latex', 'fontsize',18, 'Parent',ax_{3,1});
    
    ylabel('$\log(Q*\bar{Q}_0)$', 'interpreter','latex', 'fontsize',15, 'Parent',ax_{1,1});
    ylabel('$\omega$ [$rad/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax_{1,2});
    ylabel('$\dot{\omega}$ [$rad/s^2$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax_{1,3});

end
