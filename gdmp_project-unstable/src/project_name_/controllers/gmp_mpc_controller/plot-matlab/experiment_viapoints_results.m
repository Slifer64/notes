clc;
close all;
clear;

%% includes
addpath('utils/');
import_io_lib();
import_gmp_lib();

%% Load GMP
gmp = GMP();
gmp_.read(gmp, '../config/viapoints_gmp_model.bin');

%% Load exec data
dat = FileIO('../data/viapoints_exec_data0.bin', FileIO.in).readAll();

Time = dat.Time;
Time_target = dat.Time;
target_data = dat.target_data;

y0 = dat.P_data(:,1);
yg = target_data(:, end);
Tf = dat.Time(end);

% is_traj = is_trajectory(dat.Time, dat.P_data, dat.dP_data, dat.ddP_data)
% return

pos_lim = dat.pos_lim;
vel_lim = dat.vel_lim;
accel_lim = dat.accel_lim;

s_via = dat.via_points_data(2,:);
t_via = s_via*Tf;
via_points = dat.via_points_data(3:end,:);

data = {};

%% ========== Simulate simple DMP ==============

[Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, Tf, y0, yg);
data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','--', ...
    'color',[0 0 1], 'legend','DMP', 'plot3D',true, 'plot2D',true);

% %% ========== Simulate offline GMP opt ==============
% vp_config.via_points = [];
% for j=1:length(dat.via_points_data)
%    vp_config.via_points{j} = struct('s', dat.via_points_data(2,j), 'pos',dat.via_points_data(3:5,j), 'err_tol',5e-3); 
% end
% [Time, P_data, dP_data, ddP_data] = offlineGMPweightsOpt(gmp, Tf, y0, yg, pos_lim, vel_lim, [1.2*[-1 1]; accel_lim(2:3,:)], 1, 0, vp_config, 1);
% data{length(data)+1} = ...
%         struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
%             'color',[0 1 0], 'legend',['$' 'DMP^*$'], 'plot3D',true, 'plot2D',true);

%% =========  set execution data =============
data{length(data)+1} = ...
    struct('Time',dat.Time, 'Pos',dat.P_data, 'Vel',dat.dP_data, 'Accel',dat.ddP_data, 'linestyle','-', ...
    'color',[0.72 0.27 1], 'legend','$\overline{DMP}^*$', 'plot3D',true, 'plot2D',true);


%% ======= Plot results =========
label_font = 17;
ax_fontsize = 14;

%% Cartesian path

%% --------- plot 3D paths ----------
fig = figure;
fig.Position(3:4) = [815 716];
ax = axes();
hold(ax, 'on')
plot3(y0(1), y0(2), y0(3), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',12, 'DisplayName', '$p_0$');
plot3(yg(1), yg(2), yg(3), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',12, 'DisplayName', '$g$');
%plot3(ygd(1), ygd(2), ygd(3), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',12, 'DisplayName', '$g_d$');
plot3(via_points(1,:), via_points(2,:), via_points(3,:), 'LineWidth', 3, 'LineStyle','none', 'Color','red','Marker','*', 'MarkerSize',12, 'DisplayName','via-points');
for k=1:length(data)
    if (~data{k}.plot3D), continue; end
    plot3(data{k}.Pos(1,:), data{k}.Pos(2,:), data{k}.Pos(3,:), 'LineWidth', 3, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'DisplayName',data{k}.legend);
end
% plot3([ygd(1) yg(1)], [ygd(2) yg(2)], [ygd(3) yg(3)], 'LineWidth', 1, 'LineStyle','--', 'Color',[1 0 1 0.5], 'HandleVisibility','off');
legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.8174 0.6641 0.1531 0.3144]);
plot3Dbounds(ax, pos_lim, [1 1 1]);
view(-152, 47);
grid on;
%     ax.XLim=x_lim; ax.YLim=y_lim; ax.ZLim=z_lim;
ax.FontSize = 14;
xlabel('x [$m$]', 'interpreter','latex', 'fontsize',18);
ylabel('y [$m$]', 'interpreter','latex', 'fontsize',18);
zlabel('z [$m$]', 'interpreter','latex', 'fontsize',18);
title('Cartesian path', 'interpreter','latex', 'fontsize',20);
hold(ax, 'off');


%% trajectories

title_ = {'$X$-axis', '$Y$-axis', '$Z$-axis'};

fig = figure;
fig.Position(3:4) = [832 804];
% ======== position ==========
ax_ind = [1 2 3];
m = length(ax_ind);
sp_ind = 1:m:(2*m+1);
for i1=1:m
    
    i = ax_ind(i1);
    
    ax = subplot(3,m,sp_ind(1), 'Parent',fig);
    ax_vec = [ax];
    hold(ax, 'on')
    % plot position trajectory
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Pos(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'DisplayName', data{k}.legend, 'Parent',ax);
    end
    % plot target
%     plot(Time_target, target_data(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color',[1 0 0 0.5], 'DisplayName', 'Target update');
    plot(t_via, via_points(i,:), 'LineWidth', 3, 'LineStyle','none', 'Color','red','Marker','*', 'MarkerSize',10, 'HandleVisibility','off', 'Parent',ax);
    axis(ax, 'tight');
    % plot start and final positions
    plot(0, y0(i), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',10, 'HandleVisibility','off', 'Parent',ax);
    plot(Tf, yg(i), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'HandleVisibility','off', 'Parent',ax);
%     plot(Tf, ygd(i), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',10, 'HandleVisibility','off', 'Parent',ax);
    % plot bounds
    plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'HandleVisibility','off', 'Parent',ax);
    plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'HandleVisibility','off', 'Parent',ax);
    % labels, title ...
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
    title(title_{i}, 'interpreter','latex', 'fontsize',18, 'Parent',ax);
    %     title(title_{i}, 'interpreter','latex', 'fontsize',18);
    if (i1==1), legend(ax, {}, 'interpreter','latex', 'fontsize',17, 'Position',[0.3815 0.9458 0.2718 0.0416], 'Orientation', 'horizontal'); end
    ax.FontSize = ax_fontsize;
    hold(ax,'off')

    % ======== velocity ==========
    ax = subplot(3,m,sp_ind(2), 'Parent',fig);
    ax_vec = [ax_vec ax];
    hold(ax, 'on')
    plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'Parent',ax);
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Vel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'Parent',ax);
    end
    axis(ax, 'tight');
    % plot bounds
    plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'Parent',ax);
    plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'Parent',ax);
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
    ax.FontSize = ax_fontsize;
    hold(ax,'off')

    % ======== acceleration ==========
    ax = subplot(3,m,sp_ind(3), 'Parent',fig);
    ax_vec = [ax_vec ax];
    hold(ax, 'on')
    plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'Parent',ax);
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Accel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'Parent',ax);
    end
    axis(ax, 'tight');
    % plot bounds
    plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'Parent',ax);
    plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'Parent',ax);
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
    ax.FontSize = ax_fontsize;
    hold(ax, 'off');
    
    linkaxes(ax_vec,'x');
    
    sp_ind = sp_ind + 1;

end

% ============  Utility functions =================

function [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, Tf, y0, yg)
       
    Time = [];
    P_data = [];
    dP_data = [];
    ddP_data = [];

    p = y0;
    p_dot = zeros(size(p));
    p_ddot = zeros(size(p));
    
    gmp.setY0(y0);
    gmp.setGoal(yg);

    t = 0;
    dt = 0.002;
    
    x = 0;
    
    while (true)
        
        x_dot = 1/Tf;
        
        % if (x >= 1), x_dot = 0; end
        
        p_ref = gmp.getYd(x);
        p_ref_dot = gmp.getYdDot(x, x_dot);
        p_ref_ddot = gmp.getYdDDot(x, x_dot, 0);
        
%         p_ddot = 300*(p_ref - p) + 80*(p_ref_dot - p_dot) + p_ref_ddot;
        
        p = p_ref;
        p_dot = p_ref_dot;
        p_ddot = p_ref_ddot;

        Time = [Time t];
        P_data = [P_data p];
        dP_data = [dP_data p_dot];
        ddP_data = [ddP_data p_ddot];

        t = t + dt;
        x = x + x_dot*dt;
        p = p + p_dot*dt;
        p_dot = p_dot + p_ddot*dt;

        if (x >= 1.0) % && norm(p_dot)<5e-3 && norm(p_ddot)<1e-2)
            break; 
        end

    end

end




