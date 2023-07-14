clc;
close all;
clear;

%% includes
addpath('utils/');
import_io_lib();
import_gmp_lib();


%% Load exec data

dat = FileIO('../data/handover_exec_data0.bin', FileIO.in).readAll();

Time = dat.Time;
Time_target = dat.Time;
target_data = dat.target_data;
time_duration_data = dat.time_duration_data;

y0 = dat.P_data(:,1);
yg = target_data(:, end);
Tf = dat.Time(end);

pos_lim = dat.pos_lim;
vel_lim = dat.vel_lim;
accel_lim = dat.accel_lim;

data = {};

[~, P_data, dP_data, ddP_data] = getGMPTrajectory(y0, Time, time_duration_data, target_data);
data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
    'color',[0 0 1 0.7], 'legend','DMP', 'plot3D',true, 'plot2D',true);


data{length(data)+1} = ...
    struct('Time',dat.Time, 'Pos',dat.P_data, 'Vel',dat.dP_data, 'Accel',dat.ddP_data, 'linestyle','-', ...
    'color',[0.72 0.27 1], 'legend','$\overline{DMP}^*$', 'plot3D',true, 'plot2D',true);


%% ======= Plot results =========
label_font = 17;
ax_fontsize = 14;

%% --------- plot 3D paths ----------
fig = figure;
fig.Position(3:4) = [815 716];
ax = axes();
hold on;
plot3(y0(1), y0(2), y0(3), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',12, 'DisplayName', '$p_0$');
plot3(yg(1), yg(2), yg(3), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',12, 'DisplayName', 'handover');
%plot3(ygd(1), ygd(2), ygd(3), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',12, 'DisplayName', '$g_d$');
for k=1:length(data)
    if (~data{k}.plot3D), continue; end
    plot3(data{k}.Pos(1,:), data{k}.Pos(2,:), data{k}.Pos(3,:), 'LineWidth', 3, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'DisplayName',data{k}.legend);
end
plot3(target_data(1,:), target_data(2,:), target_data(3,:), 'LineWidth',2.5, 'LineStyle','-', 'Color',[1 0 0 0.5], 'DisplayName', 'Target');
% plot3([ygd(1) yg(1)], [ygd(2) yg(2)], [ygd(3) yg(3)], 'LineWidth', 1, 'LineStyle','--', 'Color',[1 0 1 0.5], 'HandleVisibility','off');
legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.8174 0.6641 0.1531 0.3144]);
% plot3Dbounds(ax, pos_lim, [1 1 1]);
view(73, 39);
grid on;
%     ax.XLim=x_lim; ax.YLim=y_lim; ax.ZLim=z_lim;
ax.FontSize = 14;
xlabel('x [$m$]', 'interpreter','latex', 'fontsize',18);
ylabel('y [$m$]', 'interpreter','latex', 'fontsize',18);
zlabel('z [$m$]', 'interpreter','latex', 'fontsize',18);
title('Cartesian path', 'interpreter','latex', 'fontsize',20);
hold off;


% fig = figure;
% fig.Position(3:4) = [842 1110];

% ======== position ==========
for i=1:3
    
    fig = figure;
    fig.Position(3:4) = [570 804];
    ax = subplot(4,1,1);
    ax_vec = [ax];
    hold on;
    % plot position trajectory
    legend_ = {};
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Pos(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'DisplayName', data{k}.legend);
        legend_ = [legend_ data{k}.legend];
    end
    % plot target
    plot(Time_target, target_data(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color',[1 0 0 0.5], 'DisplayName', 'Target update');
    axis tight;
    % plot start and final positions
    plot(0, y0(i), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',10, 'HandleVisibility','off');
    plot(Tf, yg(i), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'HandleVisibility','off');
%     plot(Tf, ygd(i), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',10, 'HandleVisibility','off');
    % plot bounds
    plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'HandleVisibility','off');
    plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'HandleVisibility','off');
    % labels, title ...
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',label_font);
    %     title(title_{i}, 'interpreter','latex', 'fontsize',18);
    legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.2330 0.9345 0.5520 0.0294], 'Orientation', 'horizontal');
    ax.FontSize = ax_fontsize;
    hold off;

    % ======== velocity ==========
    ax = subplot(4,1,2);
    ax_vec = [ax_vec ax];
    hold on;
    plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Vel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
    end
    axis tight;
    % plot bounds
    plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',label_font);
    ax.FontSize = ax_fontsize;
    hold off;

    % ======== acceleration ==========
    ax = subplot(4,1,3);
    ax_vec = [ax_vec ax];
    hold on;
    plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Accel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
    end
    axis tight;
    % plot bounds
    plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font);
    ax.FontSize = ax_fontsize;
    hold off;
    
    ax = subplot(4,1,4);
    ax_vec = [ax_vec ax];
    hold on;
    plot(Time, time_duration_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0 1 1]);
    axis tight;
    ax.FontSize = ax_fontsize;
    ylabel('$T_f$', 'interpreter','latex', 'fontsize',label_font);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font);
    hold off;
    
    linkaxes(ax_vec,'x');

end

%% phase variable
s_data = dat.phase_var_data(1,:);
sdot_data = dat.phase_var_data(2,:);
figure;
subplot(2,1,1);
plot(Time, s_data, 'LineWidth',2, 'LineStyle','-', 'Color','blue');
ylabel('$s$', 'interpreter','latex', 'fontsize',label_font);
subplot(2,1,2);
plot(Time, sdot_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0 1 1]);
ylabel('$\dot{s}$', 'interpreter','latex', 'fontsize',label_font);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font);
    
%% target
target_dot_data = zeros(size(target_data));
dTime = diff(Time);
for i=1:size(target_data,1)target_dot_data(i,:) = [diff(target_data(i,:))./dTime 0]; end
ind = find(target_dot_data(1,:) ~= 0);

target_dot_data = target_dot_data(:,ind);

a = 0.4;
for j=2:length(target_dot_data)
  target_dot_data(:,j) = a*target_dot_data(:,j) + (1-a)*target_dot_data(:,j-1);
end


figure;
for i=1:3
    subplot(2,3,i);
    plot(Time, target_data(i,:), 'LineStyle','-', 'Color','blue');
    ylabel('$y_g$', 'interpreter','latex', 'fontsize',label_font);
    subplot(2,3,i+3);
    plot(Time(ind), target_dot_data(i,:), 'LineStyle','-', 'Color','red');
    ylabel('$\dot{y}_g$', 'interpreter','latex', 'fontsize',label_font);
end

%% =========================================

y_data = dat.P_data;
% Tf_data = [];
% Tf_per_meter = 4;
% p_thres = 0.3;
% Tf0 = Tf_per_meter * norm( target_data(:,1) - y_data(:,1));
dist_data = [];
for j=1:size(y_data,2)
    g = target_data(:,j);
    y = y_data(:,j);
    t = Time(j);
    
    dist = norm(g-y);
%     if (dist > p_thres)
%         Tf = t + Tf_per_meter*dist;
%     else
%         Tf = t + Tf_per_meter*p_thres*p_thres/( dist+1e-16 );
%     end
%     if (Tf > Tf0), Tf = Tf0; end
%     
%     Tf_data = [Tf_data Tf];
    dist_data = [dist_data dist];
end

Tf_data = time_duration_data;

figure;
subplot(2,1,1); hold on;
plot(Time, Tf_data);
ylabel('$T_f$', 'interpreter','latex', 'fontsize',label_font);
subplot(2,1,2); hold on;
plot(Time, dist_data);
ylabel('$|| g - y ||$', 'interpreter','latex', 'fontsize',label_font);

%% ==========================================
%% ================= Utils ==================
%% ==========================================

function [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(y0, Time_data, Tf_data, target_data)
       
    import_gmp_lib();
    
    gmp = GMP();
    gmp_.read(gmp, '../config/handover_gmp_model.bin')
    
    Time = [];
    P_data = [];
    dP_data = [];
    ddP_data = [];

    p = y0;
    p_dot = zeros(size(p));
    p_ddot = zeros(size(p));
    
    gmp.setY0(y0);

    t = 0;
    dt = Time_data(2) - Time_data(1);
    
    Tf0 = Tf_data(1);
    can_sys = CanonicalSystem(Tf0);
    
    for j=1:length(Time_data)
        
        t = Time_data(j);
        Tf = Tf_data(j);
        yg = target_data(:,j);
        
        can_sys.setDuration(Tf, t);
        gmp.setGoal(yg);

        p_ref = gmp.getYd(can_sys.s);
        p_ref_dot = gmp.getYdDot(can_sys.s, can_sys.s_dot);
        p_ref_ddot = gmp.getYdDDot(can_sys.s, can_sys.s_dot, can_sys.getPhaseDDot());
        
        p_ddot = 300*(p_ref - p) + 80*(p_ref_dot - p_dot) + p_ref_ddot;

        Time = [Time t];
        P_data = [P_data p];
        dP_data = [dP_data p_dot];
        ddP_data = [ddP_data p_ddot];

        can_sys.integrate(t, t+dt);
        t = t + dt;
        p = p + p_dot*dt;
        p_dot = p_dot + p_ddot*dt;

    end

end

