clc;
close all;
clear;

%% includes
import_io_lib();
import_math_lib();
import_plot_lib();
% import_gmp_lib();
addpath('./utils/');


obst_vp = {[-0.2, 0, -0.1], [-0.2, 0, 0.1]};

% data_file = 'data/c1_small_target_perturb.bin';
% target_vp = {[0, 0, -0.1], [0, 0, -0.05]};
% carton_dims = [0.04 0.1 0.04];
% n_boxes = 4;

% data_file = 'data/c2_large_no_via.bin';

% data_file = 'data/c3_large_target_perturb.bin';
% target_vp = {[0, 0, -0.3], [0, 0, -0.24], [0, 0, -0.18], [0, 0, -0.12], [0, 0, -0.06]};
% carton_dims = [0.12 0.12 0.20];
% n_boxes = 2;


data_file = 'data/c4_large_obstacle4.bin';
target_vp = {[0, 0, -0.3], [0, 0, -0.24], [0, 0, -0.18], [0, 0, -0.12], [0, 0, -0.06]};
carton_dims = [0.12 0.12 0.20];
n_boxes = 2;


box_dims = [n_boxes*carton_dims(1), carton_dims(2:3)] + [0.02 0.01 0.01];

%% Load exec data
log = FileIO(data_file, FileIO.in).readAll();
Tf = log.t_data(end);

P0 = log.P_data(1:3, 1);
Q0 = log.P_data(4:7, 1);

Pg = log.P_data(1:3, end);
Qg = log.P_data(4:7, end);

% return

%% demo
demo = getDemoData(Tf);

%% smooth accel for better viz??
% for i=1:size(log.Vd_dot_data,1)
%     log.Vd_dot_data(i,:) = smooth(log.Vd_dot_data(i,:), round(0.01*size(log.Vd_dot_data,2)), 'moving');
% end

%% calc target change
log.target_pos_change = vecnorm(log.Pg_data(1:3, :) - log.Pg_data(1:3, 1), 2, 1);
Qg_change = math_.quat2mat(math_.quatInv(log.Pg_data(4:7,1)))*log.Pg_data(4:7,:);
log.target_orient_change = 2*atan2(vecnorm(Qg_change(2:4,:), 2, 1), Qg_change(1,:)) * 180/pi;


%% Forces

% plot_forces(log);

plot_vel_target_change(demo, log);

% plot_accel(demo, log)
% plot_target_change(log);


%% ======== plot 3D paths ========

target_vp_data = get_target_viapoints(log.Pg_data(1:3,:), log.Pg_data(4:7,:), target_vp);

label_font = 17;
ax_fontsize = 14;

fig = figure;
fig.Position(3:4) = [847 507];
ax = axes();
ax.Position = [0.1229 0.1100 0.8517 0.8887];
hold(ax, 'on')
% -----------------------------------------
plot3(P0(1), P0(2), P0(3), 'LineWidth', 2.5, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',15, 'DisplayName','$p_0$');
plotFrame(ax, P0, Q0, 'AxisLength',0.1, 'AxisWidth',2);
plot3(Pg(1), Pg(2), Pg(3), 'LineWidth', 2.5, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',15, 'DisplayName','$g$');
plotFrame(ax, Pg, Qg, 'AxisLength',0.1, 'AxisWidth',2);
% -----------------------------------------
% place
plotOrientPath(ax, log.P_data(1:3,:), log.P_data(4:7,:), ...
    'LineWidth',2.5, 'Color',[0.96 0.69 0.13], 'LineStyle','-', 'FrameAxLen',0.1, 'addFrameDist',0.25, 'DisplayName','robot');
plot3(log.Pg_data(1,:), log.Pg_data(2,:), log.Pg_data(3,:), 'LineWidth',2, 'Color',[1 0.4 0.4], 'DisplayName','place target change');

for j=1:length(target_vp_data)
    alpha = (j/length(target_vp_data))^0.5;
    path_handle = plotOrientPath(ax, target_vp_data{j}.pos, target_vp_data{j}.quat, ...
        'FrameXcolor',[1, 0, 0.5, alpha], 'FrameYcolor',[0.4, 1, 0.4, alpha], 'FrameZcolor',[0, 0.5, 1, alpha], ...
        'LineWidth',2.5, 'Color',[0.96 0.69 0.13], 'LineStyle','-', ...
        'FrameAxLen',0.1, 'addFrameDist',0.05);
    set(path_handle, 'HandleVisibility','off');
end

% -----------------------------------------
% legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.8174 0.6641 0.1531 0.3144]);
view(78, 36);
grid on;
% ax.XLim=x_lim; ax.YLim=y_lim; ax.ZLim=z_lim;
ax.FontSize = 14;
xlabel('x [$m$]', 'interpreter','latex', 'fontsize',20);
ylabel('y [$m$]', 'interpreter','latex', 'fontsize',20);
zlabel('z [$m$]', 'interpreter','latex', 'fontsize',20);
% title('Cartesian path', 'interpreter','latex', 'fontsize',20);
hold(ax, 'off');
axis equal;


P1 = P0 + quat2rotm(Q0')*[0; 0; carton_dims(3)/2];
plotcube2(ax, P1, Q0, carton_dims, [0.7 0.3 0], 0.4);

P1 = log.Pg_data(1:3,1) + quat2rotm(log.Pg_data(4:7,1)')*[0; 0; carton_dims(3)/2];
plot_carton_and_box(ax, P1, log.Pg_data(4:7,1), carton_dims, box_dims, n_boxes, 0.3);

P1 = log.Pg_data(1:3,end) + quat2rotm(log.Pg_data(4:7,end)')*[0; 0; carton_dims(3)/2];
plot_carton_and_box(ax, P1, log.Pg_data(4:7,end), carton_dims, box_dims, n_boxes, 1.0);


obst_idx = find(~isnan(log.obst_data(1,:)));
if ~isempty(obst_idx)
    k = obst_idx(end-1300);
    P_obst = log.obst_data(1:3, k);
    Q_obst = log.obst_data(4:7, k);
    obst_dims = [0.19, 0.5, 0.19];
    plotcube2(ax, P_obst, Q_obst, obst_dims, [1 0 1], 0.4);
    aug_dims = 2*[0.08, 0.18, 0.08];
    plotcube2(ax, P_obst, Q_obst, obst_dims+aug_dims, [1 0 0], 0.1);
    
    Pv = {[0.3; 0.17; 0.475], [0.3; 0; 0.5]};
    for j=1:length(Pv)
        [~, k] = min(vecnorm(log.Pd_data(1:3,:) - Pv{j}, 2 ,1));
        Pv1 = log.Pd_data(:, k);
       plot3(Pv1(1), Pv1(2), Pv1(3), 'Marker','*', 'LineStyle','None', 'LineWidth',2.5, 'MarkerSize',16, 'Color','cyan');
    end
end

% ----- Annotations -------
annotation(fig, 'textbox', 'String','carton to place in box', 'Color',[0 0 0], 'FontSize',16, 'LineStyle','none', 'Position',[0.7531 0.2939 0.1773 0.1203]);
annotation(fig, 'arrow', 'Color',[0 0 0], 'LineWidth',1, 'LineStyle','-', 'Position',[0.8300 0.4122 0.0673 0.0868]);
% ----------------
annotation(fig, 'textbox', 'String',{'robot path', '+orientation'}, 'Color',[0 0 0], 'FontSize',16, 'LineStyle','none', 'Position',[0.7861 0.8698 0.1619 0.1065]);
annotation(fig, 'arrow', 'Color',[0 0 0], 'LineWidth',1, 'LineStyle','-', 'Position',[0.8808 0.8698 -0.0685 -0.0375]);
% ----------------
annotation(fig, 'textbox', 'String',{'Initial target', '+via-points'}, 'Color',[0 0 0], 'FontSize',16, 'LineStyle','none', 'Position',[0.5886 0.4911 0.1773 0.1203]);
annotation(fig, 'arrow', 'Color',[0 0 0], 'LineWidth',1, 'LineStyle','-', 'Position',[0.6210 0.6016 -0.1122 0.0414]);
% ----------------
annotation(fig, 'textbox', 'String','Target change', 'Color',[0 0 0], 'FontSize',16, 'LineStyle','none', 'Position',[0.4036 0.2722 0.1029 0.0986]);
annotation(fig, 'arrow', 'Color',[0 0 0], 'LineWidth',1, 'LineStyle','-', 'Position',[0.4262 0.3629 -0.0449 0.1775]);
% ----------------
annotation(fig, 'textbox', 'String',{'Final target','+via-points'}, 'Color',[0 0 0], 'FontSize',16, 'LineStyle','none', 'Position',[0.2359 0.1972 0.1749 0.1026]);
annotation(fig, 'arrow', 'Color',[0 0 0], 'LineWidth',1, 'LineStyle','-', 'Position',[0.3353 0.3037 -0.0496 0.2249]);


% demo.Pos = 0.95*(demo.Pos - demo.Pos(:,1)) + P0 + [-0.12; -0.06; 0];
% 
% demo_pos_dist0 = norm(demo.Pos(:, end) - demo.Pos(:,1))
% demo_orient_dist0 = norm(math_.quatLog(math_.quatDiff(demo.Quat(:,end), demo.Quat(:,1))))*180/pi
% 
% exec_pos_dist0 = norm(log.Pg_data(1:3,1) - P0)
% exec_orient_dist0 = norm(math_.quatLog(math_.quatDiff(log.Pg_data(4:7,1), Q0)))*180/pi
% 
% plot3(demo.Pos(1,:), demo.Pos(2,:), demo.Pos(3,:), 'LineWidth',2.5, 'LineStyle',':', 'Color',[0 1 1], 'Parent',ax);

%% =========================================
%% =========================================

function plot_carton_and_box(ax, Pg, Qg, carton_dims, box_dims, n_boxes, alpha_scale)
    
    %plotcube2(ax, Pg, Qg, carton_dims, 'red', 0.3);
    Rg = quat2rotm(Qg');
    P_box = Pg + Rg*[(n_boxes/2 - 0.5)*carton_dims(1); 0; 0];
    box_h = plotcube2(ax, P_box, Qg, box_dims, [0.4 0.4 0.4], alpha_scale*0.75);
    delete(box_h.side1); % delete top side
    P_add = Rg*[carton_dims(1)+0.002; 0; 0];
    P1 = Pg + P_add;
    for k=1:n_boxes-1
        plotcube2(ax, P1, Qg, carton_dims, [0.7 0.3 0], alpha_scale*0.4);
        P1 = P1 + P_add;
    end

end

function target_vp_data = get_target_viapoints(Pg_data, Qg_data, vp)

    % dist_thres = 0.3;
    % dist = 0;
    % ind = [];
    % for j=2:size(Pg_data,2)
    %     dist = dist + norm(Pg_data(:,j) - Pg_data(:,j-1));
    %     if (dist > dist_thres)
    %         ind = [ind j];
    %         dist = 0;
    %     end
    % end
    % ind = [1 ind size(Pg_data,2)];
    ind = [1 size(Pg_data,2)];
    Pg_data = Pg_data(:, ind);
    Qg_data = Qg_data(:, ind);

    target_vp_data = cell(size(Pg_data,2), 1);
    for j=1:length(target_vp_data)
        Vp_data = zeros(3, length(vp));
        p = Pg_data(:,j);
        R = quat2rotm(Qg_data(:,j)');
        for k=1:length(vp)
            z_offset = vp{k}(:);
            Vp_data(:, k) = p + R*z_offset;
        end
        target_vp_data{j} = struct('pos',Vp_data, 'quat',repmat(Qg_data(:,j), 1, length(vp)));
    end

end


function expand_ylim(ax, percent)

    ax.YLim = ax.YLim + percent*[-1 1]*diff(ax.YLim);

end

function plot_vel_target_change(demo, log)
    
    ax_font = 13;
    leg_font = 16;
    xlb_font = 14;
    ylb_font = 15;

    fig = figure;

    % ------  Position  --------
    ax = subplot(2, 1, 1); hold on; grid on; ax.FontSize=ax_font;
    % box on;
    pl1 = plot(log.t_data, vecnorm(log.Vd_data(1:3,:), 2, 1), 'LineWidth',2, 'Color','blue', 'DisplayName','exec');
    pl2 = plot(demo.Time, vecnorm(demo.Vel, 2, 1), 'LineWidth',2, 'Color','green', 'DisplayName','demo');
    ylabel('$||\dot{p}||$ $m/s$', 'interpreter','latex', 'fontsize',ylb_font);
    % title('Velocity', 'interpreter','latex', 'fontsize',18);
    legend({}, 'interpreter','latex', 'fontsize',leg_font, 'Box','off', 'Orientation','horizontal', 'Position',[0.0949 0.9257 0.3419 0.0705]);
    axis(ax, 'tight');
    set(ax, 'XTick',[], 'Position',[0.1306 0.5786 0.7744 0.3551]);
    expand_ylim(ax, 0.05);

    ax0 = axes('Parent',fig);
    pl3 = plot(log.t_data, log.target_pos_change, 'LineWidth',1.5, 'LineStyle','-', 'Color','magenta', 'DisplayName','$||p_g - p_g(0)||$');
    axis(ax0, 'tight');
    legend({}, 'fontsize',leg_font, 'interpreter','latex', 'Position',[0.6158 0.9273 0.2981 0.0705], 'Orientation','horizontal', 'Box','off');
    set(ax0, 'YAxisLocation','right', 'XAxisLocation','top', 'XTick',[], 'fontsize',ax_font, 'color','none', 'Box','off');
    ylabel('[$m$]', 'fontsize',ylb_font, 'interpreter','latex', 'Position',[11, 0.15, -1]);
    ax0.YTick = round(linspace(0, pl3.YData(end), 3)*100)/100;
    ax0.Position = ax.Position;
    expand_ylim(ax0, 0.05);

    % ------  Orientation  --------
    ax = subplot(2, 1, 2); hold on; grid on; ax.FontSize=ax_font;
    % box on;
    plot(log.t_data, vecnorm(log.Vd_data(4:6,:), 2, 1), 'LineWidth',2, 'Color','blue');
    plot(demo.Time, vecnorm(demo.RotVel, 2, 1), 'LineWidth',2, 'Color','green');
    ylabel('$||\omega||$ $rad/s$', 'interpreter','latex', 'fontsize',ylb_font);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',xlb_font);
    axis(ax, 'tight');
    set(ax, 'Position',[0.1300 0.1235 0.7750 0.3551]);
    expand_ylim(ax, 0.05);

    ax0 = axes('Parent',fig);
    pl3 = plot(log.t_data, log.target_orient_change, 'LineWidth',1.5, 'LineStyle','-', 'Color','magenta', 'DisplayName','$||\log(Q_g * \bar{Q}_g(0))|$');
    axis(ax0, 'tight');
    legend({}, 'fontsize',leg_font, 'interpreter','latex', 'Position',[0.5335 0.4773 0.3841 0.0705], 'Orientation','horizontal', 'Box','off');
    set(ax0, 'YAxisLocation','right', 'XAxisLocation','top', 'XTick',[], 'fontsize',ax_font, 'color','none', 'Box','off');
    ylabel('[degrees]', 'fontsize',ylb_font, 'interpreter','latex');
    ax0.YTick = round(linspace(0, pl3.YData(end), 3));
    ax0.Position = ax.Position;
    expand_ylim(ax0, 0.05);

end

function plot_target_change(log)

    figure;
    subplot(2, 1, 1); hold on; box on; grid on;
    plot(log.t_data, log.target_pos_change, 'Color','magenta', 'DisplayName','$||p_g - p_g(0)||$');
    ylabel('$m$', 'interpreter','latex', 'fontsize',14);
    legend({}, 'interpreter','latex', 'fontsize',16, 'Box','off');
    title('Target pose perturbation', 'interpreter','latex', 'fontsize',18);
    axis tight;
    subplot(2, 1, 2); hold on; box on; grid on;
    plot(log.t_data, log.target_orient_change, 'Color','magenta', 'DisplayName','$||\log(Q_g * \bar{Q}_g(0))||$');
    ylabel('$degrees$', 'interpreter','latex', 'fontsize',14);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
    legend({}, 'interpreter','latex', 'fontsize',16, 'Box','off');

end


function plot_accel(demo, log)
    
figure;
subplot(2, 1, 1); hold on; box on; grid on;
plot(log.t_data, vecnorm(log.Vd_dot_data(1:3,:), 2, 1), 'Color','blue', 'DisplayName','exec');
plot(demo.Time, vecnorm(demo.Accel, 2, 1), 'Color','green', 'DisplayName','demo');
ylabel('$||\ddot{p}||$ $m/s^2$', 'interpreter','latex', 'fontsize',14);
title('Acceleration', 'interpreter','latex', 'fontsize',18);
legend({}, 'interpreter','latex', 'fontsize',16, 'Box','off');
axis tight;
subplot(2, 1, 2); hold on; box on; grid on;
plot(log.t_data, vecnorm(log.Vd_dot_data(4:6,:), 2, 1), 'Color','blue');
plot(demo.Time, vecnorm(demo.RotAccel, 2, 1), 'Color','green');
ylabel('$||\dot{\omega}||$ $rad/s^2$', 'interpreter','latex', 'fontsize',14);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
axis tight;

end

function plot_forces(log)

    figure; 
    subplot(2, 1, 1); hold on; box; grid on;
    plot(log.t_data, vecnorm(log.Fext_data(1:3,:), 2, 1));
    ylabel('$||f||$ [$N$]', 'interpreter','latex', 'fontsize',16);
    title('F/T measurements', 'interpreter','latex', 'fontsize',18);
    axis tight;
    subplot(2, 1, 2); hold on; box; grid on;
    plot(log.t_data, vecnorm(log.Fext_data(4:6,:), 2, 1));
    ylabel('$||\tau||$ [$Nm$]', 'interpreter','latex', 'fontsize',16);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
    axis tight;

end


function demo = getDemoData(Tf, P0, Pg, Q0, Qg)
    
    import_gmp_lib();
    gmp = GMP();
    gmp_.read(gmp, 'data/place_model.bin', 'pos_');
    gmp_o = GMPo();
    gmp_.read(gmp_o, 'data/place_model.bin', 'orient_');
    
    if (nargin == 5)
        gmp.setY0(P0);
        gmp.setGoal(Pg);
        gmp_o.setQ0(Q0);
        gmp_o.setQg(Qg);
    end

    n_points = 200;
    s_data = linspace(0, 1, 200);
    demo = struct('Time',s_data*Tf, 'Pos',zeros(3, n_points), 'Vel',zeros(3, n_points), 'Accel',zeros(3, n_points),...
        'Quat',zeros(4, n_points), 'RotVel',zeros(3, n_points), 'RotAccel',zeros(3, n_points));
    for j=1:n_points
        s = s_data(j);
        demo.Pos(:,j) = gmp.getYd(s);
        demo.Vel(:,j) = gmp.getYdDot(s, 1/Tf);
        demo.Accel(:,j) = gmp.getYdDDot(s, 1/Tf, 0);
        demo.Quat(:,j) = gmp_o.getQd(s);
        demo.RotVel(:,j) = gmp_o.getVd(s, 1/Tf);
        demo.RotAccel(:,j) = gmp_o.getVdDot(s, 1/Tf, 0);
    end
    
end