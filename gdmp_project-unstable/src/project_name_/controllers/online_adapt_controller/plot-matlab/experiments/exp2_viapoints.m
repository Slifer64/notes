clc;
close all;
clear;

%% includes
import_io_lib();
import_math_lib();
import_plot_lib();
addpath('./utils/');

data_path = './data/';

cnt = num2str(3);

%% Load exec data
seg1 = FileIO([data_path cnt '_exec_pick_frw.bin'], FileIO.in).readAll();
seg2 = FileIO([data_path cnt '_exec_place_frw.bin'], FileIO.in).readAll();


[Time0, P0_data, dP0_data, ddP0_data] = getModelTrajectory(seg1);


seg2.t_data = seg2.t_data - seg2.t_data(1) + seg1.t_data(end);

%% find via-points
ind = find(seg1.sv_data > 0);
Timev_data = seg1.sv_data(ind)*seg1.t_data(end);
Pv_data = seg1.Pv_data(:, ind);

% subsample via-points for clearer visualization
dist_thres = 0.05;
dist = 0;
Timev = [Timev_data(1)];
Pv = [Pv_data(:,1)];
for j=2:length(Timev_data)
    dist = dist + norm(Pv_data(1:3,j)-Pv_data(1:3,j-1));
    if (dist > dist_thres)
        dist = 0;
        Timev = [Timev Timev_data(j)];
        Pv = [Pv Pv_data(:,j)];
    end
end
Timev_data = Timev;
Pv_data = Pv;


%% ======== plot x-position and force ========

fig = figure;
ax_font = 13;
leg_font = 15;
label_font = 17;
ax_vec = [];

Time = seg1.t_data;
force_data = seg1.Fext_data(1,:);
pos_data = seg1.P_data(1,:);
target_data = seg1.Pg_data(1,:);
pos0_data = P0_data(1,:);
pv_data = Pv_data(1,:);

ax = axes('Parent',fig);
hold(ax, 'on'); 
ax.Box='on';
grid(ax, 'on');
plot(Time, pos_data, 'LineWidth',2.5, 'LineStyle','-', 'Color',[0.6, 0.2, 0], 'DisplayName','modified DMP', 'Parent',ax);
plot(Time0, pos0_data, 'LineWidth',2.5, 'LineStyle','--', 'Color',[1, 0, 1], 'DisplayName','nominal DMP', 'Parent',ax);
plot(Time(1), pos_data(1), 'LineWidth', 3, 'LineStyle','none', 'Color',[0 0.9 0],'Marker','o', 'MarkerSize',10, 'DisplayName','$y_0$', 'Parent',ax, 'HandleVisibility','off');
plot(Time(end), target_data(end), 'LineWidth', 3, 'LineStyle','none', 'Color',[1 0 0],'Marker','x', 'MarkerSize',11, 'DisplayName','$g$', 'Parent',ax, 'HandleVisibility','off');

plot(Timev_data, pv_data, 'LineStyle','none', 'Marker','*', 'MarkerSize',14, 'LineWidth',2.2, 'Color','red', 'DisplayName','via-points');

axis(ax, 'tight');
ax.FontSize = ax_font;
legend({}, 'fontsize',leg_font, 'interpreter','latex', 'Position',[0.0722 0.9341 0.8720 0.0657], 'Orientation','horizontal', 'Box','off');
ylabel("$x$-pos [$m$]", 'fontsize',label_font, 'interpreter','latex');
xlabel("time [$s$]", 'fontsize',label_font, 'interpreter','latex');
axis(ax, 'tight');
ax.YLim = ax.YLim + 0.04*(ax.YLim(2)-ax.YLim(1))*[-1 1];
ax.XLim = ax.XLim + [-0.06 0.08];


%% ======== plot 3D paths ========

target_vp_data = get_target_viapoints(seg2);

y0 = seg1.P_data(1:3, 1);
Q0 = seg1.P_data(4:7, 1);

yg = seg1.P_data(1:3, end);
Qg = seg1.P_data(4:7, end);

label_font = 17;
ax_fontsize = 14;

fig = figure;
fig.Position(3:4) = [815 716];
ax = axes();
hold(ax, 'on')
% -----------------------------------------
plot3(y0(1), y0(2), y0(3), 'LineWidth', 3, 'LineStyle','none', 'Color',[0.1,0.8,0.1],'Marker','o', 'MarkerSize',12, 'DisplayName','$p_0$', 'HandleVisibility','off');
plotFrame(ax, y0, Q0, 'AxisLength',0.1, 'AxisWidth',3);
plot3(yg(1), yg(2), yg(3), 'LineWidth', 2, 'LineStyle','none', 'Color',[1 0.5 0.5],'Marker','x', 'MarkerSize',12, 'DisplayName','$g$', 'HandleVisibility','off');
plotFrame(ax, yg, Qg, 'AxisLength',0.1, 'AxisWidth',4);
% -----------------------------------------
% pick
plot3(seg1.P_data(1,:), seg1.P_data(2,:), seg1.P_data(3,:), 'LineWidth',2.5, 'Color',[0.6, 0.2, 0], 'LineStyle','-', 'DisplayName','modified pick');

% nominal pick
plot3(P0_data(1,:), P0_data(2,:), P0_data(3,:), 'LineWidth',2.5, 'Color',[1, 0, 1], 'LineStyle','--', 'DisplayName','nominal pick');

% via-points
for j=1:size(Pv_data, 2)
   plotFrame(ax, Pv_data(1:3,j), Pv_data(4:7,j), 'AxisLength',0.1, 'AxisWidth',2, ...
       'Xcolor',[1, 0, 0.5], 'Ycolor',[0.4, 1, 0.4], 'Zcolor',[0, 0.5, 1]);
end

% place
plot3(seg2.P_data(1,:), seg2.P_data(2,:), seg2.P_data(3,:), 'LineWidth',2.5, 'Color',[0.85 0.5 0.1], 'LineStyle','-', 'DisplayName','place');

plot3(seg2.Pg_data(1,:), seg2.Pg_data(2,:), seg2.Pg_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color',[1 0.4 0.4], 'HandleVisibility','off');

for j=1:length(target_vp_data)
    opacity = (j/length(target_vp_data))^1.2;
    plotOrientPath(ax, target_vp_data{j}.pos, target_vp_data{j}.quat, ...
        'FrameXcolor',[1, 0, 0.5, opacity], 'FrameYcolor',[0.4, 1, 0.4, opacity], 'FrameZcolor',[0, 0.5, 1, opacity], ...
        'LineWidth',2.5, 'Color',[0.96 0.69 0.13], 'LineStyle','-', ...
        'FrameAxLen',0.1, 'addFrameDist',0.05, 'HandleVisibility','off')
end

% plot obstacle
xb = [0.55 0.9];
yb = [-0.08 0.08];
zb = [0 0.35];

P = [mean(xb), mean(yb), mean(zb)] ; % center point 
L = [xb(2)-xb(1), yb(2)-yb(1), zb(2)-zb(1)] ; % cube dimensions 
plotcube(L, P-L/2, 0.35, [0.4 0.4 0.4], ax);

% plot carton
P = seg1.Pg_data(1:3,end)';
L = [0.16, 0.07, 0.07] ; % cube dimensions 
box_planes = plotcube(L, P-L/2, 0.35, [0.96 0.75 0.4], ax); %[0.96 0.75 0.4]
for i=1:length(box_planes)
    rotate(box_planes{i}, [0 0 1], 60);
    box_planes{i}.XData = box_planes{i}.XData + 0.38;
    box_planes{i}.YData = box_planes{i}.YData + 0.1;
%     box_planes{i}.ZData = box_planes{i}.ZData - P(3);
end
    
% -----------------------------------------
legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.2480 0.7829 0.6346 0.0483], 'Orientation','Horizontal', 'Box','off');
view(106.23, 20.48);
grid on;
% ax.XLim=x_lim; ax.YLim=y_lim; ax.ZLim=z_lim;
ax.FontSize = 14;
xlabel('x [$m$]', 'interpreter','latex', 'fontsize',18);
ylabel('y [$m$]', 'interpreter','latex', 'fontsize',18);
zlabel('z [$m$]', 'interpreter','latex', 'fontsize',18);
% title('Cartesian path', 'interpreter','latex', 'fontsize',20);
hold(ax, 'off');

axis(ax, 'equal');

annotation(fig, 'textbox', 'String','Obstacle', 'Color',[1 0 0], 'FontSize',16, 'LineStyle','none', 'Position',[0.5648 0.4575 0.1508 0.0536]);

annotation(fig, 'textbox', 'String','Carton', 'Color',[0 0 0.8], 'FontSize',16, 'LineStyle','none', 'Position',[0.5610 0.2172 0.1214 0.0536]);
annotation(fig, 'arrow', 'Color',[0 0 0.8], 'LineWidth',2, 'LineStyle','-', 'Position',[0.6497 0.2628 0.0521 0.0588]);

annotation(fig, 'textbox', 'String','Target Box via-points', 'Color',[0 0 0.8], 'FontSize',16, 'LineStyle','none', 'Position',[0.2357 0.2388 0.2134 0.1061]);
annotation(fig, 'arrow', 'Color',[0.4 0.4 0.8], 'LineWidth',2, 'LineStyle','-', 'Position',[0.3127 0.3484 0.0440 0.1058]);
annotation(fig, 'textbox', 'String','initial', 'Color',[0.4 0.4 0.8], 'FontSize',15, 'LineStyle','none', 'Position',[0.2390 0.3729 0.1054 0.0614]);
annotation(fig, 'arrow', 'Color',[0 0 0.8], 'LineWidth',2, 'LineStyle','-', 'Position',[0.4012 0.3371 0.0830 0.0914]);
annotation(fig, 'textbox', 'String','final', 'Color',[0 0 0.8], 'FontSize',15, 'LineStyle','none', 'Position',[0.4270 0.3250 0.0895 0.0614]);

annotation(fig, 'textbox', 'String','via-points', 'Color',[0 0 0.8], 'FontSize',16, 'LineStyle','none', 'Position',[0.8052 0.6833 0.1669 0.0614]);
