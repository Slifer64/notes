clc;
close all;
clear;

%% =============  includes...  =============
import_gmp_lib();
addpath('utils/')

%% =============  Load demo data  =============                           
load('data/demo_data2.mat', 'Timed', 'Pd_data');

n_dof = size(Pd_data, 1);
% calc vel/accel numerically to plot them later
dPd_data = [diff(Pd_data, 1, 2)./diff(Timed) zeros(n_dof,1)];
ddPd_data = [diff(dPd_data, 1, 2)./diff(Timed) zeros(n_dof,1)];

%% ============= Train DMP  =============
gmp = trainDMP(Timed, Pd_data);


%% =============  DMP simulation  =============
disp('Simulation...');
t_start = tic;

%% Initial/Final values
y0d = Pd_data(:,1);   % Initial demo position
gd = Pd_data(:,end); % Target demo position
y0 = y0d; % set initial position for execution (for simplicity lets leave it the same as the demo)
g = gd + 0.0;  % set target position for execution
Tf = Timed(end); % set the time duration of the executed motion

dt = 0.005; % time step for numerical integration

via_points = {struct('s',0.38, 'pos',0.4), struct('s',0.74, 'pos',0.7)};

%% Simulate DMP
% [Time, P_data, dP_data, ddP_data, Pg_data] = simulateModel(gmp, y0, get_target_fun, Tf, dt);
% [Time2, P2_data, dP2_data, ddP2_data, Pg2_data] = simulateDMP(gmp, y0, get_target_fun, Tf, dt);


[Time, P_data, dP_data, ddP_data] = simulateModel(gmp, y0, g, Tf, dt, via_points, 'accel');
dat{1} = struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, ...
                'Color','blue', 'LineStyle','-', 'DisplayName','DMP$^{++}$');
            
[Time, P_data, dP_data, ddP_data] = simulateModel(gmp, y0, g, Tf, dt, via_points, 'pos');
dat{2} = struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, ...
                'Color',[0.96 0.69 0.13], 'LineStyle','-.', 'DisplayName','DMP$^{++}_p$');
            
[Time, P_data, dP_data, ddP_data] = simulateModel(gmp, y0, g, Tf, dt, via_points, 'weights');
dat{3} = struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, ...
                'Color','magenta', 'LineStyle',':', 'DisplayName','DMP$^{++}_w$');
            
dat{4} = struct('Time',Timed, 'Pos',Pd_data, 'Vel',dPd_data, 'Accel',ddPd_data, ...
                'Color',[0 0.9 0], 'LineStyle',':', 'DisplayName','demo');

t_v = [];
p_v = [];
for k=1:length(via_points)
    t_v = [t_v via_points{k}.s*Tf];
    p_v = [p_v via_points{k}.pos];
end

toc(t_start)
    
%% Plot results

ax_font = 13;
x_font = 16;
y_font = 16;
legend_font = 17;


for i=1:n_dof
    fig = figure; fig.Position(3:4) = [656 679];
    ax_vec = [];
    % Plot Position
    ax = subplot(3,1,1); hold(ax, 'on');
    ax_vec = [ax_vec ax];
    for k=1:length(dat)
        plot(dat{k}.Time, dat{k}.Pos(i,:), 'LineWidth',2.0 , 'LineStyle',dat{k}.LineStyle, 'Color',dat{k}.Color, 'DisplayName', dat{k}.DisplayName);
    end
    %plot(Time(end), gd(i), 'LineWidth',3, 'Marker','x', 'MarkerSize',12, 'LineStyle','none', 'Color',[1 0.6 0.6], 'DisplayName','$g_d$');
    plot(Time(end), g(i), 'LineWidth',3, 'Marker','x', 'MarkerSize',12, 'LineStyle','none', 'Color','red', 'HandleVisibility','off'); %, 'DisplayName','$g$');
    if (~isempty(t_v))
       plot(t_v, p_v(i,:), 'LineWidth',2, 'LineStyle','none', 'Marker','*', 'MarkerSize',14, 'Color',[1 0.2 0.2], 'DisplayName','via-points');
    end
    ax.FontSize = ax_font;
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',y_font);
    legend({}, 'interpreter','latex', 'fontsize',legend_font, 'Position',[0.2251 0.9396 0.5708 0.0425], 'Orientation','horizontal', 'Box','off');
    axis tight;
    xlim([Time(1) Time(end)]);
    hold off;
    
    % Plot Velocity
    ax = subplot(3,1,2); hold(ax, 'on');
    ax_vec = [ax_vec ax];
    plot([Time(1) Time(end)], [0 0], 'LineWidth',1.0, 'LineStyle',':', 'Color',0.4*[1 1 1]);
    for k=1:length(dat)
        plot(dat{k}.Time, dat{k}.Vel(i,:), 'LineWidth',2.0 , 'LineStyle',dat{k}.LineStyle, 'Color',dat{k}.Color, 'DisplayName', dat{k}.DisplayName);
    end
    ax.FontSize = ax_font;
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',y_font);
    axis tight;
    xlim([Time(1) Time(end)]);
    hold off;
    
    % Plot Acceleration
    ax = subplot(3,1,3); hold(ax, 'on');
    ax_vec = [ax_vec ax];
    plot([Time(1) Time(end)], [0 0], 'LineWidth',1.0, 'LineStyle',':', 'Color',0.4*[1 1 1]);
    for k=1:length(dat)
        plot(dat{k}.Time, dat{k}.Accel(i,:), 'LineWidth',2.0 , 'LineStyle',dat{k}.LineStyle, 'Color',dat{k}.Color, 'DisplayName', dat{k}.DisplayName);
    end
    ax.FontSize = ax_font;
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',y_font);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',x_font);
    axis tight;
    xlim([Time(1) Time(end)]);
    hold off;
    
    linkaxes(ax_vec, 'x');
    for i=1:length(ax_vec)
       ax = ax_vec(i);
       ax.Box = 'on';
       ax.YLim = ax.YLim + 0.07*(ax.YLim(2)-ax.YLim(1))*[-1 1];
    end
    ax = ax_vec(i);
    ax.XLim(2) = ax.XLim(2) + 0.05;
end


%% ============================================================
%% ============================================================

function [Time, Y_data, dY_data, ddY_data] = simulateModel(gmp, y0, yg, Tf, dt, via_points, opt_type)

    %% set initial values
    n_dofs = length(y0);

    y = y0; % position
    y_dot = zeros(n_dofs,1); % velocity
    y_ddot = zeros(n_dofs,1); % acceleration

    t = 0.0;
    t_end = Tf;
    tau = t_end;

    % phase variable, from 0 to 1
    s = 0.0;
    s_dot = 1/tau;
    s_ddot = 0; % since x_dot is constant here

    % data to log
    Time = [];
    Y_data = [];
    dY_data = [];
    ddY_data = [];
    
    model = DMP_pp(gmp);
    
    % adapt the weights to scale to new init/target position
    model.init(s, y0, yg, Tf);
    
    % model.setAdaptToRobot(false);
    % model.setRecursiveUpdate(true);
    
    % reset the optimization metric: This will now use the adapted weights
    % for the desired acceleration profile.
    model.setOptMetric(opt_type);
    % reinitialize
    model.init(s, y0, yg, Tf);
    
    % update weights to pass from the via-points
    for k=1:length(via_points)
        model.updateViapoint(via_points{k}.s, via_points{k}.pos, false);
    end
    
    %% simulate
    while (true)

        %% data logging
        Time = [Time t];
        Y_data = [Y_data y];
        dY_data = [dY_data y_dot];
        ddY_data = [ddY_data y_ddot];

        %% Update DMP_pp
        %model.update(yg, s, s_dot, y, y_dot);
        
        %% Get reference
        y_s = model.getRefPos(s);
        dy_s = model.getRefVel(s, s_dot);
        ddy_s = model.getRefAccel(s, s_dot, s_ddot);

        % GMP equation: 
        y_ddot = ddy_s + 60*(dy_s - y_dot) + 300*(y_s - y);

        %% Stopping criteria
        if (t>=1.0*t_end && norm(y-yg)<1e-3 && norm(y_dot)<5e-3)
            break;
        end
        
        if (t >= 1.4*t_end)
            warning('Time limit exceeded!');
            break; 
        end

        %% Numerical integration
        t = t + dt;
        s = s + s_dot*dt;
        s_dot = s_dot + s_ddot*dt;
        y = y + y_dot*dt;
        y_dot = y_dot + y_ddot*dt;

    end

    fprintf('Error: pos=%e , vel=%e, accel=%e \n', norm(y - yg), norm(y_dot), norm(y_ddot));

end

