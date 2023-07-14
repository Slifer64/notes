clc;
% close all;
clear;

% Tf = 10;
% sd_dot = 1/Tf;
% Ds = 500;

% s = 0;
% s_dot = 0;

% can_sys_fun = @(t,s,s_dot)my_can_sys_fun(s, s_dot, sd_dot, Ds);
% state0 = [s; s_dot];

%[Time,state_data] = ode15s(@(t,state)ode_fun(t,state, can_sys_fun), [0 12], state0);

% Time = linspace(0, 12, 100);
% [Time,state_data] = ode15s(@(t,state)ode_fun(t,state, can_sys_fun), Time, state0);
% 
% Time = Time';
% s_data = state_data(:,1)';
% sdot_data = state_data(:,2)';

t = 0;
dt = 0.05;

Time = [];
s_data = [];
sdot_data = [];

Tf0 = 10;
Tf_new = 12;
t_change = 6;
made_change = false;

% Tf = Tf0;

can_sys = CanonicalSystem(Tf0, 30);

while (true)
    
    if (t > 1.2*max(Tf0,Tf_new)), break; end
    
    if (t >= t_change && ~made_change )
        can_sys.setDuration(Tf_new, t);
        made_change = true;
    end
    
    Time = [Time t];
    s_data = [s_data can_sys.s];
    sdot_data = [sdot_data can_sys.s_dot];
    
%     can_sys_fun = @(t,s,s_dot)my_can_sys_fun(s, s_dot, sd_dot, Ds);

%     [~,state_data] = ode15s(@(t,state)ode_fun(t,state, can_sys_fun), [t t+dt], [s; s_dot]);
    
%     n_steps = 10; %(dt/0.01)
%     t_ = linspace(t, t+dt, n_steps);
%     state_data = zeros(n_steps,2);
%     sj = s;
%     sj_dot = s_dot;
%     for j=1:length(t_)-1
%         state_data(j,:) = [sj, sj_dot];
%         [sj_dot, sj_ddot] = can_sys_fun(t_(j), sj, sj_dot);
%         dtj = t_(j+1) - t_(j);
%         sj = sj + sj_dot * dtj;
%         sj_dot = sj_dot + sj_ddot * dtj;
%     end
%     state_data(end,:) = [sj, sj_dot];    
    
    can_sys.integrate(t, t+dt);
    t = t+dt;
    
%     s = state_data(end,1);
%     s_dot = state_data(end,2);
    
end

sdot_hat_data = diff(s_data) ./ diff(Time);
sdot_hat_data = [0 sdot_hat_data];

figure;
% ------------------------------------
ax = subplot(2,1,1); hold on;
plot(Time, s_data, 'LineWidth',1.5);
plot([t_change t_change], ax.YLim, 'LineWidth',1, 'LineStyle','--', 'Color','green');
plot([Tf0 Tf0], ax.YLim, 'LineWidth',1, 'LineStyle','--', 'Color','red');
plot([Tf_new Tf_new], ax.YLim, 'LineWidth',1, 'LineStyle','--', 'Color','magenta');
plot(ax.XLim, [1 1], 'LineWidth',1.5, 'LineStyle',':', 'Color',[0.3 0.3 0.3 0.5]);
ylabel('$s$', 'interpreter','latex', 'fontsize',17);
% ------------------------------------
ax = subplot(2,1,2); hold on;
plot(Time, sdot_data, 'LineWidth',1.5);
plot(Time, sdot_hat_data, 'LineWidth',1.5, 'LineStyle',':', 'Color',[0.85 0.33 0.1]);
plot([Tf0 Tf0], ax.YLim, 'LineWidth',1, 'LineStyle','--', 'Color','red');
plot([Tf_new Tf_new], ax.YLim, 'LineWidth',1, 'LineStyle','--', 'Color','magenta');
ylabel('$\dot{s}$', 'interpreter','latex', 'fontsize',17);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);

%% =================================================================

function [s_dot, s_ddot] = my_can_sys_fun(s, s_dot, sd_dot, Ds)

    s_dot = s_dot;
    
    if (s < 1), s_ddot = -Ds*(s_dot - sd_dot);
    else,       s_ddot = -400*s_dot - 800*(s-1);            
    end

end

function state_dot = ode_fun(t, state, can_sys_fun)

    s = state(1);
    s_dot = state(2);
    
    [s_dot, s_ddot] = can_sys_fun(t, s, s_dot);
    
    state_dot = [s_dot; s_ddot];

end