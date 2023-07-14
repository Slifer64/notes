function plot_pos_trajectories(seg1, seg2, plot_vel, plot_accel, is_pos)
  
if (nargin < 5), is_pos = 1; end

if (is_pos)
    pos_ylabel = 'pos [$m$]';
    vel_ylabel = 'vel [$m/s$]';
    accel_ylabel = 'accel [$m/s^2$]';
else
    pos_ylabel = 'log$(Q)$';
    vel_ylabel = 'vel [$rad/s$]';
    accel_ylabel = 'accel [$rad/s^2$]';
end


n_rows = 1 + plot_vel + plot_accel;

if (seg2.t_data(1) ~= seg1.t_data(end))
    seg2.t_data = seg2.t_data - seg2.t_data(1) + seg1.t_data(end);
end

y0_pick = seg1.P_data(:,1);
yg_pick = seg1.Pg_data(:,end);

y0_place = seg2.P_data(:,end);
yg_place = seg2.Pg_data(:,end);


title_ = {'$X$-axis', '$Y$-axis', '$Z$-axis'};
ax_fontsize = 12;
label_font = 16;

fig = figure;
fig.Position(3:4) = [1050 638];
% ======== position ==========

ax_ind = [1 2 3];
m = length(ax_ind);
sp_ind = 1:m:(2*m+1);
for i1=1:m
    
    i = ax_ind(i1);
    
    ax = subplot(n_rows,m,sp_ind(1), 'Parent',fig);
    
    ax_vec = [ax];
    hold(ax, 'on'); ax.Box='on';
    % plot position trajectory
    plot(seg1.t_data, seg1.P_data(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color','blue', 'HandleVisibility', 'off', 'Parent',ax);
    plot(seg2.t_data, seg2.P_data(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color','cyan', 'HandleVisibility', 'off', 'Parent',ax);
    axis(ax, 'tight');
    ax.FontSize = ax_fontsize;
    % plot start and final positions
    plot(seg1.t_data(1), y0_pick(i), 'LineWidth', 3, 'LineStyle','none', 'Color',[0 0.9 0],'Marker','o', 'MarkerSize',10, 'HandleVisibility','off', 'Parent',ax);
    
    plot(seg1.t_data, seg1.Pg_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color',[1 0.5 0.5], 'HandleVisibility', 'off', 'Parent',ax);
    plot(seg1.t_data(end), yg_pick(i), 'LineWidth', 3, 'LineStyle','none', 'Color',[1 0 0],'Marker','x', 'MarkerSize',11, 'HandleVisibility','off', 'Parent',ax);
    
    plot(seg2.t_data, seg2.Pg_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color',[1 0.5 1], 'HandleVisibility', 'off', 'Parent',ax);
    plot(seg2.t_data(end), yg_place(i), 'LineWidth', 3, 'LineStyle','none', 'Color',[1 0 1],'Marker','x', 'MarkerSize',11, 'HandleVisibility','off', 'Parent',ax);
    
    % labels, title ...
    if (i1==1), ylabel(pos_ylabel, 'interpreter','latex', 'fontsize',label_font, 'Parent',ax); end
    title(title_{i}, 'interpreter','latex', 'fontsize',18, 'Parent',ax);
    %if (i1==1), legend(ax, {}, 'interpreter','latex', 'fontsize',17, 'Position',[0.3815 0.9458 0.2718 0.0416], 'Orientation', 'horizontal'); end
    hold(ax,'off');

    % ======== velocity ==========
    if (plot_vel)
        ax = subplot(n_rows,m,sp_ind(2), 'Parent',fig);
        ax_vec = [ax_vec ax];
        hold(ax, 'on'); ax.Box='on';
        ax.FontSize = ax_fontsize;
        %plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'Parent',ax);
        plot([0 seg2.t_data(end)], [0 0], 'LineWidth', 2, 'LineStyle',':', 'Color',0.4*[1 1 1], 'Parent',ax);
        plot(seg1.t_data, seg1.V_data(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color','blue', 'Parent',ax);
        plot(seg2.t_data, seg2.V_data(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color','cyan', 'Parent',ax);
        axis(ax, 'tight');
        if (i1==1), ylabel(vel_ylabel, 'interpreter','latex', 'fontsize',label_font, 'Parent',ax); end
        hold(ax,'off');
    end

    % ======== acceleration ==========
    if (plot_accel)
        ax = subplot(n_rows,m,sp_ind(3), 'Parent',fig);
        ax_vec = [ax_vec ax];
        hold(ax, 'on'); ax.Box='on';
        ax.FontSize = ax_fontsize;
        plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',10, 'Parent',ax);
        plot(Time, Accel(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color','red', 'Parent',ax);
        axis(ax, 'tight');
        if (i1==1), ylabel(accel_ylabel, 'interpreter','latex', 'fontsize',label_font, 'Parent',ax); end
        hold(ax, 'off');
    end
    
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
    
    linkaxes(ax_vec,'x');
    
    sp_ind = sp_ind + 1;

end

% create legend:
% ax = axes('Parent',fig, 'Position',[-5 -5 0.01 0.01]);
ax = axes('Parent',fig, 'Position',[0 0 0.5 0.5], 'color','none', 'XTick',[], 'YTick',[]);
hold(ax, 'on')
s1_pl = plot(nan, nan, 'LineWidth',2.5, 'LineStyle','-', 'Color','blue', 'DisplayName','pick DMP');
s2_pl = plot(nan, nan, 'LineWidth',2.5, 'LineStyle','-', 'Color','cyan', 'DisplayName','place DMP');
g1_pl = plot(nan, nan, 'LineWidth',2, 'LineStyle','-', 'Color',[1 0.5 0.5], 'DisplayName','pick target');
g2_pl = plot(nan, nan, 'LineWidth',2, 'LineStyle','-', 'Color',[1 0.5 1], 'DisplayName','place target');
lg = legend(ax, {}, 'interpreter','latex', 'fontsize',15, 'Orientation','vertical', 'Position',[0.8476 0.8336 0.1470 0.1591], 'Box','off');

end
