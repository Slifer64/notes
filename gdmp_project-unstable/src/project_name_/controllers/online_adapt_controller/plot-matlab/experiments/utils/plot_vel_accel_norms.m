function plot_vel_accel_norms(forward, reverse)

forward = calcVelAccelNorm(forward);
reverse = calcVelAccelNorm(reverse);

if (reverse.t_data(1) ~= forward.t_data(end))
    reverse.t_data = reverse.t_data - reverse.t_data(1) + forward.t_data(end);
end

figure;
subplot(2, 2, 1); hold on;
plot(forward.t_data, forward.v_norm, 'LineWidth',2, 'Color','blue');
plot(reverse.t_data, reverse.v_norm, 'LineWidth',2, 'LineStyle','-', 'Color','cyan');
ylabel('vel $m/s$', 'interpreter','latex', 'fontsize',14);
title('Translation', 'interpreter','latex', 'fontsize',16);
axis tight;
subplot(2, 2, 2); hold on;
plot(forward.t_data, forward.vRot_norm, 'LineWidth',2, 'Color','blue');
plot(reverse.t_data, reverse.vRot_norm, 'LineWidth',2, 'LineStyle','-', 'Color','cyan');
ylabel('rot-vel $rad/s$', 'interpreter','latex', 'fontsize',14);
title('Rotation', 'interpreter','latex', 'fontsize',16);
axis tight;
legend({'forward','reverse'}, 'interpreter','latex', 'fontsize',16);
subplot(2, 2, 3); hold on;
plot(forward.t_data, forward.vdot_norm, 'LineWidth',2, 'Color','blue');
plot(reverse.t_data, reverse.vdot_norm, 'LineWidth',2, 'LineStyle','-', 'Color','cyan');
ylabel('accel $m/s^2$', 'interpreter','latex', 'fontsize',14);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
axis tight;
subplot(2, 2, 4); hold on;
plot(forward.t_data, forward.vRot_dot_norm, 'LineWidth',2, 'Color','blue');
plot(reverse.t_data, reverse.vRot_dot_norm, 'LineWidth',2, 'LineStyle','-', 'Color','cyan');
ylabel('rot-accel $rad/s^2$', 'interpreter','latex', 'fontsize',14);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
axis tight;

end


function data = calcVelAccelNorm(data)

    data.v_norm = vecnorm(data.Vd_data(1:3, :), 2, 1);
    data.vdot_norm = vecnorm(data.Vd_dot_data(1:3, :), 2, 1);
    data.vRot_norm = vecnorm(data.Vd_data(4:6, :), 2, 1);
    data.vRot_dot_norm = vecnorm(data.Vd_dot_data(4:6, :), 2, 1);

end
