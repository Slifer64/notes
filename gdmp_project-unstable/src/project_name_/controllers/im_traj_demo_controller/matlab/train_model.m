clc;
close all;
clear;

import_io_lib();
import_gmp_lib();

filename = 'obstacles_train_data.bin';

data = FileIO(['../config/training/' filename]).readAll();

Timed = data.Timed;
Pd_data = data.Pd_data;
Qd_data = data.Qd_data;

x_data = Timed/Timed(end);

%% ============  position ===========

gmp = GMP(3, 20, 1.5);
train_err = gmp.train('LWR', x_data, Pd_data)

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
end

gmp_.write(gmp, 'obstacles_gmp_model.bin', '');

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
end

% gmp_.write(gmp_o, 'orient_gmp_model.bin', '');


