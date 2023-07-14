clc;
close all;
clear;

import_gmp_lib();
import_io_lib();

fid = FileIO('place_model.bin', FileIO.in);

% fid.printHeader();

gmp_p = GMP();
gmp_.read(gmp_p, fid, 'pos_');

gmp_o = GMPo();
gmp_.read(gmp_o, fid, 'orient_');


n_data = 200;
s_data = linspace(0, 1, n_data);
P_data = zeros(3, n_data);
dP_data = zeros(3, n_data);
ddP_data = zeros(3, n_data);
Q_data = zeros(4, n_data);

Tf = 5;

for j=1:n_data
   s = s_data(j);
   P_data(:, j) = gmp_p.getYd(s);
   dP_data(:, j) = gmp_p.getYdDot(s, 1/Tf);
   ddP_data(:, j) = gmp_p.getYdDDot(s, 1/Tf, 0);
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

