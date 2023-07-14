function [Time0, P0_data, dP0_data, ddP0_data] = getModelTrajectory(seg)

import_gmp_lib();
gmp = GMP();
gmp_.read(gmp, 'data/gmp_model_pick.bin', 'pos_');
y0 = seg.P_data(1:3,1);
get_target_fun = @(t) calc_target(t, seg.t_data, seg.Pg_data(1:3,:));
Tf = seg.t_data(end);
dt = 0.005;
[Time0, P0_data, dP0_data, ddP0_data] = simulateModel(gmp, y0, get_target_fun, Tf, dt);

% gmp_o = GMPo();
% gmp_.read(gmp_o, [data_path 'gmp_model_pick.bin'], 'orient_');

end

function g = calc_target(t, Time, Pg_data)

    extrap_val = Pg_data(:,end);
    g = interp1(Time, Pg_data', t, 'linear', nan)';
    
    ind = isnan(g);
    g(ind) = extrap_val(ind);

end

function [Time, Y_data, dY_data, ddY_data, Yg_data] = simulateModel(gmp, y0, get_target_fun, Tf, dt)

    %% set initial values
    n_dofs = length(y0);

    y = y0; % position
    y_dot = zeros(n_dofs,1); % velocity
    y_ddot = zeros(n_dofs,1); % acceleration
    O_ndof = zeros(n_dofs, 1);

    t = 0.0;

    t_end = Tf;
    tau = t_end;

    % phase variable, from 0 to 1
    s = 0.0;
    s_dot = 1/tau;
    s_ddot = 0; % since x_dot is constant here

    iters = 0;

    % data to log
    Time = [];
    Y_data = [];
    dY_data = [];
    ddY_data = [];
    Yg_data = [];
    
    yg = get_target_fun(t);
    
    model = DMP_pp(gmp);
    model.init(s, y0, yg, Tf);
    model.setAdaptToRobot(false);
    % model.setRecursiveUpdate(true);
    
    use_filt = false;
    ag = 200; % goal filter coeff, settling time ~= 4/ag sec
    
    %% simulate
    while (true)

        %% data logging
        Time = [Time t];
        Y_data = [Y_data y];
        dY_data = [dY_data y_dot];
        ddY_data = [ddY_data y_ddot];

        yg_new = get_target_fun(t);
        yg_dot = ag*(yg_new - yg);
        
        if (~use_filt), yg = yg_new; end
        
        Yg_data =[Yg_data yg_new];

        %% Update Model
        model.update(yg, s, s_dot, y, y_dot);
        
        %% Get reference
        y_s = model.getRefPos(s);
        dy_s = model.getRefVel(s, s_dot);
        ddy_s = model.getRefAccel(s, s_dot, s_ddot);

        K = 300; % set the DMP stiffness
        D = 60; % set the DMP damping

        external_signal = 0; % optionally add some external signal

        % Track it using a 2nd order dynamical system. This is actually the DMP. 
        y_ddot = ddy_s + D*(dy_s - y_dot) + K*(y_s - y) + external_signal;

        %% Stopping criteria
        if (t>=1.0*t_end && norm(y-yg)<1e-3 && norm(y_dot)<5e-3)
            break;
        end
        
        if (t >= 1.4*t_end)
            warning('Time limit exceeded!');
            break; 
        end

        %% Numerical integration
        iters = iters + 1;
        t = t + dt;
        s = s + s_dot*dt;
        s_dot = s_dot + s_ddot*dt;
        y = y + y_dot*dt;
        y_dot = y_dot + y_ddot*dt;
        yg = yg + yg_dot*dt;

    end

    fprintf('Error: pos=%e , vel=%e, accel=%e \n', norm(y - yg), norm(y_dot), norm(y_ddot));

end
