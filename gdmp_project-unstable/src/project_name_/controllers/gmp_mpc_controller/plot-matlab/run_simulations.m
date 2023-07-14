%% -------- Accumulate data ---------

data = {};

opt_type = 'p';
if (opt_vel), opt_type = 'v'; end

%% --------- Demo -----------
if (demo)
    gmp.setScaleMethod(TrajScale_Prop(n_dof));
    [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, tau, y0, ygd);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
            'color',[0.6 0.6 0.6], 'legend',['$' 'demo' '$'], 'plot3D',true, 'plot2D',false); 
end


%% --------- Proportional scaling -----------
if (dmp_prop)
    gmp.setScaleMethod(TrajScale_Prop(n_dof));
    [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, tau, y0, yg0, yg, t_g);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
            'color','blue', 'legend',['$' 'DMP' '$'], 'plot3D',true, 'plot2D',true); 
end

%% --------- Rotational scaling -----------
if (dmp_rot)
    gmp2 = gmp.deepCopy();
    traj_sc = TrajScale_Rot_wb();
    traj_sc.setWorkBenchNormal([0; 0; 1]);
    gmp2.setScaleMethod(traj_sc);
    [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp2, tau, y0, yg);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
        'color','cyan', 'legend',['$' 'DMP-rot' '$'], 'plot3D',true, 'plot2D',true);
end

%% --------- Offline GMP-weights optimization -----------
if (dmp_opt)
    [Time, P_data, dP_data, ddP_data] = offlineGMPweightsOpt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, vp_config, qp_solver_type);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
            'color',[0.64,0.08,0.18], 'legend',['$' 'DMP^*_' opt_type '$'], 'plot3D',true, 'plot2D',true);
end


%% ---------- Online GMP-weights optimization ------------
% [Time, P_data, dP_data, ddP_data] = onlineGMPweightsOpt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, qp_solver_type);
% data{length(data)+1} = ...
%     struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
%     'color',[1, 0.41, 0.16], 'legend',['opt-w:' opt_type '(online)'], 'plot3D',true, 'plot2D',true);


%% ---------- GMP-MPC optimization ------------
if (dmp_mpc)
    [Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp, tau, y0, yg0, yg, t_g, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, vp_config);

    % %% ------- Load logged data/results ----------
    % log_data = FileIO('../data/log_data.bin', FileIO.in).readAll();
    % Time = log_data.Time;
    % P_data = log_data.P_data;
    % dP_data = log_data.dP_data;
    % ddP_data = log_data.ddP_data;
    % plotSlackVariables(Time, log_data.slack_data, log_data.slack_gains);

    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
        'color',[0.72 0.27 1], 'legend',['$' '\overline{DMP}^*_' opt_type '$'], 'plot3D',true, 'plot2D',true);
end

%% ---------- Offline GMP-trajectory optimization ------------
if (traj_opt)
    [Time, P_data, dP_data, ddP_data] = offlineGMPtrajOpt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, qp_solver_type);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
            'color','green', 'legend',['$' 'traj^*_' opt_type '$'], 'plot3D',true, 'plot2D',true);
end

%% ---------- Online GMP-trajectory optimization ------------
if (traj_mpc)
    [Time, P_data, dP_data, ddP_data] = mpcTrajOpt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, qp_solver_type);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
        'color',[0 0.7 0], 'legend',['$' 'traj-mpc_' opt_type '$'], 'plot3D',true, 'plot2D',true);
end

%% ---------- GMP with repulsive forces ------------
if (dmp_rf)
    [Time, P_data, dP_data, ddP_data] = gmpWithRepulsiveForces(gmp, tau, y0, yg0, yg, t_g, pos_lim, vel_lim, accel_lim);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
        'color',[0.93 0.69 0.13], 'legend',['$' 'DMP-RF' '$'], 'plot3D',true, 'plot2D',true);
end

%% ---------- QP-DMP 2015 ------------
if (dmp_qp)
    [Time, P_data, dP_data, ddP_data] = QP_DMP_opt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
        'color',[1 0 0.65], 'legend',['$' 'DMP-QP' '$'], 'plot3D',true, 'plot2D',true);
end