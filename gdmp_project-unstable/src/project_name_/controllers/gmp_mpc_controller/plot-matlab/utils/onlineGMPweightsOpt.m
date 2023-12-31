function [Time, P_data, dP_data, ddP_data] = onlineGMPweightsOpt(gmp0, tau, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, qp_solver_type)
      
    gmp = gmp0.deepCopy();
   
    n_dof = length(y0);
    
    %% --------  Init sim  --------
    gmp.setScaleMethod(TrajScale_Prop(n_dof));
    gmp.setY0(y0);
    gmp.setGoal(yg);
    
    N_kernels = 30; %gmp0.numOfKernels();
    s_data = 0:0.01:1;
    Yd_data = zeros(n_dof, length(s_data));
    for j=1:length(s_data), Yd_data(:,j)=gmp.getYd(s_data(j)); end
    gmp = GMP(n_dof, N_kernels, 1.5);
    gmp.train('LS', s_data, Yd_data);
    gmp.setTruncatedKernels(1e-8);
    
    Time = [];
    P_data = [];
    dP_data = [];
    ddP_data = [];
    
    slack_data = [];

    y = y0;
    y_dot = zeros(size(y));
    y_ddot = zeros(size(y));
    
    K = 300;
    D = 80;

    t = 0;
    dt = 0.005;
    s = t/tau;
    s_dot = 1/tau;
    s_ddot = 0;

    n_dof3 = 3*n_dof; % for pos, vel, accel
    
    %% --------  Init MPC  --------
    N = 10; %15;    
%     dt_ = dt * (1:N).^0.9;
    dt_ = 0.02*ones(1,N); %dt;
    
    N_kernels = gmp.numOfKernels();
    
    pos_slack = 0;
    vel_slack = 0;
    accel_slack = 0;
    n_slack = pos_slack + vel_slack + accel_slack;
    
    Aineq_slack = [];
    Q_slack = [];
    if (pos_slack)
        Q_slack = blkdiag(Q_slack, 1e6); 
        Aineq_slack = [Aineq_slack [-ones(n_dof,1); zeros(2*n_dof,1)] ];
    end
    if (vel_slack)
        Q_slack = blkdiag(Q_slack, 100);
        Aineq_slack = [Aineq_slack [zeros(n_dof,1); -ones(n_dof,1); zeros(n_dof,1)] ];
    end
    if (accel_slack)
        Q_slack = blkdiag(Q_slack, 20);
        Aineq_slack = [Aineq_slack [zeros(2*n_dof,1); -ones(n_dof,1)] ];
    end
    Q_slack = sparse(Q_slack);
    Aineq_slack = sparse(Aineq_slack);
    
    n = n_dof * N_kernels + n_slack;
    
    % State tracking gains: (x(i) - xd(i))'*Qi*(x(i) - xd(i))
    Qi = blkdiag( opt_pos*speye(n_dof,n_dof) , opt_vel*10*speye(n_dof,n_dof));
    QN = blkdiag( 100*speye(n_dof,n_dof) , 1*speye(n_dof,n_dof));

    z_min = [pos_lim(:,1); vel_lim(:,1); accel_lim(:,1)];
    z_max = [pos_lim(:,2); vel_lim(:,2); accel_lim(:,2)];
    
    % initial guess for weights
    % w = gmp.W'; 
    %Z0 = [w; zeros(n_slack,1)];
    Z0 = zeros(n,1);
    n_ineq = N*n_dof3 + n_slack;
    n_eq = n_dof3;
    Z0_dual_ineq = zeros(n_ineq, 1);
    Z0_dual_eq = zeros(n_eq, 1);
    
    %% --------  Init solver  --------
    if (qp_solver_type == 0)
        solver_opt = optimoptions('quadprog', 'Algorithm','interior-point-convex', 'LinearSolver','sparse', 'StepTolerance',0, 'Display','off', 'MaxIterations',2000);
    end
    
    text_prog = ProgressText(40);
    text_prog.init();
    
    t_start = tic;

    phi0 = gmp.regressVec(s);
    phi0_dot = gmp.regressVecDot(s, s_dot);
    phi0_ddot = gmp.regressVecDDot(s, s_dot, s_ddot);
    Phi0 = [kron(speye(n_dof),phi0'); kron(speye(n_dof),phi0_dot'); kron(speye(n_dof),phi0_ddot')];
    x0 = [y0; zeros(n_dof,1); zeros(n_dof,1)];
    
    phi_f = gmp.regressVec(1);
    phi_f_dot = gmp.regressVecDot(1, 0);
    phi_f_ddot = gmp.regressVecDDot(1, 0, 0);
    Phi_f = sparse([kron(eye(n_dof),phi_f'); kron(eye(n_dof),phi_f_dot'); kron(eye(n_dof),phi_f_ddot')]);
    x_final = [yg; zeros(n_dof,1); zeros(n_dof,1)];
    
    s_vp = [1];
    Phi_vp = {Phi_f};
    x_vp = {x_final};
    
    Aeq = [Phi0, sparse(n_dof3, n_slack)];
    beq = x0;
    
    %% --------  Simulation loop  --------
    while (true)
        
        %% --------  Stopping criteria  --------
        if (s > 1.0), break; end
        
        text_prog.update(100*t/tau);
        
        if (s >= 1)
            s = 1;
            s_dot = 0;
            s_ddot = 0;
        end

        %% -------  calc problem matrices  --------
        
%         % restore bounds in case they were changed in previous iter for si>=1
%         Z_min = repmat(z_min, N,1);
%         Z_max = repmat(z_max, N,1);
        
        H = sparse(n,n);
        q = zeros(n,1);
        % add slacks to bounds. I.e. one could optionaly define with slacks
        % bounds the maximum allowable error
        Aineq = zeros(N*n_dof3 + n_slack, n);
        Aineq(end-n_slack+1:end,end-n_slack+1:end) = eye(n_slack);
        
        % DMP phase variable
        si = s;
        si_dot = s_dot;
        si_ddot = s_ddot;
        
        for i=1:N
            
%             if (si > 1)
%                 N = i-1;
%                 break;
%             end

            yd_i = gmp.getYd(si);
            dyd_i = gmp.getYdDot(si, si_dot);
            
            phi = gmp.regressVec(si);
            phi_dot = gmp.regressVecDot(si, si_dot);
            phi_ddot = gmp.regressVecDDot(si, si_dot, si_ddot);

            if (i==N), Qi_ = QN;
            else, Qi_ = Qi;
            end
            
            Psi = [kron(speye(n_dof),phi'); kron(speye(n_dof),phi_dot')];
            xd_i = [yd_i; dyd_i];

            H = H + blkdiag(Psi'*Qi_*Psi, Q_slack);
            q = q - [Psi'*Qi_*xd_i; zeros(n_slack,1)];
            
            Aineq_i = [kron(speye(n_dof),phi'); kron(speye(n_dof),phi_dot'); kron(speye(n_dof),phi_ddot')];
            Aineq((i-1)*n_dof3+1 : i*n_dof3, :) = [Aineq_i, Aineq_slack];
            
            si = si + si_dot*dt_(i);
            si_dot = si_dot + si_ddot*dt_(i);
            % si_ddot = ... (if it changes too)
        end
                      
        H = 1e-6*speye(n) + (H+H')/2; % to account for numerical errors
        Aineq = sparse(Aineq);
        
%         size(H,1)*size(H,2)
%         nnz(H)
%         
%         size(Aineq,1)*size(Aineq,2)
%         nnz(Aineq)
%         
% %         Aineq
% %         H
%         
%         pause
        
        % Here we set the slacks to be unbounded, but in general, we could
        % specify as bounds the maximum allowable violation from the
        % kinematic bounds.
        Z_min = [repmat(z_min, N,1); -inf(n_slack,1)];
        Z_max = [repmat(z_max, N,1); inf(n_slack,1)];
        
%         % This is required when horizon N shrinks towards the end
%         % take all constraints up to the new horizon N and append the
%         % identity for the slack bounds constraints
%         Aineq = Aineq(1:N*n_dof3,:); 
%         Aineq = [Aineq; [sparse(n_slack, n_dof*N_kernels) speye(n_slack)] ];
%         
%         if (size(Aineq,1) < length(Z0_dual_ineq))
%            z_slack = Z0_dual_ineq(end-n_slack+1:end);
%            Z0_dual_ineq = [Z0_dual_ineq(1:size(Aineq,1)-n_slack); z_slack];
%         end
        
        Aeq(1:n_dof3,:) = [Phi0, sparse(n_dof3, n_slack)];
        beq(1:n_dof3) = x0;
        
        i_vp = find( (s_vp>0) && (s_vp<=si) );
        if (~isempty(i_vp))
            for k=1:length(i_vp)
                i = i_vp(k);
                Aeq = [Aeq; [Phi_vp{i}, sparse(n_dof3, n_slack)] ];
                beq = [beq; x_vp{i}];
            end
            s_vp(i_vp) = -1;
   
            n_eq_plus = size(Aeq,1) - length(Z0_dual_eq);
            if (n_eq_plus>0), Z0_dual_eq = [Z0_dual_eq; zeros(n_eq_plus, 1)]; end
        end
        
        %% ===========  solve optimization problem  ==========

        %% --------- OSQP solver ----------
        if (qp_solver_type == 1)
            
            A_osqp = [Aineq; Aeq];
            lb = [Z_min; beq];
            ub = [Z_max; beq];

            Z0_dual = [Z0_dual_ineq; Z0_dual_eq];
    
            % Create an OSQP object
            osqp_solver = osqp;
            osqp_solver.setup(H, q, A_osqp, lb, ub, 'warm_start',false, 'verbose',false, 'eps_abs',1e-4, 'eps_rel',1e-5);%, 'max_iter',20000);
            osqp_solver.warm_start('x', Z0, 'y',Z0_dual);
            
            res = osqp_solver.solve();

            if ( res.info.status_val ~= 1)
                %res.info
                warning(res.info.status);
                text_prog.printInNewLine();
                if (res.info.status_val == -3 || res.info.status_val == -4 || res.info.status_val == -7 || res.info.status_val == -10), break; end
            end
            
            Z = res.x;
            Z0 = Z;
            Z0_dual = res.y;
            n_ineq = size(Aineq,1);
            Z0_dual_ineq = Z0_dual(1:n_ineq);
            Z0_dual_eq = Z0_dual(n_ineq+1:end);
            
        end

        %% --------- matlab solver ----------
        if (qp_solver_type == 0)

            A = [Aineq; -Aineq];
            b = [Z_max; -Z_min];
            
            [Z, ~, ex_flag, opt_output] = quadprog(H,q, A,b, Aeq,beq, [],[], Z0, solver_opt);
            
            if (ex_flag == 1 || ex_flag == 2)
                % success
            else
                warning(opt_output.message);
                text_prog.printInNewLine();
                if (ex_flag < 0), break; end
            end
            
            Z0 = Z;

        end
        
        if (qp_solver_type == 2)

            A = [Aineq; -Aineq];
            b = [Z_max; -Z_min]; 
            lb = -inf(n,1);
            ub = inf(n,1);
            
            % option.maxiter = 1000;
            [Z,~,ex_flag,opt_output] = qpGoldfarbIdnani(full(H),q, full(A),b, full(Aeq),beq, lb,ub); %,option)
            
            if (ex_flag == 1)
                % success
            else
                warning(opt_output.status);
                text_prog.printInNewLine();
                if (ex_flag < 0), break; end
            end
            
        end
        
        
        w = Z(1:end-n_slack);
        slack_var = Z(end-n_slack+1:end);
        W = reshape(w, N_kernels, n_dof)';

        
        %% --------  Simulate dynamics  --------
        
        yd = W*gmp.regressVec(s);
        yd_dot = W*gmp.regressVecDot(s, s_dot);
        yd_ddot = W*gmp.regressVecDDot(s, s_dot, s_ddot);

%         y_ddot = -K*(y-yd) -D*(y_dot-yd_dot) + yd_ddot;
        
        y = yd;
        y_dot = yd_dot;
        y_ddot = yd_ddot;
        
        y0_ = y;
        y0_dot = y_dot;
        y0_ddot = y_ddot;
        phi0 = gmp.regressVec(s);
        phi0_dot = gmp.regressVecDot(s, s_dot);
        phi0_ddot = gmp.regressVecDDot(s, s_dot, s_ddot);
        Phi0 = [kron(speye(n_dof),phi0'); kron(speye(n_dof),phi0_dot'); kron(speye(n_dof),phi0_ddot')];
        x0 = [y; y_dot; y_ddot];
        
        %% --------  Log data  --------
        Time = [Time t];
        P_data = [P_data y];
        dP_data = [dP_data y_dot];
        ddP_data = [ddP_data y_ddot];
        
        slack_data = [slack_data slack_var];
    
        %% --------  Numerical integration  --------
        t = t + dt;
        s = s + s_dot*dt;
        s_dot = s_dot + s_ddot*dt;
        y = y + y_dot*dt;
        y_dot = y_dot + y_ddot*dt;
    
    end

%     Time = [Time tau];
%     P_data = [P_data W*gmp.regressVec(1)];
%     dP_data = [dP_data W*gmp.regressVecDot(1, 0)];
%     ddP_data = [ddP_data W*gmp.regressVecDDot(1, 0, 0)];

    if (n_slack)
        y_lb = {};
        if (pos_slack), y_lb = [y_lb, {'pos'}]; end
        if (vel_slack), y_lb = [y_lb, {'vel'}]; end
        if (accel_slack), y_lb = [y_lb, {'accel'}]; end
        figure;
        for i=1:n_slack
            subplot(n_slack,1,i);
            plot(Time, slack_data(i,:), 'LineWidth',2, 'Color','red');
            ylabel(y_lb{i}, 'fontsize',15);
            if (i==1), title('slack variables', 'fontsize',17); end
        end
    end
    
    text_prog.update(100);
    fprintf('\n');
    fprintf('===> online-GMP-weights optimization finished! Elaps time: %f ms\n',toc(t_start)*1000);

end
