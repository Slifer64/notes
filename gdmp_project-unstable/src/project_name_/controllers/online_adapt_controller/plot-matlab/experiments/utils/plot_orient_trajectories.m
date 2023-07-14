function plot_orient_trajectories(seg1, seg2, plot_vel, plot_accel)
    
    seg = {seg1, seg2};
    Q_prev = seg{1}.P_data(4:end,1);
    for k=1:length(seg)
        Q_data = seg{k}.P_data(4:end,:);
        Qg_data = seg{k}.Pg_data(4:end,:);
        qLog_data = zeros(3, size(Q_data,2));
        qgLog_data = zeros(3, size(Q_data,2));
        for j=1:size(qLog_data,2)
            Q = Q_data(:,j);
            Qg = Qg_data(:,j);
            if (dot(Q_prev, Q)<0), Q=-Q; end
            if (dot(Q_prev, Qg)<0), Qg=-Qg; end
            Q_prev = Q;
            qLog_data(:,j) = math_.quatLog(Q);
            qgLog_data(:,j) = math_.quatLog(Qg);
        end

        seg{k}.P_data = qLog_data;
        seg{k}.V_data = seg{k}.V_data(4:end,:);
        seg{k}.V_dot_data = seg{k}.V_dot_data(4:end,:);
        seg{k}.Pg_data = qgLog_data;
    end
    
    plot_pos_trajectories(seg{1}, seg{2}, plot_vel, plot_accel, 0);
    
end
