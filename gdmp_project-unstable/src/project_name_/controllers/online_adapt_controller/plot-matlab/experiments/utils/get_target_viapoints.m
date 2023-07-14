function target_vp_data = get_target_viapoints(log)

    vp_offsets = [0.15, 0.12, 0.08, 0.05, 0.03, 0.015, 0.0];

    Pg_data = log.Pg_data(1:3, :);
    Qg_data = log.Pg_data(4:7, :);

    % dist_thres = 0.3;
    % dist = 0;
    % ind = [];
    % for j=2:size(Pg_data,2)
    %     dist = dist + norm(Pg_data(:,j) - Pg_data(:,j-1));
    %     if (dist > dist_thres)
    %         ind = [ind j];
    %         dist = 0;
    %     end
    % end
    % ind = [1 ind size(Pg_data,2)];
    ind = [1 size(Pg_data,2)];
    Pg_data = Pg_data(:, ind);
    Qg_data = Qg_data(:, ind);

    target_vp_data = cell(size(Pg_data,2), 1);
    for j=1:length(target_vp_data)
        Vp_data = zeros(3, length(vp_offsets));
        p = Pg_data(:,j);
        R = quat2rotm(Qg_data(:,j)');
        for k=1:length(vp_offsets)
            z_offset = [0; 0; vp_offsets(k)];
            Vp_data(:, k) = p + R*z_offset;
        end
        target_vp_data{j} = struct('pos',Vp_data, 'quat',repmat(Qg_data(:,j), 1, length(vp_offsets)));
    end

end