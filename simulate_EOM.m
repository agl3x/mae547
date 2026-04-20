function simulate_EOM(EOM, params, tau_func, q0, qd0, tspan)
    % params - struct with numeric values for all physical parameters
    %          e.g. params.L1 = 1; params.m1 = 2; params.IL1 = ...
    %
    % All other args same as before

    n = length(EOM.Q.q);
    q   = EOM.Q.q;
    qd  = EOM.Q.qd;

    %% Substitute numeric parameters into symbolic matrices
    % Collect all symbolic variables in B that aren't q or qd
    all_vars  = [q; qd];
    param_vars = setdiff(symvar(EOM.B), all_vars); % find leftover syms

    % Build substitution vectors from params struct
    param_vals = zeros(size(param_vars));
    for i = 1:length(param_vars)
        vname = char(param_vars(i));
        if isfield(params, vname)
            param_vals(i) = params.(vname);
        else
            error('Missing parameter: %s not found in params struct', vname);
        end
    end

    % Substitute into each matrix
    B_num = subs(EOM.B, param_vars, param_vals);
    C_num = subs(EOM.c, param_vars, param_vals);
    G_num = subs(EOM.G, param_vars, param_vals);

    %% Convert to numeric function handles
    B_func = matlabFunction(B_num, 'Vars', {q});
    C_func = matlabFunction(C_num, 'Vars', {q, qd});
    G_func = matlabFunction(G_num, 'Vars', {q});

    %% Define state derivative
    function dxdt = dynamics(t, x)
        q_num  = x(1:n);
        qd_num = x(n+1:end);

        B_k   = B_func(q_num);
        C_k   = C_func(q_num, qd_num);
        G_k   = G_func(q_num);
        tau_k = tau_func(t, q_num, qd_num);

        qdd_num = B_k \ (tau_k - C_k*qd_num - G_k);
        dxdt = [qd_num; qdd_num];
    end

    %% Integrate
    x0   = [q0(:); qd0(:)];
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t_out, x_out] = ode45(@dynamics, tspan, x0, opts);

    q_out  = x_out(:, 1:n);
    qd_out = x_out(:, n+1:end);

    %% Back-calculate qdd
    qdd_out = zeros(size(q_out));
    for k = 1:length(t_out)
        q_k   = q_out(k,:).';
        qd_k  = qd_out(k,:).';
        tau_k = tau_func(t_out(k), q_k, qd_k);
        qdd_out(k,:) = (B_func(q_k) \ (tau_k - C_func(q_k,qd_k)*qd_k - G_func(q_k))).';
    end

    %% Plot
    joint_names = cell(1,n);
    for i = 1:n
        joint_names{i} = char(q(i));
    end

    figure('Name','Robot Arm Dynamics','NumberTitle','off');
    titles  = {'Joint Position (q)','Joint Velocity (qd)','Joint Acceleration (qdd)'};
    data    = {q_out, qd_out, qdd_out};
    ylabels = {'q','qd','qdd'};

    for p = 1:3
        subplot(3,1,p);
        hold on;
        for i = 1:n
            plot(t_out, data{p}(:,i), 'LineWidth', 1.5, 'DisplayName', joint_names{i});
        end
        xlabel('Time (s)'); ylabel(ylabels{p});
        title(titles{p}); legend('show'); grid on;
    end
end