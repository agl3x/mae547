function [t_out, q_out, qd_out, qdd_out] = DynSim_EOM(EOM, params, u, q0, qd0, tspan)

% u - input function, Example:  u = @(t,q,qd) [cos(t); sin(t)];
% q0 - [1xn] or [n x 1]
% qd0 - [1xn] or [n x 1]  
% tspan - [t_start, t_end]   
%
% OUTPUTS
%   t_out   - [N x 1]  time vector from ode45
%   q_out   - [N x n]  joint positions
%   qd_out  - [N x n]  joint velocities
%   qdd_out - [N x n]  joint accelerations (reconstructed from EOM)
%
%


    %% Identify main varaible q

    q_sym  = EOM.Q.q;
    qd_sym = EOM.Q.qd;
    n      = numel(q_sym);

    %%  Substitute all parameters into B, C, G
    %  (leaves only joint variables q and qd as remaining symbols)
    B_sym = EOM.B;
    C_sym = EOM.c;
    G_sym = EOM.G;

    param_fields = fieldnames(params);
    for i = 1:numel(param_fields)
        pname = param_fields{i};
        pval  = params.(pname);
        psym  = sym(pname);
        B_sym = subs(B_sym, psym, pval);
        C_sym = subs(C_sym, psym, pval);
        G_sym = subs(G_sym, psym, pval);
    end


    %%  Make Functions from symbolic matricies
    %  State vector:  sv = [q1; q2; ...; qn; qd1; qd2; ...; qdn]  (2n x 1)
    sv = [q_sym(:); qd_sym(:)];   % 2n x 1 symbolic state vector

    B_func = matlabFunction(B_sym, 'Vars', {sv});
    C_func = matlabFunction(C_sym, 'Vars', {sv});
    G_func = matlabFunction(G_sym, 'Vars', {sv});

    %%  ODE right-hand side  (state-space form)
    function dxdt = odefun(t, x)
        q_n  = x(1:n);
        qd_n = x(n+1:end);

        tau_n = u(t, q_n, qd_n);
        tau_n = tau_n(:);                   % ensure column vector

        B_n = double(B_func(x));            % [n x n]
        C_n = double(C_func(x));            % [n x n]
        G_n = double(G_func(x));            % [n x 1]

        % Forward dynamics: qdd = B \ (tau - C*qd - G)
        qdd_n = B_n \ (tau_n - C_n * qd_n - G_n);

        dxdt = [qd_n; qdd_n];
    end

    %%  Integrate with ode45
    x0   = [q0(:); qd0(:)];
    opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);

    [t_out, x_out] = ode45(@odefun, tspan, x0, opts);

    q_out  = x_out(:, 1:n);        % [N x n]
    qd_out = x_out(:, n+1:end);    % [N x n]

    %%  Solve for qdd from qd,q for each
    N       = numel(t_out);
    qdd_out = zeros(N, n);

    for k = 1:N
        xk    = x_out(k, :).'; 
        tau_k = u(t_out(k), q_out(k,:).', qd_out(k,:).'); 
        tau_k = tau_k(:);

        B_k   = double(B_func(xk));
        C_k   = double(C_func(xk));
        G_k   = double(G_func(xk));

        qdd_out(k, :) = (B_k \ (tau_k - C_k * qd_out(k,:).' - G_k)).';
    end

    %% ------------------------------------------------------------------ %%
    %  Plot results
    %% ------------------------------------------------------------------ %%
    joint_labels = arrayfun(@(i) sprintf('Joint %d', i), 1:n, 'UniformOutput', false);

    figure('Name', 'Robotic Arm Dynamics', 'NumberTitle', 'off', ...
           'Position', [100 100 900 700]);

    % ---- q ----
    subplot(3, 1, 1);
    plot(t_out, q_out, 'LineWidth', 1.8);
    xlabel('Time (s)');
    ylabel('$q$', 'Interpreter', 'latex', 'FontSize', 12);
    title('Joint Position', 'Interpreter', 'latex', 'FontSize', 13);
    legend(joint_labels, 'Location', 'best');
    grid on;  box on;

    % ---- qd ----
    subplot(3, 1, 2);
    plot(t_out, qd_out, 'LineWidth', 1.8);
    xlabel('Time (s)');
    ylabel('$\dot{q}$', 'Interpreter', 'latex', 'FontSize', 12);
    title('Joint Velocity', 'Interpreter', 'latex', 'FontSize', 13);
    legend(joint_labels, 'Location', 'best');
    grid on;  box on;

    % ---- qdd ----
    subplot(3, 1, 3);
    plot(t_out, qdd_out, 'LineWidth', 1.8);
    xlabel('Time (s)');
    ylabel('$\ddot{q}$', 'Interpreter', 'latex', 'FontSize', 12);
    title('Joint Acceleration', 'Interpreter', 'latex', 'FontSize', 13);
    legend(joint_labels, 'Location', 'best');
    grid on;  box on;

    sgtitle('Robotic Arm Dynamics Simulation', 'FontSize', 14, 'FontWeight', 'bold');

end