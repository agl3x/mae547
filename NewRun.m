%% MAE 547 Simulate Robot Project
clc; clear; close all;


% mode 0: dynamics simulation
% mode 1: compliance control
% mode 2: impedance control

<<<<<<< HEAD
mode = 2;
testing = true;
=======
mode = 1;
testing = false;
>>>>>>> 343b48b864de5f1e0a1e66f7172527c60a48446c

g0 = [0, 0, -9.81];    % gravity in -Z direction

%% Stage 1: Input

if testing

    % robot properties
    L1 = 0.40;   % upper arm length  [m]
    L2 = 0.35;   % forearm length    [m]
    L3 = 0.25;   % wrist offset      [m]

    %        a         alpha    d   theta
    dh_raw = [0,       pi/2,   0,   0;    % joint 1: shoulder pan
              L1,      0,      0,   0;    % joint 2: shoulder lift
              L2+L3,   0,      0,   0];   % joint 3: elbow flex

    joint_types = ["R"; "R"; "R"];
    DOF = length(joint_types);

    m_l = [2.5,  1.8,  0.8];       % link masses        [kg]
    m_m = [0.5,  0.4,  0.3];       % motor masses       [kg]
    I_l = [0.08, 0.04, 0.01];      % link inertias      [kg·m²]
    I_m = [0.005,0.003,0.001];     % motor inertias     [kg·m²]
    k_r = [50,   50,   30  ];      % gear ratios (high torque)
    F_s = [0,    0,    0   ];      % static friction
    F_v = [0,    0,    0   ];      % viscous friction
    
    % scenario properties

    q0  = [pi/4; pi/4; -pi/4];
    dq0 = [0; 0; 0];

    x_d  = @(t) [0.10*t; 0.20; 0.30; 0; 0; 0];
    xd_d  = @(t) [.1; 0; 0; 0; 0; 0];
    xdd_d = @(t) [0; 0; 0; 0; 0; 0];
    t_stop = 10;
    % xd_d = zeros(6, 1);
    
    h_e = [-8; 0; 0; 0; 0; 0];

    % controller properties

    K_p = 400;
    K_d = 40;
    M_d = 10;
   

else

    % launch gui window
    input_ui = RoboticSim();
    disp(input_ui)
    
    waitfor(input_ui, 'solving', 1)
    x = input_ui.jointCount;

    % robot properties
    a     = input_ui.aValues;
    alpha = input_ui.alphaValues;
    d     = input_ui.dValues;
    theta = input_ui.thetaValues;
    rp    = input_ui.rpValues;
    
    dh_raw = [a',alpha',d',theta'];
    joint_types = extractBefore(string(rp), 2);
    DOF = length(joint_types);

    m_l = input_ui.mLinkValues;
    m_m = input_ui.mJointValues;
    I_m = input_ui.IJointValues;
    I_l = input_ui.ILinkValues; 
    k_r = input_ui.krValues;
    F_s = input_ui.sfValues;
    F_v = input_ui.vfValues;
    
    % scenario properties

    t_values = input_ui.tValues;


    % controller properties

    mode = input_ui.menu - 1;

    kpkmd = input_ui.kpkmdValues;
    
    K_p = kpkmd(1);
    K_d = kpkmd(2);
    M_d = kpkmd(3);
    
    delete(input_ui)

end

%% Stage 1.5: Build Robot Toolbox Model

Links = [];
for i=1:N
    if joint_types(i) == "R"
        Links(i) = Revolute('d', 0, ...   % link length (Dennavit-Hartenberg notation)
            'a', A(i), ...               % link offset (Dennavit-Hartenberg notation)
            'alpha', alpha(i), ...        % link twist (Dennavit-Hartenberg notation)
            'I', [0, 0, I_l(i), 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
            'r', [l(i), 0, 0], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
            'm', m(i), ...               % mass of link
            'Jm', I_m(i), ...         % actuator inertia 
            'G', k_r(i)); % gear ratio
    elseif joint_types(i) == "P"
        Links(i) = Revolute('d', 0, ...   % link length (Dennavit-Hartenberg notation)
            'a', A(i), ...               % link offset (Dennavit-Hartenberg notation)
            'alpha', alpha(i), ...        % link twist (Dennavit-Hartenberg notation)
            'I', [0, 0, I_l(i), 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
            'r', [l(i), 0, 0], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
            'm', m(i), ...               % mass of link
            'Jm', I_m(i), ...         % actuator inertia 
            'G', k_r(i)); % gear ratio
    end
end


%% Stage 2: Determine Symbolic EOM

q_sym  = sym('q',  [1 DOF]);
dq_sym = sym('dq', [1 DOF]);
assume(q_sym,  'real')
assume(dq_sym, 'real')

dh_sym = sym(dh_raw);
for i = 1:DOF
    if joint_types(i) == "R"
        dh_sym(i,4) = dh_sym(i,4) + q_sym(i);
    else
        dh_sym(i,3) = dh_sym(i,3) + q_sym(i);
    end
end


disp("Calculating symbolic equations of motion...")
[EOM1] = EOM(dh_sym, m_l, m_m, ...
              I_l, I_m, k_r, ...
              g0, joint_types, F_v, F_s);

disp("B(q):")
disp(EOM1.B)

disp("c(q,dq):")
disp(EOM1.c)

disp("G(q):")
disp(EOM1.G)

% t_i = 0;
% t_f = 10;
% 
% dimensions = 6;
% X_d = SimpleTrajectory(t_i, t_f, 10*rand(dimensions, 1)-5, 10*rand(dimensions, 1)-5, 5);
% dX_d = diff(X_d);
% d2X_d = diff(X_d, 2);
% 
% 
% X_d = matlabFunction(X_d);
% dX_d = matlabFunction(dX_d);
% d2X_d = matlabFunction(d2X_d);
% 
% disp(X_d)
% disp(dX_d)
% disp(d2X_d)


%% Stage 3: Run simulation with chosen dyanamic model

if mode == 0
    disp("Dyn sim not implemented yet")

% Compliance ----------------------------------
elseif mode == 1 % 

    disp("Solving with compliance control")

    dh_base = dh_raw;       % n x 4  [a, alpha, d, theta]  (zeros where q goes)

    jt = zeros(DOF, 1);
    for i = 1:DOF
        jt(i) = double(joint_types(i) == "R");   % 1=revolute, 0=prismatic
    end
    
    m_l = m_l(:);
    m_m = m_m(:);
    I_l = I_l(:);
    I_m = I_m(:);
    k_r = k_r(:);

    KP = eye(6)*K_p;
    KD = eye(6)*K_d;

    t_f_cc = t_stop;

    q0_cc  = q0;
    dq0_cc = dq0;

    disp("Starting CompControl simulation...")
    out = sim("CompControl_V2.slx");
    disp("Done. Run plot_results.m for output plots.")










% IMPEDENCE ------------------------------------
elseif mode == 2
    disp("Solving with impedance control")

    Md = eye(6) * M_d;
    Cd = eye(6) * K_p;
    Kd = eye(6) * K_d;

    q0     = q0;
    qdot_0 = dq0;
    dt    = 0.001;
    T = t_stop;


    DH = double(dh_raw);
    a = dh_raw(:,1);
    alpha = dh_raw(:,2);
    d = dh_raw(:,3);
    theta = dh_raw(:,4);
    
    mass = m_l(:); 

    N = length(joint_types);
    inertia = zeros(N, 3, 3);
    for i = 1:N
        inertia(i,:,:) = diag([I_l(i), I_l(i), I_l(i)]);
    end
    
    % Center of mass
    com = zeros(N, 3);
    for i = 1:N
        com(i,1) = dh_raw(i,1) / 2;
    end
    
    % Assignment from workspace to simulink constants
    assignin('base', 'a',       a);
    assignin('base', 'd',       d);
    assignin('base', 'alpha',   alpha);
    assignin('base', 'theta',   theta);
    assignin('base', 'DH',      DH);
    assignin('base', 'mass',    mass);
    assignin('base', 'com',     com);
    assignin('base', 'inertia', inertia);
    assignin('base', 'Md',      Md);
    assignin('base', 'Cd',      Cd);
    assignin('base', 'Kd',      Kd);
    assignin('base', 'q0',      q0);
    assignin('base', 'q_dot0',  qdot_0);
    assignin('base', 'dt',      dt);
    
    open_system("impedanceSimulink.slx")
    set_param("impedanceSimulink", "SimulationMode", "normal")
    disp("Starting impedance control simulation...")
    out = sim("impedanceSimulink.slx");

end


%% Stage 4: Plot results



%% Stage 4 Plotting 

if isa(out.x_e_out, 'timeseries')
    t    = out.x_e_out.Time;
    x_e  = squeeze(out.x_e_out.Data)';       % T × 6
    h_e  = squeeze(out.h_e_out.Data)';       % T × 6
    x_d  = squeeze(out.x_d_out.Data)';       % T × 6
else
    % Logged as 'Array' format: rows = time, columns = signals
    t    = out.tout;
    x_e  = squeeze(out.x_e_out);
    h_e  = squeeze(out.h_e_out);
end
 
if size(x_d) ~= size(x_e)
    x_d = repmat(x_d', length(t), 1);
end

if size(h_e) ~= size(x_e)
    h_e = repmat(h_e', length(t), 1);
end

% ── Figure 1: Desired vs Actual End-Effector POSITION ───────────────────
 
labels_pos = {'p_x (m)', 'p_y (m)', 'p_z (m)'};
labels_ori = {'roll (rad)', 'pitch (rad)', 'yaw (rad)'};
 
figure('Name', 'End-Effector Position: Desired vs Actual', ...
       'NumberTitle', 'off', 'Color', 'w');
 
for i = 1:3
    subplot(3, 1, i);
    hold on; grid on;
    plot(t, x_d(:, i), 'b--', 'LineWidth', 1.8, 'DisplayName', 'Desired x_d');
    plot(t, x_e(:, i), 'r-',  'LineWidth', 1.5, 'DisplayName', 'Actual  x_e');
    xlabel('Time (s)');
    ylabel(labels_pos{i});
    legend('Location', 'best');
    title(labels_pos{i});
 
    % % Annotate steady-state error (last 10% of simulation)
    % i_ss = round(0.9 * length(t));
    % err_ss = mean(x_d(i_ss:end, i) - x_e(i_ss:end, i));
    % text(t(i_ss), x_e(i_ss, i), ...
    %      sprintf('  SS error = %.4f', err_ss), ...
    %      'FontSize', 8, 'Color', [0.5 0 0]);
end
sgtitle('Compliance Control — Desired vs Actual End-Effector Position', ...
        'FontWeight', 'bold');
 
% ── Figure 2: End-Effector Contact Forces ────────────────────────────────
 
force_labels  = {'f_x (N)', 'f_y (N)', 'f_z (N)'};
moment_labels = {'m_x (N·m)', 'm_y (N·m)', 'm_z (N·m)'};
 
figure('Name', 'End-Effector Contact Forces', ...
       'NumberTitle', 'off', 'Color', 'w');
 
for i = 1:3
    subplot(3, 1, i);
    hold on; grid on;
    plot(t, h_e(:, i),   'r-',  'LineWidth', 1.5, 'DisplayName', force_labels{i});
    % yline(h_e(end, i), 'k--', 'LineWidth', 1.0, 'DisplayName', 'Steady state');
    xlabel('Time (s)');
    ylabel(force_labels{i});
    legend('Location', 'best');
    title(force_labels{i});
end
sgtitle('Compliance Control — End-Effector Contact Forces h_e', ...
        'FontWeight', 'bold');

