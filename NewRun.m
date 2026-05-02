%% MAE 547 Simulate Robot Project
clc; clear; close all;


% mode 0: dynamics simulation
% mode 1: compliance control
% mode 2: impedance control

mode = 1;
testing = false;

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

    x_d  = @(t) [0.40; 0.20; 0.30; 0; 0; 0];
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

elseif mode == 1

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

















elseif mode == 2
    disp("Solving with impedance control")

end


%% Stage 4: Plot results





