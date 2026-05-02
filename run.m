clear all; clc; close all;

%% initial inputs (DH table and joint types, parameters optional)

testmode = true; % example: cylindrical robot

g0 = [0 0 -9.81];

if testmode
    dh_raw = [0,     0,  0,     0;
              0, -pi/2,  0,     0;
              0,     0,  0,     0];
    joint_types = ["R"; "P"; "P"];
      
    params.m_l = [3, 3, 3];
    params.m_m = [0.1 0.1 0.1];
    params.I_l = [0.01 0.01 0.01];
    params.I_m = [0.001 0.001 0.001];
    params.k_r = [10 10 10];
else
    input_ui = MAE547_Final_Project_App();
    disp(input_ui)
    
    waitfor(input_ui, 'solving', 1)
    x = input_ui.jointCount;

    a     = input_ui.aValues;
    alpha = input_ui.alphaValues;
    d     = input_ui.dValues;
    theta = input_ui.thetaValues;
    rp    = input_ui.rpValues;

    dh_raw = [a',alpha',d',theta'];
    joint_types = extractBefore(string(rp), 2);
    
    params.m_l = input_ui.mLinkValues;
    params.I_l = input_ui.ILinkValues;
    params.m_m = input_ui.mJointValues;
    params.I_m = input_ui.IJointValues;
    params.k_r = input_ui.krValues;
    
    delete(input_ui)
end
% 
% disp(dh_raw)
% disp(joint_types)

%% generate symbolic variables

N = length(joint_types);
if size(dh_raw, 1) ~= N
    disp("DH table and Joint Types don't have the same length.")
    return
else
    fprintf("Robot specified with %d joints, with types %s\n", N, join(joint_types));
    
end

q = sym('q',[1 N]);
dq = sym('dq',[1 N]);
assume(q, 'real')
assume(dq, 'real')

dh = sym(dh_raw);
for i=1:N
    if joint_types(i) == "R"
        dh(i, 4) = dh(i, 4) + q(i);
    elseif joint_types(i) == "P"
        dh(i, 3) = dh(i, 3) + q(i);
    else
        disp("Specify joint types as 'R' or 'P'")
    end
end

disp("DH Table: ")
disp(dh)

m_l = sym('m_l', [N 1]); % link masses
m_m = sym('m_m', [N 1]); % motor masses
I_l = sym('I_l', [N 1]); % link moments of inertia
I_m = sym('I_m', [N 1]); % motor moments of inertia
k_r = sym('k_r', [N 1]); % gear ratios

mode = 'symbolic';
if exist('params', 'var')
    try
        m_l = sym(params.m_l);
        m_m = sym(params.m_m);
        I_l = sym(params.I_l);
        I_m = sym(params.I_m);
        k_r = sym(params.k_r);
        mode = 'numeric';
        disp("Successfully input parameters, solving numerically")

    catch exception
        disp("Incomplete parameters, solving symbolically")
        m_l = sym('m_l', [N 1]); 
        m_m = sym('m_m', [N 1]);
        I_l = sym('I_l', [N 1]);
        I_m = sym('I_m', [N 1]);
        k_r = sym('k_r', [N 1]);
    end  
else
    disp("Nonexistent parameters, solving symbolically")
end


%% get equations of motion

disp("Calculating equations of motion")
[EOM2] = EOM(dh,m_l,m_m,I_l,I_m,k_r,g0, joint_types);


q = EOM2.Q.q;
dq = EOM2.Q.qd;
qdq = [q;dq];

B(q) = EOM2.B;
C(qdq) = EOM2.c;
G(q) = EOM2.G;

disp("B(q):")
disp(B)

disp("C(q, dq):")
disp(C)

disp("G(q):")
disp(G)

B = matlabFunction(B);
C = matlabFunction(C);
G = matlabFunction(G);

%% simulink test

t_i = 0;
t_f = 10;



dimensions = 4;
X_d = SimpleTrajectory(t_i, t_f, 10*rand(dimensions, 1)-5, 10*rand(dimensions, 1)-5, 5);
dX_d = diff(X_d);
d2X_d = diff(X_d, 2);


X_d = matlabFunction(X_d);
dX_d = matlabFunction(dX_d);
d2X_d = matlabFunction(d2X_d);

disp("Starting simulink...")
sim("inversedynamicstest.slx");

%% Bridge to impedance model
DH = double(dh_raw);
a = dh_raw(:,1);
alpha = dh_raw(:,2);
d = dh_raw(:,3);
theta = dh_raw(:,4);

mass = params.m_l(:); 

inertia = zeros(N, 3, 3);
for i = 1:N
    inertia(i,:,:) = diag([params.I_l(i), params.I_l(i), params.I_l(i)]);
end

% Center of mass
com = zeros(N, 3);
for i = 1:N
    com(i,1) = dh_raw(i,1) / 2;
end

% Impedance gains
Md = eye(6) * 10;
Cd = eye(6) * 60;
Kd = eye(6) * 400;

% Desired pose — use end of your trajectory
o_d = [1; 0; 0];   % replace with actual target, currently hardcoded 
R_d = eye(3);

% Initial conditions
xi = zeros(N, 1);
xf = zeros(N, 1);
q0     = zeros(N, 1);
qdot_0 = zeros(N, 1);
dt    = 0.001;
T = t_f;

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
assignin('base', 'o_d',     o_d);
assignin('base', 'R_d',     R_d);
assignin('base', 'q0',      q0);
assignin('base', 'q_dot0',  qdot_0);
assignin('base', 'dt',      dt);
assignin('base', 'T_sim',   T);
assignin('base', 'n_dof',   N);
assignin('base', 'xi',      xi);
assignin('base', 'xf',      xf);

open_system("impedanceSimulink.slx")
set_param("impedanceSimulink", "SimulationMode", "normal")
disp("Starting impedance control simulation...")
Impedance_out = sim("impedanceSimulink.slx");


%% Exporting xe and xd

t_log = Impedance_out.tout;

try
    x_d = Impedance_out.x_d_log;
catch
    try
        x_d = logsout.getElement('x_d_log').Values.Data;
    catch
        error("x_d_log not found. Check your To Workspace block name.")
    end
end

try
    x_e = Impedance_out.x_e_log;
catch
    try
        x_e = logsout.getElement('x_e_log').Values.Data;
    catch
        error("x_e_log not found. Check your To Workspace block name.")
    end
end

% % ── Handle array dimensions ───────────────────────────────────────────
% % Simulink To Workspace outputs [timesteps x signal_dim]
% % If it comes out transposed, fix it:
% if size(x_d, 1) ~= length(t_log)
%     x_d = x_d';
% end
% if size(x_e, 1) ~= length(t_log)
%     x_e = x_e';
% end

n_dims = size(x_d, 2);   % number of DOF / task-space dims





