%% MAE 547 Simulate Robot Project
clc; clear; close all;


%% Load UI
input_ui = MAE547_Final_Project_App();
disp(input_ui)

waitfor(input_ui, 'solving', 1)
x = input_ui.jointCount;

% Get DH table
a     = input_ui.aValues;
alpha = input_ui.alphaValues;
d     = input_ui.dValues;
theta = input_ui.thetaValues;
rp    = input_ui.rpValues;

dh_raw = [a',alpha',d',theta'];
joint_types = extractBefore(string(rp), 2);

%
params.m_l = input_ui.mLinkValues;
params.I_l = input_ui.ILinkValues;
params.m_m = input_ui.mJointValues;
params.I_m = input_ui.IJointValues;
params.k_r = input_ui.krValues;

delete(input_ui)

%% ── Numeric workspace variables for Simulink Constant blocks ─────────────

dh_base = dh_raw;       % n x 4  [a, alpha, d, theta]  (zeros where q goes)

jt = zeros(N, 1);
for i = 1:N
    jt(i) = double(joint_types(i) == "R");   % 1=revolute, 0=prismatic
end

m_l = params.m_l(:);
m_m = params.m_m(:);
I_l = params.I_l(:);
I_m = params.I_m(:);
k_r = params.k_r(:);

%% ── Symbolic EOM (for display and report) ────────────────────────────────

q_sym  = sym('q',  [1 N]);
dq_sym = sym('dq', [1 N]);
assume(q_sym,  'real')
assume(dq_sym, 'real')

dh_sym = sym(dh_raw);
for i = 1:N
    if joint_types(i) == "R"
        dh_sym(i,4) = dh_sym(i,4) + q_sym(i);
    else
        dh_sym(i,3) = dh_sym(i,3) + q_sym(i);
    end
end

disp("Calculating symbolic equations of motion...")
[EOM1] = EOM(dh_sym, sym(params.m_l), sym(params.m_m), ...
              sym(params.I_l), sym(params.I_m), sym(params.k_r), ...
              g0, joint_types);

disp("B(q):"),    disp(EOM1.B)
disp("c(q,dq):"), disp(EOM1.c)
disp("G(q):"),    disp(EOM1.G)

