%% CompControlTest_V2
clear; clc; close all;

%% ── Robot definition ────────────────────────────────────────────────────
%
%  TEST ROBOT: 3R Spatial Elbow Manipulator
%  Three revolute joints, full 3D workspace.
%
%  Kinematic layout:
%    Joint 1: base rotation about world Z   (shoulder pan)
%    Joint 2: shoulder rotation about X     (shoulder lift)
%    Joint 3: elbow rotation about X        (elbow flex)
%
%  Link lengths:  L1 = 0.4 m  (upper arm)
%                 L2 = 0.35 m (forearm)
%                 L3 = 0.25 m (wrist / end-effector offset)
%
%  DH convention: [a, alpha, d, theta]
%   Row 1: a=0,   alpha=pi/2,  d=0,  theta=q1  (pan, twist into shoulder)
%   Row 2: a=L1,  alpha=0,     d=0,  theta=q2  (upper arm)
%   Row 3: a=L2,  alpha=0,     d=0,  theta=q3  (forearm)
%   EE offset handled via L3 in a=L2, or add a fixed frame — here we
%   fold L3 into a=L2+L3 on the last link for simplicity.
%
%  At q = [0;0;0] the arm is fully extended along X, EE at [L1+L2, 0, 0].

testmode = true;

g0 = [0, 0, -9.81];    % gravity in -Z direction

if testmode

    L1 = 0.40;   % upper arm length  [m]
    L2 = 0.35;   % forearm length    [m]
    L3 = 0.25;   % wrist offset      [m]

    %        a         alpha    d   theta
    dh_raw = [0,       pi/2,   0,   0;    % joint 1: shoulder pan
              L1,      0,      0,   0;    % joint 2: shoulder lift
              L2+L3,   0,      0,   0];   % joint 3: elbow flex

    joint_types = ["R"; "R"; "R"];

    % Physical parameters — realistic 3kg-class robot arm
    %   Link masses reduce toward the tip (typical for serial arms)
    params.m_l = [2.5,  1.8,  0.8];       % link masses        [kg]
    params.m_m = [0.5,  0.4,  0.3];       % motor masses       [kg]
    params.I_l = [0.08, 0.04, 0.01];      % link inertias      [kg·m²]
    params.I_m = [0.005,0.003,0.001];     % motor inertias     [kg·m²]
    params.k_r = [50,   50,   30  ];      % gear ratios (high torque)

    % Initial configuration: arm at 45° shoulder, 45° elbow
    % This puts the EE somewhere reachable in 3D space (not a singularity)
    q0_cc  = [pi/4; pi/4; -pi/4];         % [rad]
    dq0_cc = [0; 0; 0];

    % Desired pose: hold current end-effector position
    % (compute analytically from q0 for a clean regulation task)
    % EE position at q0 via forward kinematics:
    %   p_x = (L1+L2+L3)*cos(q1)*cos(q2+q3) ... (approx, exact from FK)
    % Set desired to a nearby reachable point:
    %x_d  = [0.40; 0.20; 0.30; 0; 0; 0];  % desired pose [pos(m); rpy(rad)]
    %xd_d = zeros(6, 1);                   % regulation task (stationary)

    % Step contact force: 8 N push in -X direction, turns on at t = 2 s
    % Implemented in Simulink as: h_e_mag * (t >= t_step)
    % Use a Step block in Simulink for h_e, OR set constant h_e here
    % for a simpler test (constant force from t=0):
    h_e       = [-8; 0; 0; 0; 0; 0];     % 8 N in -X direction [N; N·m]
    t_step_he = 2.0;                      % step onset time [s]
    % NOTE: wire h_e in Simulink as a Step block:
    %   Initial value = [0;0;0;0;0;0], Final value = h_e, Step time = t_step_he
    % OR use the constant h_e above for a constant-force test.

    x_d = @(t) [cos(t); sin(t); .1*t; 0; 0; 0];
    xd_d = @(t) [-sin(t); cos(t); .1; 0;0;0];

else
    input_ui = MAE547_Final_Project_App();
    disp(input_ui)
    waitfor(input_ui, 'solving', 1)

    a     = input_ui.aValues;
    alpha = input_ui.alphaValues;
    d     = input_ui.dValues;
    theta = input_ui.thetaValues;
    rp    = input_ui.rpValues;

    dh_raw      = [a', alpha', d', theta'];
    joint_types = extractBefore(string(rp), 2);

    params.m_l = input_ui.mLinkValues;
    params.I_l = input_ui.ILinkValues;
    params.m_m = input_ui.mJointValues;
    params.I_m = input_ui.IJointValues;
    params.k_r = input_ui.krValues;
    delete(input_ui)

    q0_cc  = zeros(length(joint_types), 1);
    dq0_cc = zeros(length(joint_types), 1);
    x_d    = zeros(6, 1);
    xd_d   = zeros(6, 1);
    h_e    = zeros(6, 1);
    t_step_he = 2.0;
end

N = length(joint_types);
fprintf("Robot: %d-DOF, joints: %s\n", N, join(joint_types, "-"));

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

% %% ── Symbolic EOM (for display and report) ────────────────────────────────
% 
% q_sym  = sym('q',  [1 N]);
% dq_sym = sym('dq', [1 N]);
% assume(q_sym,  'real')
% assume(dq_sym, 'real')
% 
% dh_sym = sym(dh_raw);
% for i = 1:N
%     if joint_types(i) == "R"
%         dh_sym(i,4) = dh_sym(i,4) + q_sym(i);
%     else
%         dh_sym(i,3) = dh_sym(i,3) + q_sym(i);
%     end
% end
% 
% disp("Calculating symbolic equations of motion...")
% [EOM2] = EOM(dh_sym, sym(params.m_l), sym(params.m_m), ...
%               sym(params.I_l), sym(params.I_m), sym(params.k_r), ...
%               g0, joint_types);
% 
% disp("B(q):"),    disp(EOM2.B)
% disp("C(q,dq):"), disp(EOM2.c)
% disp("G(q):"),    disp(EOM2.G)

%% ── Compliance control gains ─────────────────────────────────────────────
%
%  KP = task-space stiffness (6x6).  KP^-1 = compliance matrix (eq. 9.3).
%  KD tuned for ~critical damping:  KD ≈ 2*sqrt(KP) on each diagonal.
%
%  Position stiffness 500 N/m  → compliance 2 mm/N
%  Expected steady-state error under 8N:  8 * (1/500) = 16 mm in X

KP = diag([500, 500, 500,  80,  80,  80]);
KD = diag([ 45,  45,  45,  18,  18,  18]);

fprintf("\nPosition compliance (KP^-1): %.4f m/N  (%.1f mm/N)\n", ...
        1/KP(1,1), 1000/KP(1,1));
fprintf("Expected SS position error under %.0f N: %.1f mm\n", ...
        abs(h_e(1)), abs(h_e(1))*1000/KP(1,1));

%% ── Simulation time ──────────────────────────────────────────────────────

t_f_cc = 10;    % [s]  — long enough to see settling before and after step

%% ── Verify initial pose is reachable ────────────────────────────────────

x_e0 = fkine_pose_num(q0_cc, dh_base, jt);
fprintf("\nInitial EE pose (q0):\n");
fprintf("  position:    [%.3f, %.3f, %.3f] m\n",   x_e0(1), x_e0(2), x_e0(3));
fprintf("  orientation: [%.3f, %.3f, %.3f] rad\n", x_e0(4), x_e0(5), x_e0(6));
fprintf("Desired pose x_d:\n");
fprintf("  position:    [%.3f, %.3f, %.3f] m\n",   x_d(1),  x_d(2),  x_d(3));
fprintf("Initial position error: [%.3f, %.3f, %.3f] m\n", ...
        x_d(1)-x_e0(1), x_d(2)-x_e0(2), x_d(3)-x_e0(3));

%% ── Launch Simulink ──────────────────────────────────────────────────────

disp("Starting CompControl simulation...")
out = sim("CompControl_V2.slx");
disp("Done. Run plot_results.m for output plots.")

%
%  Run this script AFTER CompControl.slx finishes simulating.
%  Produces the two plots required by the rubric:
%    1. Desired vs actual end-effector position (x, y, z) over time
%    2. End-effector contact forces (fx, fy, fz) over time
%
%  Requires in workspace (logged by Simulink To Workspace blocks):
%    x_e_out   — timeseries or (T × 6) array: actual pose
%    x_d_out   — timeseries or (T × 6) array: desired pose
%    h_e_out   — timeseries or (T × 6) array: contact wrench
%    tout      — (T × 1) time vector (logged automatically by Simulink)
% =========================================================================
 
%% ── Unpack data from Simulink To Workspace ───────────────────────────────
%
%  Simulink logs timeseries objects by default. Convert to plain arrays.
 
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

%% ── Figure 1: Desired vs Actual End-Effector POSITION ───────────────────
 
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
 
%% ── Figure 2: End-Effector Contact Forces ────────────────────────────────
 
force_labels  = {'f_x (N)', 'f_y (N)', 'f_z (N)'};
moment_labels = {'m_x (N·m)', 'm_y (N·m)', 'm_z (N·m)'};
 
figure('Name', 'End-Effector Contact Forces', ...
       'NumberTitle', 'off', 'Color', 'w');
 
for i = 1:3
    subplot(3, 1, i);
    hold on; grid on;
    plot(t, h_e(:, i),   'r-',  'LineWidth', 1.5, 'DisplayName', force_labels{i});
    yline(h_e(end, i), 'k--', 'LineWidth', 1.0, 'DisplayName', 'Steady state');
    xlabel('Time (s)');
    ylabel(force_labels{i});
    legend('Location', 'best');
    title(force_labels{i});
end
sgtitle('Compliance Control — End-Effector Contact Forces h_e', ...
        'FontWeight', 'bold');
 
% %% ── Figure 3: Orientation error (bonus) ─────────────────────────────────
% 
% figure('Name', 'End-Effector Orientation: Desired vs Actual', ...
%        'NumberTitle', 'off', 'Color', 'w');
% 
% for i = 1:3
%     subplot(3, 1, i);
%     hold on; grid on;
%     plot(t, x_d(:, i+3), 'b--', 'LineWidth', 1.8, 'DisplayName', 'Desired');
%     plot(t, x_e(:, i+3), 'r-',  'LineWidth', 1.5, 'DisplayName', 'Actual');
%     xlabel('Time (s)');
%     ylabel(labels_ori{i});
%     legend('Location', 'best');
%     title(labels_ori{i});
% end
% sgtitle('Compliance Control — Desired vs Actual Orientation', ...
%         'FontWeight', 'bold');
% 


%% Needed Functions
function x_e = fkine_pose_num(q, dh_base, jt)
% FKINE_POSE_NUM  End-effector pose as a 6x1 vector.
%
% INPUTS:
%   q        - (n x 1) joint positions
%   dh_base  - (n x 4) DH table (zeros where q goes)
%   jt       - (n x 1) joint types: 1=revolute, 0=prismatic
%
% OUTPUT:
%   x_e      - (6 x 1) [px; py; pz; roll; pitch; yaw]
%              ZYX RPY convention (same as tr2rpy in Robotics Toolbox)
%
% Pure numeric — fully Simulink codegen compatible.

n   = numel(q);
T0i = fk_transforms_num(q, dh_base, jt);
T0n = T0i(:,:,n);           % end-effector transform

p = T0n(1:3, 4);            % position

% ZYX RPY extraction from rotation matrix R
R = T0n(1:3, 1:3);
pitch = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
roll  = atan2( R(3,2), R(3,3));
yaw   = atan2( R(2,1), R(1,1));

x_e = [p; roll; pitch; yaw];   % 6 x 1
end

function T0i = fk_transforms_num(q, dh_base, jt)
% FK_TRANSFORMS_NUM  Compute all T_0^i transforms for an n-DOF robot.
%
% INPUTS:
%   q        - (n x 1) joint positions [rad or m]
%   dh_base  - (n x 4) DH table [a, alpha, d, theta] with zeros where q goes
%   jt       - (n x 1) joint types: 1 = revolute, 0 = prismatic
%
% OUTPUT:
%   T0i      - (4 x 4 x n) stack of cumulative transforms T_0^1 ... T_0^n
%
% Pure numeric — fully Simulink codegen compatible.

n   = numel(q);
T0i = zeros(4, 4, n);

for i = 1:n
    a     = dh_base(i, 1);
    alpha = dh_base(i, 2);
    d     = dh_base(i, 3);
    theta = dh_base(i, 4);

    % Substitute joint variable into correct DH parameter
    if jt(i) == 1        % revolute  — q adds to theta
        theta = theta + q(i);
    else                 % prismatic — q adds to d
        d = d + q(i);
    end

    Ti = homo_trans_num(a, alpha, d, theta);

    if i == 1
        T0i(:,:,i) = Ti;
    else
        T0i(:,:,i) = T0i(:,:,i-1) * Ti;
    end
end
end

function T = homo_trans_num(a, alpha, d, theta)
% HOMO_TRANS_NUM  4x4 homogeneous transform from one DH row.
%   Standard DH convention: (a, alpha, d, theta)
%   Pure numeric — fully Simulink codegen compatible.

ct = cos(theta);  st = sin(theta);
ca = cos(alpha);  sa = sin(alpha);

T = [ct, -st*ca,  st*sa,  a*ct;
     st,  ct*ca, -ct*sa,  a*st;
      0,     sa,     ca,     d;
      0,      0,      0,     1];
end

