robot = loadrobot('universalUR5', 'DataFormat', 'column'); % add robot
n = numel(homeConfiguration(robot));

mode = 'impedance'; %switch for compliance 

% Impedance parameters
Md = diag([2, 2, 2, 0.5, 0.5, 0.5]);  % accel error (mass/inertial)
Kd = diag([40, 40, 40, 10, 10, 10]);  % vel error (damping)
Kp = diag([400, 400, 400, 100, 100, 100]);  % position error (stiffness)

% Compliance parameters
Kp_c = diag([300, 300, 300, 80, 80, 80]);  % position
Kv = diag([30, 30, 30, 8, 8, 8]);  % velocity
Kc = diag([500, 500, 500, 150, 150, 150]);  % force to position

% time configurations
dt = 0.001;
T = 5.0;
time = 0:dt:T;

q  = homeConfiguration(robot);
dq = zeros(n, 1);

T_d = getTransform(robot, q, 'tool0');
x_d = T_to_x(T_d);
dx_d = zeros(6, 1);
ddx_d = zeros(6, 1);

%Storage for plot
log_q = zeros(n, length(time));
log_tau = zeros(n, length(time));
log_e = zeros(6, length(time));

for i = 1:length(time)
    t = time(i);

    % external force (contact at t=2s)
    F_ext = zeros(6, 1);
    if t > 2.0
        F_ext(3) = -5.0;
    end

    % Kinematics
    T_ee = getTransform(robot, q, 'tool0');
    x_curr = T_to_x(T_ee);
    J = geometricJacobian(robot, q, 'tool0');
    dx_curr = J * dq;

    % Dynamics
    M = massMatrix(robot, q);
    C = velocityProduct(robot, q, dq);
    G = gravityTorque(robot, q);

    % Controller
    switch mode
        case 'impedance'
            tau = impedance_control(q, dq, x_d, dx_d, ddx_d, ...
                                    x_curr, dx_curr, J, M, C, G, ...
                                    F_ext, Md, Bd, Kd, robot);

        case 'compliance'
            tau = compliance_control(q, dq, x_d, dx_d, ...
                                     x_curr, dx_curr, J, G, ...
                                     F_ext, Kp, Kv, Kc, n);
    end
    
    % Reduce Physicalities
    tau_max = 150 * ones(n, 1);
    tau = max(min(tau, tau_max), -tau_max);
    ddq = M \ (tau - C - G + J' * F_ext);
    dq = dq + ddq * dt;
    q = q  + dq  * dt;

    log_q(:,i) = q;
    log_tau(:,i) = tau;
    log_e(:,i) = x_d - x_curr;
end

%% Plot
figure;
subplot(3,1,1)
plot(time, log_e(1:3,:)')
title(['Cartesian position error: ' mode]); ylabel('meters'); xlabel('time (s)')
legend('x','y','z'); grid on

subplot(3,1,2)
plot(time, log_tau')
title('Joint torques'); ylabel('N . m'); xlabel('time (s)')
grid on

subplot(3,1,3)
plot(time, log_q')
title('Joint positions'); ylabel('rad'); xlabel('time (s)')
grid on


%% functions

% Impedance Control
function tau = impedance_control(q, dq, x_d, dx_d, ddx_d, ...
                                  x_curr, dx_curr, J, M, C, G, ...
                                  F_ext, Md, Bd, Kd, robot)
    n  = length(q);
    e  = x_d  - x_curr;
    de = dx_d - dx_curr;
    ddx_imp = ddx_d + Md \ (Bd * de + Kd * e + F_ext);
    J_dot = compute_Jdot(robot, q, dq);
    J_pinv = pinv(J);
    ddq_imp = J_pinv * (ddx_imp - J_dot * dq);
    tau = M * ddq_imp + C + G - J' * F_ext;

    if n > 6
        tau = tau + null_space_torque(q, dq, J, n);
    end
end



% compliance control
function tau = compliance_control(q, dq, x_d, dx_d, ...
                                   x_curr, dx_curr, J, G, ...
                                   F_ext, Kp, Kv, Kc, n)

    x_d_comply = x_d + Kc \ F_ext;

    e = x_d_comply - x_curr;
    de = dx_d  - dx_curr;

    tau = G + J' * (Kp * e + Kv * de);

    if n > 6
        tau = tau + null_space_torque(q, dq, J, n);
    end
end

%transformation
function x = T_to_x(T)
    pos = T(1:3, 4);
    euler = rotm2eul(T(1:3,1:3), 'ZYX')';
    x = [pos; euler];
end

%jacobian
function J_dot = compute_Jdot(robot, q, dq)
    eps = 1e-6;
    J0 = geometricJacobian(robot, q,'tool0');
    J1 = geometricJacobian(robot, q + dq*eps, 'tool0');
    J_dot = (J1 - J0) / eps;
end

%operational space
function tau_null = null_space_torque(q, dq, J, n)
    q_min = -pi * ones(n, 1);
    q_max = pi * ones(n, 1);
    q_mid = (q_max + q_min) / 2;

    k_null = 10;
    d_null = 2;

    N = eye(n) - pinv(J) * J;
    tau_0 = -k_null * (q - q_mid) - d_null * dq;
    tau_null = N * tau_0;
end