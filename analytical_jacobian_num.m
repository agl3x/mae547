function JA = analytical_jacobian_num(q, dh_base, jt)
% ANALYTICAL_JACOBIAN_NUM  Analytical Jacobian J_A(q) — 6 x n.
%
%   ẋ_e = J_A * q̇    where ẋ_e = [ṗ; φ̇]  (position + RPY angle rates)
%
%   Built from geometric J and T_A:
%       J_A = blkdiag(I3, T_A^-1) * J
%
%   ZYX RPY T_A  (ω = T_A * [roll_dot; pitch_dot; yaw_dot]):
%       T_A = [cos(yaw)*cos(pitch), -sin(yaw), 0]
%             [sin(yaw)*cos(pitch),  cos(yaw), 0]
%             [-sin(pitch),          0,        1]
%
%   T_A is singular at pitch = ±90°. pinv handles this gracefully.
%
% INPUTS:
%   q        - (n x 1) joint positions
%   dh_base  - (n x 4) DH table
%   jt       - (n x 1) joint types: 1=revolute, 0=prismatic
%
% OUTPUT:
%   JA       - (6 x n) analytical Jacobian
%
% Pure numeric — fully Simulink codegen compatible.

J   = geom_jacobian_num(q, dh_base, jt);       % 6 x n
x_e = fkine_pose_num(q, dh_base, jt);          % 6 x 1

pitch = x_e(5);
yaw   = x_e(6);

TA = [ cos(yaw)*cos(pitch), -sin(yaw),  0;
       sin(yaw)*cos(pitch),  cos(yaw),  0;
      -sin(pitch),            0,         1];

TA_inv = pinv(TA);             % robust to singularity at pitch = ±90°

JA         = J;
JA(4:6,:)  = TA_inv * J(4:6,:);   % only angular rows change
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


function J = geom_jacobian_num(q, dh_base, jt)
% GEOM_JACOBIAN_NUM  Geometric Jacobian J(q) at the end-effector — 6 x n.
%
%   ẋ = J * q̇    where ẋ = [v; ω]  (linear + angular velocity)
%
%   Uses the standard formula:
%     Revolute  j:  Jp_j = z_{j-1} x (p_n - p_{j-1})
%                   Jo_j = z_{j-1}
%     Prismatic j:  Jp_j = z_{j-1}
%                   Jo_j = 0
%
% INPUTS:
%   q        - (n x 1) joint positions
%   dh_base  - (n x 4) DH table
%   jt       - (n x 1) joint types: 1=revolute, 0=prismatic
%
% OUTPUT:
%   J        - (6 x n) geometric Jacobian
%
% Pure numeric — fully Simulink codegen compatible.

n   = numel(q);
T0i = fk_transforms_num(q, dh_base, jt);
p_n = T0i(1:3, 4, n);          % end-effector position

Jp = zeros(3, n);
Jo = zeros(3, n);

for j = 1:n
    if j == 1
        z_j1 = [0; 0; 1];      % base z-axis
        p_j1 = [0; 0; 0];      % base origin
    else
        z_j1 = T0i(1:3, 3, j-1);
        p_j1 = T0i(1:3, 4, j-1);
    end

    if jt(j) == 1              % revolute
        Jp(:,j) = cross(z_j1, p_n - p_j1);
        Jo(:,j) = z_j1;
    else                       % prismatic
        Jp(:,j) = z_j1;
        Jo(:,j) = [0; 0; 0];
    end
end

J = [Jp; Jo];                  % 6 x n
end