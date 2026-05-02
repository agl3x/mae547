function [o_d, odot_d, oddot_d, R_d, omega_d, omegadot_d] = trajectory(x_d, t)
% x_d : 6x1 [x; y; z; psi(t); theta(t); phi(t)]
%   psi   = yaw (Z), theta = pitch (Y), phi = roll (X)
%   ZYX: R_d = Rz(psi) * Ry(theta) * Rx(phi)

dt = 1e-5;

% -----------------------------------------------------------------------
% USER DEFINES TRAJECTORY HERE
% x_d is passed in from GUI for position (rows 1-3).
% For angles, define them explicitly as functions of t so we can
% differentiate at t+dt and t-dt.
% Replace these with your actual angle equations:
psi_fn   = @(tt) x_d(4);   % e.g. @(tt) 0.5*sin(tt)
theta_fn = @(tt) x_d(5);   % e.g. @(tt) 0.1*tt
phi_fn   = @(tt) x_d(6);   % e.g. @(tt) 0
% -----------------------------------------------------------------------

% --- Evaluate angles at t-dt, t, t+dt ---
angles_m = [x_d(1:3); psi_fn(t-dt); theta_fn(t-dt); phi_fn(t-dt)];
angles_0 = [x_d(1:3); psi_fn(t);    theta_fn(t);    phi_fn(t)];
angles_p = [x_d(1:3); psi_fn(t+dt); theta_fn(t+dt); phi_fn(t+dt)];

% --- Position (pass-through from x_d) ---
o_d     = x_d(1:3);
odot_d  = (angles_p(1:3) - angles_m(1:3)) / (2*dt);
oddot_d = (angles_p(1:3) - 2*angles_0(1:3) + angles_m(1:3)) / dt^2;

% --- Rotation matrices ---
R_m = eul2rot_ZYX(angles_m(4), angles_m(5), angles_m(6));
R_0 = eul2rot_ZYX(angles_0(4), angles_0(5), angles_0(6));
R_p = eul2rot_ZYX(angles_p(4), angles_p(5), angles_p(6));

R_d = R_0;

% --- Angular velocity and acceleration ---
Rdot  = (R_p - R_m) / (2*dt);
Rddot = (R_p - 2*R_0 + R_m) / dt^2;

omega_d    = vex(Rdot  * R_0');
omegadot_d = vex(Rddot * R_0' + Rdot * Rdot');

% --- Ensure column vectors ---
o_d        = o_d(:);
odot_d     = odot_d(:);
oddot_d    = oddot_d(:);
omega_d    = omega_d(:);
omegadot_d = omegadot_d(:);
end

function R = eul2rot_ZYX(psi, theta, phi)
Rz = [ cos(psi) -sin(psi) 0;
        sin(psi)  cos(psi) 0;
        0         0        1];
Ry = [ cos(theta) 0 sin(theta);
        0          1 0;
       -sin(theta) 0 cos(theta)];
Rx = [1 0        0;
       0 cos(phi) -sin(phi);
       0 sin(phi)  cos(phi)];
R  = Rz * Ry * Rx;
end

function v = vex(S)
    v = [S(3,2); S(1,3); S(2,1)];
end