function [o_d, odot_d, oddot_d, R_d, omega_d, omegadot_d] = trajectory(t, ...
    a, alpha, d, theta, ...
    xi, xf, tf, deg)

n = length(a);

Ri = dh_rotation(a, alpha, d, theta, xi, n);
Rf = dh_rotation(a, alpha, d, theta, xf, n);

ti = 0;
tc  = max(ti, min(tf, t));
tau = (tc - ti) / (tf - ti);
dt  = tf - ti;

if deg >= 5
    bs = 6*tau^5  - 15*tau^4  + 10*tau^3;
    bsd = (30*tau^4 - 60*tau^3  + 30*tau^2) / dt;
    bsdd = (120*tau^3 - 180*tau^2 + 60*tau) / dt^2;
else
    bs = 3*tau^2-2*tau^3;
    bsd = (6*tau-6*tau^2) / dt;
    bsdd = (6-12*tau) / dt^2;
end

delta = xf(:) - xi(:);
o_d = xi(:) + bs*delta;
odot_d  = bsd * delta;
oddot_d = bsdd * delta;

dR= Ri' * Rf;
log_dR = mat_log_so3(dR);
R_d = Ri * mat_exp_so3(bs * log_dR);
omega_d = vex(Ri * (bsd * log_dR) * R_d');
omegadot_d = vex(Ri * (bsdd * log_dR) * R_d');

o_d = o_d(:);
odot_d = odot_d(:);
oddot_d = oddot_d(:);
omega_d = omega_d(:);
omegadot_d = omegadot_d(:);

end

function R = dh_rotation(a, alpha, d, theta_offset, q, n)
%T_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
    T = eye(4);
    for i = 1:n
        ct = cos(q(i) + theta_offset(i));
        st = sin(q(i) + theta_offset(i));
        ca = cos(alpha(i));
        sa = sin(alpha(i));
        Ti = [ct  -st*ca   st*sa   a(i)*ct;
               st   ct*ca  -ct*sa   a(i)*st;
                0      sa      ca      d(i);
                0       0       0         1];
        T = T*Ti;
    end
    R = T(1:3, 1:3);
end

function S = skew(v)
    S = [ 0    -v(3)  v(2);
          v(3)  0    -v(1);
         -v(2)  v(1)  0];
end

function v = vex(S)
    v = [S(3,2); S(1,3); S(2,1)];
end

function R = mat_exp_so3(S)
    v = vex(S);
    theta = norm(v);
    if theta < 1e-10
        R = eye(3);
    else
        K = S / theta;
        R = eye(3) + sin(theta)*K + (1 - cos(theta))*(K*K);
    end
end

function S = mat_log_so3(R)
    val = max(-1, min(1, (trace(R) - 1) / 2));
    theta = acos(val);
    if theta < 1e-10
        S = zeros(3,3);
    elseif abs(theta - pi) < 1e-6
        B = (R + R') / 2 - eye(3);
        [~, idx] = max(diag(B));
        v = sqrt(max(0, B(idx,idx) + 1));
        axis_vec = R(:,idx) / v;
        S = skew(theta * axis_vec);
    else
        S = (theta / (2*sin(theta))) * (R - R');
    end
end