clear all; clc; close all;
clear all; clc; close all;

%% initial inputs (DH table and joint types, parameters optional)

% example: from input app
input = MAE547_Final_Project_App();
uiwait(input.UIFigure)

x = input.jointCount;
disp(x)

dh_raw      = input.dh_raw;
joint_types = input.joint_types;
g0          = input.g0;
params      = input.params;

% example: cylindrical robot
% dh_raw = [0,     0,  0,     0;
%           0, -pi/2,  0,     0;
%           0,     0,  0,     0];
% joint_types = ["R"; "P"; "P"];
% 
% g0 = [0 0 -9.81];
% 
% params.m_l = [1, 1, 1];
% params.m_m = [0.1 0.1 0.1];
% params.I_l = [0.01 0.01 0.01];
% params.I_m = [0.001 0.001 0.001];
% params.k_r = [10 10 10];

%% generate symbolic variables

N = length(joint_types);
if size(dh_raw, 1) ~= N
    disp("DH table and Joint Types don't have the same length.")
    return
else
    fprintf("Robot specified with %d joints, with types %s\n", N, join(joint_types));
end

q = sym('q',[1 N]);
assume(q, 'real')

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

[EOM2] = EOM(dh,m_l,m_m,I_l,I_m,k_r,g0, joint_types);

B = EOM2.B;
C = EOM2.c;
G = EOM2.G;

