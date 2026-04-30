%% EOM Test
clc; clear;

%% Case 1 - 2 revolute
syms kr1 kr2 L1 L2 t1 t2 m1 m2 mm1 mm2 IL1 IL2 IM1 IM2

dh = [L1, 0, 0, t1;
      L2, 0, 0, t2];

kr = [kr1; kr2];

Joint_type = ["R"; "R"];

g0 = [0 -9.81 0];

m = [m1 m2];
mm = [mm1 mm2];
IL = [IL1 IL2];
Im = [IM1 IM2];

[EOM1] = EOM(dh,m,mm,IL,Im, kr,g0, Joint_type);

u = @(t,q,qd) [cos(t);sin(t)];

params.L1  = 1.0;
params.L2  = 0.8;
params.m1  = 2.0;
params.m2  = 1.5;
params.mm2 = 0.5;
params.IL1 = 0.1;   
params.IL2 = 0.05;
params.IM1 = 0.01;
params.IM2 = 0.01;
params.kr1 = 10;
params.kr2 = 10;

[t_out1,q_out1,q_out2]=DynSim_EOM(EOM1,params,u,[0,0],[0,0],[0,10]);

%% Case 2 - 2 prismatic
syms kr1 kr2  d1 d2 m1 m2 mm1 mm2 IL1 IL2 IM1 IM2

dh = [0, -pi/2, d1, -pi/2;
      0, 0, d2, 0];

kr = [kr1; kr2];

Joint_type = ["P"; "P"];

g0 = [0 0 -9.81];

m = [m1 m2];
mm = [mm1 mm2];
IL = [IL1 IL2];
Im = [IM1 IM2];

[EOM2] = EOM(dh,m,mm,IL,Im,kr,g0, Joint_type);

