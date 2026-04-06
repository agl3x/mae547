%% MAE 547 Project Dynamics Test
clc; clear; close all;
% just testing a few of the commands in the toolbox
% Doesn't solve for EOM analytically

%% 
DH = [
  1, 0, 0, 0;
  1, 0, 0, 0;
]; % 2 link revolute (a, alpha, d, theta)

q = [pi/2 , pi/2];
qd = [0.1, 0.2];
qdd = [0.2, 0.3];


joint_Type = ['R','R'];

mass = [1, 1];

go = [0, 0, -9.81];
n = length(DH(:,1)); % # of joints

for i = 1:n
    a = DH(i,1);
    alpha =DH(i,2);
    
    if joint_Type(i) == 'R'
        d = DH(i,3); 
        L{i} = Link('d',d,'a',a,'alpha',alpha);
    else
        theta = DH(i,4);
        L{i} =Link('a',a,'alpha',alpha,'theta',theta);
    end
    
    L{i}.m = mass(i);

end

R = SerialLink([L{1}, L{2}]); 

B = R.inertia(q); %inertia matrix

C = R.coriolis(q, qd); % coriolis

G = R.gravload(q); % gravity

tau = R.rne(q, qd, qdd); % inverse dynamics

