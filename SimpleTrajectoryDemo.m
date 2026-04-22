clear all; close all; clc;

t_i = 0;
t_f = 10;

x_d = SimpleTrajectory(t_i, t_f, 10*rand(6, 1)-5, 10*rand(6, 1)-5, 3);
disp(x_d)


figure(1)
hold on;
for i=1:length(x_d)
    fplot(x_d(i), [t_i, t_f], 'LineWidth', 2)
end
grid on;


