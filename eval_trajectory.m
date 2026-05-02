function [x_d, xd_d,xdd_d] = eval_trajectory(t)
% EVAL_TRAJECTORY  Evaluates x_d(t) and xd_d(t) function handles
%                  stored in the base workspace by the run script.
%
%   Called as an extrinsic from cc_trajectory.m — runs in interpreted
%   mode so evalin is allowed.
%
%   The run script must define before sim():
%       x_d  = @(t) [...]   6x1 function handle
%       xd_d = @(t) [...]   6x1 function handle

if mode == 1
    x_d_fn  = evalin('base', 'x_d');
    xd_d_fn = evalin('base', 'xd_d');
    x_d  = x_d_fn(t);    % 6x1
    xd_d = xd_d_fn(t);   % 6x1
else
    x_d_fn  = evalin('base', 'x_d');
    xd_d_fn = evalin('base', 'xd_d');
    xdd_d_fn = evalin('base', 'xdd_d');
    x_d  = x_d_fn(t);    % 6x1
    xd_d = xd_d_fn(t);   % 6x1
    xdd_d = xdd_d_fn(t);   % 6x1
end

end
 