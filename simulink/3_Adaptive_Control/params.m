clear;

m = 1;

% Params of 1dof link
param.I = 0.1;
param.F = 0.2;
param.G = m*9.81*0.2; % mgd

K = diag([0.3]);
lambda = 30;

% PD Controller parameters
Kp = diag([1]);
Kd = diag([0.2]);




