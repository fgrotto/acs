clear;

m = 1;

% Params of 1dof link
param.I = 0.1;
param.F = 0.2;
param.G = m*9.81*0.2; % mgd

K = 0.3;
lambda = 0.99;

% PD Controller parameters
Kp = diag([10]);
Kd = diag([0.2]);




