clear;

m = 1;

% Params of 1dof link
param.I = 0.1;
param.F = 0.2;
param.G = m*9.81*0.2; % mgd

K = diag([1.2]);
lambda = 40;

% PD Controller parameters
Kp = diag([25]);
Kd = diag([10]);


Ts = 0.001;
