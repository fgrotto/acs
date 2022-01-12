clear;

% URDF Parameters
param.m1 = 1.3565;
param.m2 = 1.1178;
param.m3 = 0.0405;
param.pi = 3.14;

% From the urdf provided
param.l0 = 0.4;
param.l3 = 0.3/2;
param.l1 = 0.4;
param.l2 = 0.46;
param.r1 = 0.02;
param.b2 = 0.03;
param.b3 = 0.01;
param.g = 9.81;
param.Fsv = 0;

l1 = param.l1;
l2 = param.l2;
l3 = param.l3;

t1 = 0.2;
d2 = 0.2;
d3 = 0.1;

% H0_3 computed with the main.m
H = [[0,  sin(t1), cos(t1), cos(t1)*(d3 + l3) + l1*cos(t1)]
    [0, -cos(t1), sin(t1), sin(t1)*(d3 + l3) + l1*sin(t1)]
    [1,        0,       0,                        d2 + l2]
    [0,        0,       0,                              1]];


% Return x with positions and orientations
xd = [ H(1:3,4); rotm2eul(H(1:3,1:3), 'ZYZ')'];

% Environment only one component for a simplified force control
param.wall = 0.2;
param.axis = 2; % 1-x 2-y 3-z wrt of frame 0
param.K = diag([10 100 10 0 0 0]);

% Desired Force
Fd = [0 0.2 0 0 0 0];

% Impedance
% PD Controller parameters
Kp = diag([700 700 700 1 1 1]);
Kd = diag([50 50 50 1 1 1]);
Md = diag([1 1 1 1 1 1]);
inv_Md = inv(Md);

% Compliance
Kf = diag([3 1 1.4 1 1 1]);
Ki = diag([3 1 1.4 1 1 1]);


