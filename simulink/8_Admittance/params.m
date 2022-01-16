clear all;
addpath("/home/filippo/univr/acs/simulink/model/.");

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

Kp = diag([500 100 100 1 1 1]);
Kd = diag([30 20 14 1 1 1]);
Md = diag([1 1 1 1 1 1]);
inv_Md = inv(Md);

% Environment like a box (3 walls in front of the robot)
K = 10*diag([1 1 1 1 1 1]);
param.Kp = Kp;
param.K = K;

% Environment only one component for a simplified force control
param.wall = 0.6;
param.axis = 1; % 1-x 2-y 3-z wrt of frame 0

% Admittance Controller
Mt = diag([1 1 1 1 1 1]);
KPt = diag([0.1 1 1.3 1 1 1]);
KDt = diag([50 1 2 1 1 1]);
