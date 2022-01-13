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

% PD Controller parameters
Kp = diag([600 380 250]);
Kd = diag([200 40 15]);





