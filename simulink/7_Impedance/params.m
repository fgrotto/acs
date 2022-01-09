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

Kp = diag([100 50 100 1 1 1]);
Kd = diag([30 1 14 1 1 1]);
Md = diag([1 1 1 1 1 1]);
inv_Md = inv(Md);

% Environment like a box (3 walls in front of the robot)
K = 0.1*diag([1 1 1 1 1 1]);
param.Env = [0.6 10 10]; % wrt of frame 0
param.Kp = Kp;
param.K = K;


%% Calculation of derivative of Jacobians
% syms d3 real;
% syms l3 real;
% syms t1 real;
% syms d3 real;
% syms d_t1 real;
% syms d_d3 real;
% 
% q = [t1, d3];
% d_q = [d_t1, d_d3]; 
% 
% J = [[-sin(t1)*(d3 + l3), cos(t1), cos(t1)];
%         [ cos(t1)*(d3 + l3), sin(t1), sin(t1)];
%         [                 0,       0,       0];
%         [                 0,       0,       0];
%         [                 0,       0,       0];
%         [                 1,       0,       0]];
%     
% t = sym('t', 'real');
% q_time = [symfun('t1(t)', t), symfun('d3(t)', t)];
% dq_time = diff(q_time);
% 
% J_time = subs(J, q, q_time);
% dJ_time = diff(J_time);
% 
% d_J = simplify(subs(diff(J_time,t), [dq_time,q_time], [d_q,q]));

