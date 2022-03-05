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

Kp = diag([300 1000 300 1 1 1]);
Kd = diag([30 30 14 1 1 1]);
Md = diag([1 0.1 1 1 1 1]);
inv_Md = inv(Md);

% Environment like a box (3 walls in front of the robot)
K = 1*diag([200 1 1 1 1 1]);
param.Kp = Kp;
param.K = K;

% Environment only one component for a simplified force control
param.wall = 0.6;
param.axis = 1; % 1-x 2-y 3-z wrt of frame 0


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

% %%
%     
syms phi real;
syms theta real;
syms phi_dot real;
syms theta_dot real;

q = [phi, theta];
d_q = [phi_dot, theta_dot]; 

Ta =  [1, 0, 0, 0, 0, 0;
            0, 1, 0, 0, 0, 0;
            0, 0, 1, 0, 0, 0;
            0, 0, 0, 0, -sin(phi), cos(phi) * sin(theta);
            0, 0, 0, 0, cos(phi), sin(phi) * sin(theta);
            0, 0, 0, 1, 0, cos(theta)];
T_inv = Ta;
    
t = sym('t', 'real');
q_time = [symfun('phi(t)', t), symfun('theta(t)', t)];
dq_time = diff(q_time);

T_time = subs(T_inv, q, q_time);
dT_time = diff(T_time);

d_T = simplify(subs(diff(T_time,t), [dq_time,q_time], [d_q,q]));

