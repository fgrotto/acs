clear all;
close all;

%% Use the robotic toolbox to load the urdf file of my rpp robot
% Load our robot and display a configuration
robot = importrobot('RPP.urdf');
showdetails(robot)
config = homeConfiguration(robot);
config(1).JointPosition = pi/4;
config(2).JointPosition = -0.2;
config(3).JointPosition = -0.2;

%% Put symbols used for the entire project according to our robot rpp
syms l0 real;
syms l1 real;
syms l2 real;
syms l3 real;
syms t1 real;
syms d2 real;
syms d3 real;
pi = sym(pi);

kinematics_direct;
kinematics_inverse;
kinematics_differential;
energies;
coriolis_matrix;
gravity_matrix;


%% Evaluate some results (B, U, T) and dynamic model of the manipulator (Lagrangian formulation)

% You can choose the values
t1 = pi;
d3 = -0.2;
d2 = -0.2;
g = 9.81;

% From the urdf provided
l0 = 0.4;
l3 = 0.3/2;
l1 = 0.4;
l2 = 0.46;
r1 = 0.02;
b2 = 0.03;
b3 = 0.01;

% Assuming homogenous volume and a density of aluminim
density = 2700; % (kg/m^3)
m1 = (pi*r1^2*l1) * density;
m2 = (b2*b2*l2) * density;
m3 = (b3*b3*l3) * density;

% Evaluate B to check if it is positive definite
eval(B)
eval(U)
eval(T)
eval(C)

% Write down the symbolic equation of the robot
tau = B * ddq + C * dq + G;
tau = simplify(tau);

% Put some values in the velocities and accelerations
dt1 = 2;
dd2 = 2;
dd3 = 2;
ddt1 = 0;
ddd2 = 0.1;
ddd3 = 0.2;

eval(tau)