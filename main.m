clear all;
close all;

%% Use the robotic toolbox to load the urdf file of my rpp robot
% Load our robot and display a configuration
robot = importrobot('RPP.urdf');
showdetails(robot)
config = homeConfiguration(robot);
config(1).JointPosition = pi/4;
config(2).JointPosition = 0;
config(3).JointPosition = 0.1;

% figure;
% show(robot,config);

%% Put symbols used for the entire project according to our robot rpp
% Links parameters
syms l0 real;
syms l1 real;
syms l2 real;
syms l3 real;
syms r1 real;
syms b2 real; % base of the prismatic joint to b2*b2
syms b3 real; % base of the prismatic joint to b3*b3

% Joint parameters (positions, velocites and accelerations) 
syms t1 real;
syms d2 real;
syms d3 real;
syms d_t1 real;
syms d_d2 real;
syms d_d3 real;
syms dd_t1 real;
syms dd_d2 real;
syms dd_d3 real;

% Operational space variables (dynamic model)
syms x_dd y_dd z_dd phi_dd theta_dd psi_dd real;
dd_x = [x_dd; y_dd; z_dd; phi_dd; theta_dd; psi_dd];

% Masses of the links
syms m1 real;
syms m2 real;
syms m3 real;
syms g real;
pi = sym(pi);

% Forces and torques for rne
f_1 =  sym('f_1', [3 1], 'real');
f_2 =  sym('f_2', [3 1], 'real');
f_3 =  sym('f_3', [3 1], 'real');
f_e =  sym('f_e', [3 1], 'real');
mu_1 =  sym('mu_1', [3 1], 'real');
mu_2 =  sym('mu_2', [3 1], 'real');
mu_3 =  sym('mu_3', [3 1], 'real');
mu_e =  sym('mu_e', [3 1], 'real');

% Frictions coefficients for each joint
Fvi = sym('Fvi',[3 1]);
Fsi = sym('Fsi',[3 1]);

% Group together joint values
q = [t1,d2,d3];
d_q = [d_t1,d_d2,d_d3];
dd_q = [dd_t1,dd_d2,dd_d3];
dof = 3;

kinematics_direct;
kinematics_inverse;
kinematics_differential;
energies;
coriolis_centrifugal_matrix;
gravity_matrix;
rne;
operational_space_dynamic_model;
evaluate_results;