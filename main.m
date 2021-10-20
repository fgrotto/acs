clear all;
close all;
% help show
% help rigidBodyJoint
% help rigidBodyTree
% help rigidBody

% https://it.mathworks.com/help/robotics/ug/rigid-body-tree-robot-model.html#responsive_offcanvas
% https://it.mathworks.com/help/robotics/ref/rigidbodytree.html
% https://it.mathworks.com/help/robotics/ref/rigidbody.html
% https://it.mathworks.com/help/robotics/ref/rigidbodytree.show.html

% robot = importrobot('iiwa14.urdf');

% robot = importrobot('pivot_giunti.urdf');
% robot = importrobot('PRR.urdf');
robot = importrobot('RPP.urdf');
% robot = importrobot('PPR.urdf');
% robot = importrobot('RRP.urdf');
% robot = importrobot('PRP.urdf');
% robot = importrobot('RRP_2.urdf');

% robot_PRP.DataFormat = 'column';
% config = [0.1, pi/3, -0.15]';

showdetails(robot)

figure;
config = homeConfiguration(robot);
config(1).JointPosition = 0.1;
config(2).JointPosition = 0;
config(3).JointPosition = 0;

show(robot,config);
xlim([-0.5 0.8])
ylim([-0.5 0.5])
zlim([0 0.8])

%% Calculate direct kinematics

dk = getTransform(robot, config, 'ee', 'base_link')

t1 = 0.1;
d3 = 0;
d2 = 0;
L0h = 0.4;
L3h = 0.3/2;
L1h = 0.4;
L2h = 0.46;

% Direct kinematics
P = [(d3+L3h)*(cos(t1))+L1h*(cos(t1));
    -d2-L2h;
    (d3+L3h)*(sin(t1))+L1h*(sin(t1))+L0h;]


% Geometric Jacobians
geometricJacobian = geometricJacobian(robot, config, 'ee')

J = [-sin(t1)*(d3+L3h+L1h)   0   cos(t1);
        0                   -1   0;
     cos(t1)*(L1h+d3+L3h)    0   sin(t1);
        0                    0   0;
       -1                    0   0;
        0                    0   0;]

% Inverse kinematics
x = P(1,1);
y = P(2,1);
z = P(3,1);

d2
d2_calc = -L2h-y

t1
t1_calc =atan2(z-L0h, x)

d3
d3_calc1 = -L1h - sqrt(L1h^2+(x^2+(z-L0h)^2-L1h^2)) - L3h
d3_calc2 = -L1h + sqrt(L1h^2+(x^2+(z-L0h)^2-L1h^2)) - L3h % Correct one
    
% Analytical Jacobian
Ja = [-sin(t1)*(d3+L3h+L1h)   0   cos(t1);
        0                   -1   0;
     cos(t1)*(L1h+d3+L3h)    0   sin(t1);
        1                    0   0;
        0                    0   0;
        0                    0   0;]
    
% %% Inverse kinementics
% 
% ik = inverseKinematics('RigidBodyTree', robot);
% 
% weights = [1 1 1 1 1 1];
% 
% % [configSol,solInfo] = ik(endeffector,pose,weights,initialguess)
% % Weight for pose tolerances, specified as a six-element vector. The first three elements
% % correspond to the weights on the error in orientation for the desired pose. The last 
% % three elements correspond to the weights on the error in xyz position for the desired 
% % pose.
% [configSol, solInfo] = ik('ee', dk, weights, robot.homeConfiguration);