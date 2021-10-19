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
show(robot, config);

config(1).JointPosition = 0;
config(2).JointPosition = 0;
config(3).JointPosition = 0;


show(robot,config);
% xlim([-0.5 0.8])
% ylim([-0.5 0.5])
% zlim([0 0.8])

%% Calculate direct kinematics

dk = getTransform(robot, config, 'base_link', 'ee')

%% Inverse kinementics

ik = inverseKinematics('RigidBodyTree', robot);

weights = [1 1 1 1 1 1];

% [configSol,solInfo] = ik(endeffector,pose,weights,initialguess)
% Weight for pose tolerances, specified as a six-element vector. The first three elements
% correspond to the weights on the error in orientation for the desired pose. The last 
% three elements correspond to the weights on the error in xyz position for the desired 
% pose.
[configSol, solInfo] = ik('ee', dk, weights, robot.homeConfiguration);

%% Calculate geometric jacobian

geometricJacobian = geometricJacobian(robot, config, 'ee')
