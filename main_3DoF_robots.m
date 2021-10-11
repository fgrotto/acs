
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
show(robot,config);

% config(1).JointPosition = 0.1;
% config(2).JointPosition = pi/3;
% config(3).JointPosition = 0.1;
% 
% 
% show(robot,config);
% xlim([-0.5 0.8])
% ylim([-0.5 0.5])
% zlim([0 0.8])



