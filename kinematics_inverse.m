%% Inverse kinematics

% Let's evaluate the values of [x;y;z] using the computed direct kinematics
x = Pb_e(1,1);
y = Pb_e(2,1);
z = Pb_e(3,1);

% Compare the actual value with the one generated from the direct
% kinematics and getting back the value with the inverse one
d2;
d2_calc = -l2-y;

t1;
t1_calc = atan2(z-l0, x);

d3;
% The first solution is numerical valid but has no physical meaning so we
% can easily drop it and use the second solution
% d3_calc1 = -l1 - sqrt(l1^2+(x^2+(z-l0)^2-l1^2)) - l3
d3_calc = -l1 + sqrt(l1^2+(x^2+(z-l0)^2-l1^2)) - l3;

% You can compare it also with the robotic toolbox using the following
% commands
%
% ik = inverseKinemageometricJacobian = geometricJacobian(robot, config, 'ee')tics('RigidBodyTree', robot);
% 
% weights = [1 1 1 1 1 1];
% 
% % [configSol,solInfo] = ik(endeffector,pose,weights,initialguess)
% % Weight for pose tolerances, specified as a six-element vector. The first three elements
% % correspond to the weights on the error in orientation for the desired pose. The last 
% % three elements correspond to the weights on the error in xyz position for the desired 
% % pose.
% [configSol, solInfo] = ik('ee', dk, weights, robot.homeConfiguration);