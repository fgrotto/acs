%% Calculate Direct Kinematics

Hb_0 = homogeneous(l0,0,0,pi/2);
H0_1 = homogeneous(0,t1,l1,0);
H1_2 = homogeneous(l2+d2,-pi/2,0,-pi/2);
H2_3 = homogeneous(l3+d3,pi/2,0,0);
H3_e = homogeneous(0,0,0,0); % it's the identify it doesn't care
Hb_e = Hb_0*H0_1*H1_2*H2_3;

% t1 = 0;
% H = eval(H1_2);
% rotm2eul(double(H(1:3,1:3)),'XYZ')

% Direct kinematics manually computed
% P = [(d3+l3)*(cos(t1))+l1*(cos(t1));
%     -d2-l3;
%     (d3+l3)*(sin(t1))+l1*(sin(t1))+l0;]
%
% Direct kinematics from base frame to ee using robotics toolbox
dk = getTransform(robot, config, 'ee', 'base_link');
% Direct kinematics from base frame to ee
Pb_e = Hb_e(1:3,4);