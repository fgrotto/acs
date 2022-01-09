%% Differential Kinematics 

% Let's evaluate the analytical jacobian using the jacobian matlab function
% which uses the symbolic toolbox for evaluation
%
% Analytical Jacobian manually calculated (from base to ee)
Ja = [-sin(t1)*(d3+l3+l1)    0   cos(t1);
        0                   -1   0;
     cos(t1)*(l1+d3+l3)      0   sin(t1);
        1                    0   0;
        0                    0   0;
        0                    0   0;];
JaLinearVel =   jacobian(Pb_e, [t1;d2;d3]);
JaOrientation = [1 0 0; 0 0 0; 0 0 0];
Ja_sym = [JaLinearVel; JaOrientation];


% Geometric Jacobians (from base to ee)
% The manually computed geometric jacobian is reported
% J = [-sin(t1)*(d3+l3+l1)   0   cos(t1);
%         0                   -1   0;
%      cos(t1)*(l1+d3+l3)    0   sin(t1);
%         0                    0   0;
%        -1                    0   0;
%         0                    0   0;]
tmp1 = (Hb_0*H0_1);
tmp2 = (Hb_0*H0_1*H1_2);
JP1 = cross(Hb_0(1:3,3), Pb_e-Hb_0(1:3,4));
JP2 = tmp1(1:3,3); 
JP3 = tmp2(1:3,3); 
J01 = [0; -1; 0];
J02 = [0; 0; 0;];
J03 = [0; 0; 0;];

JP = [JP1 JP2 JP3];
J0 = [J01 J02 J03];

% Put together the upper and the lower part to obtain the full geometric
% jacobian from the base frame to the end effector
J = [JP; J0];
geometricJacobian = geometricJacobian(robot, config, 'ee');

%% Jacobians wrt of the frame 0
% Let's calculate the geometric jacobian wrt of the 0 frame
%
% Manually computed version wrt of frame 0
% J0 = [-sin(t1)*(l1 + l3 + d3) 0 cos(t1);
%  cos(t1)*(l1 + l3 + d3) 0 sin(t1);
%                         0 1       0;
%                         0 0       0;
%                         0 0       0;
%                         1 0       0]
%
tmp1 = (H0_1*H1_2);
tmp2 = (H0_1*H1_2*H2_3);
P0_e = tmp2(1:3,4);
JP1 = cross(H0_1(1:3,3), P0_e-H0_1(1:3,4));
JP2 = tmp1(1:3,3); 
JP3 = tmp2(1:3,3); 
J01 = H0_1(1:3,3);
J02 = [0; 0; 0;];
J03 = [0; 0; 0;];

JP = [JP1 JP2 JP3];
J0 = [J01 J02 J03];

% Put together the upper and the lower part to obtain the full geometric
% jacobian from the 0 frame to the ee
J0_e_sym = [JP; J0];

% Manually computed analytical jacobian wrt of frame 0
Ja0_e = [-sin(t1)*(l1 + l3 + d3) 0 cos(t1);
 cos(t1)*(l1 + l3 + d3) 0           sin(t1);
                        0 1         0;
                        1 0         0;
                        0 0         0;
                        0 0         0];
% Symbolic computed analytical jacobian wrt of frame 0                  
JaLinearVel =   jacobian(P0_e, [t1;d2;d3]);
JaOrientation = [1 0 0; 0 0 0; 0 0 0];
Ja0_e_sym = [JaLinearVel; JaOrientation];