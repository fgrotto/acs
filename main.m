clear all;
close all;

%% Use the robotic toolbox to load the urdf file of my rpp robot
% Load our robot and display a configuration
robot = importrobot('RPP.urdf');
showdetails(robot)

figure;
config = homeConfiguration(robot);
config(1).JointPosition = pi/4;
config(2).JointPosition = -0.2;
config(3).JointPosition = -0.2;

% Print an image with the actual robot
% show(robot,config);
% xlim([-0.5 0.8])
% ylim([-0.5 0.5])
% zlim([0 0.8])

%% Put symbols used for the entire project according to our robot rpp
syms l0 real;
syms l1 real;
syms l2 real;
syms l3 real;
syms t1 real;
syms d2 real;
syms d3 real;

%% Calculate Direct Kinematics

Hb_0 = homogeneous(l0,0,0,pi/2);
H0_1 = homogeneous(0,t1,l1,0);
H1_2 = homogeneous(l2+d2,pi/2,0,pi/2);
H2_3 = homogeneous(l3+d3,pi/2,0,0);
H3_e = homogeneous(0,0,0,0); % it's the identify it doesn't care
Hb_e = Hb_0*H0_1*H1_2*H2_3;


% Direct kinematics manually computed
% P = [(d3+l3)*(cos(t1))+l1*(cos(t1));
%     -d2-l3;
%     (d3+l3)*(sin(t1))+l1*(sin(t1))+l0;]
%
% Direct kinematics from base frame to ee using robotics toolbox
dk = getTransform(robot, config, 'ee', 'base_link');
% Direct kinematics from base frame to ee
Pb_e = Hb_e(1:3,4);

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
% d3_calc1 = -l1 - sqrt(l1^2+(x^2+(z-L0h)^2-l1^2)) - l3
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
Ja = [JaLinearVel; JaOrientation];


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
Jb_e_sym = [JP; J0];
geometricJacobian = geometricJacobian(robot, config, 'ee');

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

% Manually computer analytical jacobian wrt of frame 0
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

% syms ph;
% syms th;
% Ta = [1 0, 0, 0,        0,               0;
% 0 1, 0 0        0               0;
% 0 0, 1 0        0               0;
% 0 0, 0 0 -sin(ph) cos(ph)*sin(th);
% 0 0, 0 0  cos(ph) sin(ph)*sin(th);
% 0 0, 0 1        0         cos(th)]

%% Energy calculations

% Compute the position of CoM of link i with the respect of the frame i
pL1_1 = [-l1/2; 0; 0];
pL2_2 = [0; -l2/2; 0];
pL3_3 = [0; 0; -l3/2]; 

% Calculate the partial transformations for energy calculations
% H0_1 = [cos(t1) -sin(t1) 0 l1*cos(t1);
%         sin(t1)  cos(t1) 0 l1*sin(t1);
%               0        0 1           0;
%               0        0 0           1];
% 
H0_1;

% H0_2 =[-sin(t1) 0 cos(t1) l1*cos(t1);
%         cos(t1) 0 sin(t1) l1*sin(t1);
%            0    1       0    l2 + d2;
%            0    0       0           1];
H0_2 = H0_1*H1_2;

% H0_3 = [ sin(t1)  0 cos(t1) cos(t1)*(l1 + l3 + d3);
%         -cos(t1)  0 sin(t1) sin(t1)*(l1 + l3 + d3);
%                0 -1       0                 l2 + d2;
%                0  0       0                        1];
H0_3 = H0_1*H1_2*H2_3;

pL(:,1) = H0_1(1:3,1:3)*pL1_1 + H0_1(1:3,4);
pL(:,2) = H0_2(1:3,1:3)*pL2_2 + H0_2(1:3,4);
pL(:,3) = H0_3(1:3,1:3)*pL3_3 + H0_3(1:3,4);

% Manually computed pL vectors (by column)
% [(l1*cos(t1))/2, l1*cos(t1) + (l2*sin(t1))/2, cos(t1)*(l1 + l3/2 + d3)]
% [(l1*sin(t1))/2, l1*sin(t1) - (l2*cos(t1))/2, sin(t1)*(l1 + l3/2 + d3)]
% [              0,                      l2 + t2,                   l2 + t2]

syms m1 real;
syms m2 real;
syms g real;
syms m3 real;

% Rotate from the base frame to zero
R = [1 0 0; 0 0 -1; 0 1 0];
% g0 was manually rotate using the rotation R
U1 = m1 * [0 -g 0] * pL(:,1);
U2 = m2 * [0 -g 0] * pL(:,2);
U3 = m3 * [0 -g 0] * pL(:,3);

U = U1 + U2 + U3;
U = simplify(U);

syms r1 real;
syms b2 real; % base of the prismatic joint to b2*b2
syms b3 real; % base of the prismatic joint to b3*b3

% Manually computed
% IL1_1 = m1*[1/2*r1^2 0 0;
%         0   1/2*(3*r1^4+l1^2)+(l1^2)/4 0;
%         0   0   1/2*(3*r1^4+l1^2)+(l1^2)/4];
%     
% IL2_2 =(1/12)*m2*[b2^2+l2^2 0 0;
%                 0 b2^2+4*l2^2 0;
%                 0 0 b2^2+b2^2];
% 
% IL3_3 =(1/12)*m3*[b3^2+l3^2 0 0;
%     0 b3^2+4*l3^2 0;
%     0 0 b3^2+b3^2];

% Cylinder Link 1
L1I = m1 * [1/2*r1^2          0                  0 ; 
            0    1/12*(3*r1^2 + l1^2)          0; 
          0             0         1/12*(3*r1^2 + l1^2)] + steiner(m1, pL1_1);

% Prismatic Link 2
L2I = m2 * [1/12*(b2^2 + l2^2)        0                0 ; 
                0         1/12*(b2^2 + l2^2)        0 ; 
                0                0         1/12*(b2^2 + b2^2)]  + steiner(m2, pL2_2);

% Prismatic Link 3
L3I = m3 * [1/12*(b3^2 + l3^2)        0                0 ; 
                0         1/12*(b3^2 + l3^2)        0 ; 
                0                0         1/12*(b3^2 + b3^2)] + steiner(m3, pL3_3);

            
IL1_1 = simplify(H0_1(1:3,1:3) * L1I * H0_1(1:3,1:3)');
IL2_2 = simplify(H0_2(1:3,1:3) * L2I * H0_2(1:3,1:3)');
IL3_3 = simplify(H0_3(1:3,1:3) * L3I * H0_3(1:3,1:3)');
 

% Let's compute the partial jacobians
% the manually computed partial jacobians are: 
% JPL1 = [-(l1*sin(t1))/2 0 0;
%  (l1*cos(t1))/2 0 0;
%                0 0 0];
% JPL2 = [(l2*cos(t1))/2 - l1*sin(t1) 0 0;
% l1*cos(t1) + (l2*sin(t1))/2 0 0;
%                             0 1 0];
% JPL3 = [-sin(t1)*(l1 + l3/2 + d3) 0 cos(t1);
%  cos(t1)*(l1 + l3/2 + d3) 0 sin(t1);
%                           0 1       0];

% Perform the calculations using the symbolic toolbox
JPL1_1 = cross(H0_1(1:3,3), pL(:,1)-H0_1(1:3,4));
JPL1_2 = [0;0;0];
JPL1_3 = [0;0;0];
JPL1 = [JPL1_1 JPL1_2 JPL1_3];

JPL2_1 = cross(H0_2(1:3,3), pL(:,2)-H0_2(1:3,4));
JPL2_2 = H0_1(1:3,3);
JPL2_3 = [0; 0; 0];
JPL2 = [JPL2_1 JPL2_2 JPL2_3];

JPL3_1 = cross(H0_3(1:3,3), pL(:,3)-H0_3(1:3,4));
JPL3_2 = H0_1(1:3,3);
JPL3_3 = H0_2(1:3,3);
JPL3 = [JPL3_1 JPL3_2 JPL3_3];

% They are pretty easy for an rpp
JOL1 = [ 0 0 0; 0 0 0; 1 0 0;];
JOL2 = [ 0 0 0; 0 0 0; 1 0 0;];
JOL3 = [ 0 0 0; 0 0 0; 1 0 0;];

% Compute the B contributions using the partial jacobian formulas
B1 = m1 * (JPL1' * JPL1) + ((H0_1(1:3,1:3)'*JOL1)' * IL1_1 * H0_1(1:3,1:3)'*JOL1);
B2 = m2 * (JPL2' * JPL2) + ((H0_2(1:3,1:3)'*JOL2)' * IL2_2 * H0_2(1:3,1:3)'*JOL2);
B3 = m3 * (JPL3' * JPL3) + ((H0_3(1:3,1:3)'*JOL3)' * IL3_3 * H0_3(1:3,1:3)'*JOL3);

B = B1 + B2 + B3;
B = simplify(B);

syms dt1 real;
syms dd2 real;
syms dd3 real;
dq = [dt1; dd2; dd3];
T = 1/2*dq'*B*dq;

T = simplify(T);

%% Evaluate some results 

% You can choose the values
t1 = pi/4;
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