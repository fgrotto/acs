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
config(1).JointPosition = 0;
config(2).JointPosition = 0;
config(3).JointPosition = 0;

show(robot,config);
xlim([-0.5 0.8])
ylim([-0.5 0.5])
zlim([0 0.8])

%% Calculate direct kinematics

dk = getTransform(robot, config, 'ee', 'base_link')

t1 = pi/4;
d3 = -0.2;
d2 = -0.2;
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
% d3_calc1 = -L1h - sqrt(L1h^2+(x^2+(z-L0h)^2-L1h^2)) - L3h
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

% Jacobian wrt of the 0 frame
% J0
% Ja0           
% Ta 
J0 = [-sin(t1)*(L1h + L3h + d3) 0 cos(t1);
 cos(t1)*(L1h + L3h + d3) 0 sin(t1);
                        0 1       0;
                        0 0       0;
                        0 0       0;
                        1 0       0]

Ja0 = [-sin(t1)*(L1h + L3h + d3) 0 cos(t1);
 cos(t1)*(L1h + L3h + d3) 0 sin(t1);
                        0 1       0;
                        1 0       0;
                        0 0       0;
                        0 0       0]

syms ph;
syms th;
Ta = [1 0, 0, 0,        0,               0;
0 1, 0 0        0               0;
0 0, 1 0        0               0;
0 0, 0 0 -sin(ph) cos(ph)*sin(th);
0 0, 0 0  cos(ph) sin(ph)*sin(th);
0 0, 0 1        0         cos(th)]

%% Energy calculations
syms t1 real;
syms d3 real;
syms d2 real;
syms L1h real;
syms L2h real;
syms L3h real;

pL1_1 = [-L1h/2; 0; 0];
pL2_2 = [0; -L2h/2; 0];
pL3_3 = [0; 0; -L3h/2]; 

H0_1 = [cos(t1) -sin(t1) 0 L1h*cos(t1);
        sin(t1)  cos(t1) 0 L1h*sin(t1);
              0        0 1           0;
              0        0 0           1];

H0_2 =[-sin(t1) 0 cos(t1) L1h*cos(t1);
        cos(t1) 0 sin(t1) L1h*sin(t1);
           0    1       0    L2h + d2;
           0    0       0           1];

H0_3 = [ sin(t1)  0 cos(t1) cos(t1)*(L1h + L3h + d3);
        -cos(t1)  0 sin(t1) sin(t1)*(L1h + L3h + d3);
               0 -1       0                 L2h + d2;
               0  0       0                        1];

pL(:,1) = H0_1(1:3,1:3)*pL1_1 + H0_1(1:3,4);
pL(:,2) = H0_2(1:3,1:3)*pL2_2 + H0_2(1:3,4);
pL(:,3) = H0_3(1:3,1:3)*pL3_3 + H0_3(1:3,4);

syms m1 real;
syms m2 real;
syms g real;
syms m3 real;

% Rotate from the base frame to zero
R = [1 0 0; 0 0 -1; 0 1 0];
U1 = m1 * [0 0 -g] * R*pL(:,1);
U2 = m2 * [0 0 -g] * R*pL(:,2);
U3 = m3 * [0 0 -g] * R*pL(:,3);

U = U1 + U2 + U3;
U = simplify(U);

syms r1 real;
syms L2 real; % base of the prismatic joint to L2*L2
syms L3 real; % base of the prismatic joint to L3*L3

       
L1I = m1 * [1/2*r1^2          0                  0 ; 
            0    1/12*(3*r1^2 + L1h^2)          0; 
          0             0         1/12*(3*r1^2 + L1h^2)] + steiner(m1, pL1_1);

L2I = m2 * [1/12*(L2^2 + L2h^2)        0                0 ; 
                0         1/12*(L2^2 + L2h^2)        0 ; 
                0                0         1/12*(L2^2 + L2^2)]  + steiner(m2, pL2_2);
L3I = m3 * [1/12*(L3^2 + L3h^2)        0                0 ; 
                0         1/12*(L3^2 + L3h^2)        0 ; 
                0                0         1/12*(L3^2 + L3^2)] + steiner(m3, pL3_3);

            
IL1_1 = simplify(H0_1(1:3,1:3) * L1I * H0_1(1:3,1:3)');
IL2_2 = simplify(H0_2(1:3,1:3) * L1I * H0_2(1:3,1:3)');
IL3_3 = simplify(H0_3(1:3,1:3) * L1I * H0_3(1:3,1:3)');

% Manually computed
% IL1_1 = m1*[1/2*r1^2 0 0;
%         0   1/2*(3*r1^4+L1h^2)+(L1h^2)/4 0;
%         0   0   1/2*(3*r1^4+L1h^2)+(L1h^2)/4];
%     
% IL2_2 =(1/12)*m2*[L2^2+L2h^2 0 0;
%                 0 L2^2+4*L2h^2 0;
%                 0 0 L2^2+L2^2];
% 
% IL3_3 =(1/12)*m3*[L3^2+L3h^2 0 0;
%     0 L3^2+4*L3h^2 0;
%     0 0 L3^2+L3^2];
 

JPL1 = [-(L1h*sin(t1))/2 0 0;
 (L1h*cos(t1))/2 0 0;
               0 0 0];
JPL2 = [(L2h*cos(t1))/2 - L1h*sin(t1) 0 0;
L1h*cos(t1) + (L2h*sin(t1))/2 0 0;
                            0 1 0];
JPL3 = [-sin(t1)*(L1h + L3h/2 + d3) 0 cos(t1);
 cos(t1)*(L1h + L3h/2 + d3) 0 sin(t1);
                          0 1       0];
                      
JOL1 = [ 0 0 0; 0 0 0; 1 0 0;];
JOL2 = [ 0 0 0; 0 0 0; 1 0 0;];
JOL3 = [ 0 0 0; 0 0 0; 1 0 0;];

B1 = m1 * (JPL1' * JPL1) + (H0_1(1:3,1:3)*JOL1' * IL1_1 * H0_1(1:3,1:3)*JOL1);
B2 = m2 * (JPL2' * JPL2) + (H0_2(1:3,1:3)*JOL2' * IL2_2 * H0_2(1:3,1:3)*JOL2);
B3 = m3 * (JPL3' * JPL1) + (H0_3(1:3,1:3)*JOL3' * IL3_3 * H0_3(1:3,1:3)*JOL3);

B = B1 + B2 + B3;
B = simplify(B);

syms dt1 real;
syms dd2 real;
syms dd3 real;
dq = [dt1; dd2; dd3];
T = 1/2*dq'*B*dq;

T = simplify(T);