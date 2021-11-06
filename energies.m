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


%% Potential Energy
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

%% Kinetic Energy
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

partial_jacobians;
inertia_matrix;

syms dt1 real;
syms dd2 real;
syms dd3 real;
syms ddt1 real;
syms ddd2 real;
syms ddd3 real;
dq = [dt1; dd2; dd3];
ddq = [ddt1; ddd2; ddd3];
T = 1/2*dq'*B*dq;

T = simplify(T);