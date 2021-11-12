%% Compute the partial jacobians
%
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
z0 = [0; 0; 1];
JPL1_1 = cross(z0, pL(:,1)-z0);
JPL1_2 = [0;0;0];
JPL1_3 = [0;0;0];
JPL1 = [JPL1_1 JPL1_2 JPL1_3];

JPL2_1 = cross(z0, pL(:,2)-z0);
JPL2_2 = H0_1(1:3,3);
JPL2_3 = [0; 0; 0];
JPL2 = [JPL2_1 JPL2_2 JPL2_3];

JPL3_1 = cross(z0, pL(:,3)-z0);
JPL3_2 = H0_1(1:3,3);
JPL3_3 = H0_2(1:3,3);
JPL3 = [JPL3_1 JPL3_2 JPL3_3];

% They are pretty easy for an rpp
JOL1 = [ 0 0 0; 0 0 0; 1 0 0;];
JOL2 = [ 0 0 0; 0 0 0; 1 0 0;];
JOL3 = [ 0 0 0; 0 0 0; 1 0 0;];
