%% Forward Newton-Euler formulation procedure for RPP robot
z0 = [0 0 1]';
w0_0 = [0 0 0]';
d_w0_0 = [0 0 0]';
dd_p0_0 = [0 0 0]';

% Distances links 
% @TODO check the sign here
r1_01 = [-l1; 0; 0];
r2_12 = [0; -l2; 0];
r3_23 = [0; 0; -l3];

% Distances CoM and links
r1_1c1 = [-l1/2; 0; 0];
r2_2c2 = [0; -l2/2; 0];
r3_3c3 = [0; 0; -l3/2];

% First rotative joint
R0_1 = H0_1(1:3,1:3);
w1_1 = (R0_1)*w0_0+R0_1'*d_q(1)*z0;
d_w1_1 = (R0_1)*d_w0_0+R0_1'*(dd_q(1)*z0+cross(d_q(1)*w0_0,z0));
dd_p1_1 = R0_1'*dd_p0_0+cross(d_w1_1,r1_01)+cross(w1_1,cross(w1_1,r1_01));
dd_p1_c1 = dd_p1_1 + cross(d_w1_1,r1_1c1) + cross(w1_1,cross(w1_1,r1_1c1));
d_w0_m1 = d_w0_0;


% Second prismatic joint
R1_2 = H1_2(1:3,1:3);
w2_2 = (R1_2)*w1_1;
d_w2_2 = (R1_2)*d_w1_1;
dd_p2_2 = R1_2'*dd_p1_1+cross(d_w2_2,r2_12)+cross(w2_2,cross(w2_2,r2_12))+R1_2'*dd_q(2)*z0+cross(2*d_q(2)*w2_2,R1_2'*z0);
dd_p2_c2 = dd_p2_2 + cross(d_w2_2,r2_2c2) + cross(w2_2,cross(w2_2,r2_2c2));
d_w1_m2 = d_w1_1;


% Third prismatic joint
R2_3 = H2_3(1:3,1:3);
w3_3 = (R2_3)*w2_2;
d_w3_3 = (R2_3)*d_w2_2;
dd_p3_3 = R2_3'*dd_p2_2+cross(d_w3_3,r3_23)+cross(w3_3,cross(w3_3,r3_23))+R2_3'*dd_q(3)*z0+cross(2*d_q(3)*w3_3,R2_3'*z0);
dd_p3_c3 = dd_p3_3 + cross(d_w3_3,r3_3c3) + cross(w3_3,cross(w3_3,r3_3c3));
d_w2_m3 = d_w2_2;


%% Backward Newton Euler Formulation 
he = [f_1; f_2; f_3; f_e; mu_1; mu_2; mu_3; mu_e];

% First rotative joint
f_1 = R1_2*f_2+m1*dd_p1_c1;
mu_1 = -cross(f_1, (r1_01+r1_1c1))+R1_2*mu_2+R1_2*cross(f_2,r1_1c1)+L1I*d_w1_1+cross(w1_1,L1I*w1_1);
tau1 = mu_1'*R0_1'*z0+Fvi(1)*d_q(1)+Fsi(1)*sign(d_q(1));

% Second prismatic joint
f_2 = R2_3*f_3+m2*dd_p2_c2;
mu_2 = -cross(f_2, (r2_12+r2_2c2))+R2_3*mu_3+R2_3*cross(f_3,r2_2c2)+L2I*d_w2_2+cross(w2_2,L2I*w2_2);
tau2 = f_2'*R1_2'*z0+Fvi(1)*d_q(2)+Fsi(1)*sign(d_q(2));

% Third prismatic joint
R3_4 = H3_e(1:3,1:3);
f_3 = R3_4*f_e+m3*dd_p3_c3;
mu_3 = -cross(f_3, (r3_23+r3_3c3))+R3_4*mu_e+R3_4*cross(f_e,r3_3c3)+L3I*d_w3_3+cross(w3_3,L3I*w3_3);
tau3 = f_e'*R2_3'*z0+Fvi(1)*d_q(3)+Fsi(1)*sign(d_q(3));








