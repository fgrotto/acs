%% Forward Newton-Euler formulation procedure for RPP robot
z0 = [0 0 1]';
w0_0 = [0 0 0]';
d_w0_0 = [0 0 0]';
dd_p0_0 = [0 g 0]';

R0_1 = H0_1(1:3,1:3);
R1_2 = H1_2(1:3,1:3);
R2_3 = H2_3(1:3,1:3);
R3_e = H3_e(1:3,1:3);

% Distances links 
r1_01 = R0_1'*H0_1(1:3,4);
r2_12 = R1_2'*H1_2(1:3,4);
r3_23 = R2_3'*H2_3(1:3,4);

% Distances CoM and links
r1_1c1 = [-l1/2; 0; 0];
r2_2c2 = [0; l2/2; 0];
r3_3c3 = [0; 0; -l3]; % renaming

% First rotative joint
w1_1 = (R0_1)'*w0_0+R0_1'*d_q(1)*z0;
d_w1_1 = (R0_1)'*d_w0_0+R0_1'*(dd_q(1)*z0+cross(d_q(1)*w0_0,z0));
dd_p1_1 = R0_1'*dd_p0_0+cross(d_w1_1,r1_01)+cross(w1_1,cross(w1_1,r1_01));
dd_p1_c1 = dd_p1_1 + cross(d_w1_1,r1_1c1) + cross(w1_1,cross(w1_1,r1_1c1));
d_w0_m1 = d_w0_0;


% Second prismatic joint
w2_2 = (R1_2)'*w1_1;
d_w2_2 = (R1_2)'*d_w1_1;
dd_p2_2 = R1_2'*dd_p1_1+cross(d_w2_2,r2_12)+cross(w2_2,cross(w2_2,r2_12))+R1_2'*dd_q(2)*z0+cross(2*d_q(2)*w2_2,R1_2'*z0);
dd_p2_c2 = dd_p2_2 + cross(d_w2_2,r2_2c2) + cross(w2_2,cross(w2_2,r2_2c2));
d_w1_m2 = d_w1_1;


% Third prismatic joint
w3_3 = (R2_3)'*w2_2;
d_w3_3 = (R2_3)'*d_w2_2;
dd_p3_3 = R2_3'*dd_p2_2+cross(d_w3_3,r3_23)+cross(w3_3,cross(w3_3,r3_23))+R2_3'*dd_q(3)*z0+cross(2*d_q(3)*w3_3,R2_3'*z0);
dd_p3_c3 = dd_p3_3 + cross(d_w3_3,r3_3c3) + cross(w3_3,cross(w3_3,r3_3c3));
d_w2_m3 = d_w2_2;


%% Backward Newton Euler Formulation 
he = [f_e; mu_e];

% Third prismatic joint
f_3 = R3_e*f_e+m3*dd_p3_c3;
mu_3 = -cross(f_3, (r3_23+r3_3c3))+R3_e*mu_e+cross(R3_e*f_e,r3_3c3)+IL3_3*d_w3_3+cross(w3_3,IL3_3*w3_3);
tau3 = f_3'*R2_3'*z0+Fvi(3)*d_q(3)+Fsi(3)*sign(d_q(3));

% Second prismatic joint
f_2 = R2_3*f_3+m2*dd_p2_c2;
mu_2 = -cross(f_2, (r2_12+r2_2c2))+R2_3*mu_3+cross(R2_3*f_3,r2_2c2)+IL2_2*d_w2_2+cross(w2_2,IL2_2*w2_2);
tau2 = f_2'*R1_2'*z0+Fvi(2)*d_q(2)+Fsi(2)*sign(d_q(2));

% First rotative joint
f_1 = R1_2*f_2+m1*dd_p1_c1;
mu_1 = -cross(f_1, (r1_01+r1_1c1))+R1_2*mu_2+cross(R1_2*f_2,r1_1c1)+IL1_1*d_w1_1+cross(w1_1,IL1_1*w1_1);
tau1 = mu_1'*R0_1'*z0+Fvi(1)*d_q(1)+Fsi(1)*sign(d_q(1));