%% Evaluate some results (B, U, T) and dynamic model of the manipulator (Lagrangian formulation)

% You can choose the values for positions, velocites and accelerations
t1 = pi;
d3 = -0.2;
d2 = -0.2;
d_t1 = 2;
d_d2 = 2;
d_d3 = 2;
dd_t1 = 0;
dd_d2 = 0.1;
dd_d3 = 0.2;

% Gravity
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

% Forces and torques
f_11 = 0; f_12 = 0; f_13 = 0;
f_21 = 0; f_22 = 0; f_23 = 0;
f_31 = 0; f_32 = 0; f_33 = 0;
f_e1 = 0; f_e2 = 1; f_e3 = 1;
mu_11 = 0; mu_12 = 0; mu_13 = 0;
mu_21 = 0; mu_22 = 0; mu_23 = 0;
mu_31 = 0; mu_32 = 0; mu_33 = 0;
mu_e1 = 0; mu_e2 = 0; mu_e3 = 0;
Fvi1 = 0; Fvi2 = 0; Fvi3 = 0;
Fsi1 = 0; Fsi2 = 0; Fsi3 = 0; 

% Evaluate B to check if it is positive definite
eval(B);
eval(U);
eval(T);
eval(C);

% Write down the symbolic equation of the robot
tau = B * [dd_t1; dd_d2; dd_d3] + C * [d_t1; d_d2; d_d3] + G;
tau = simplify(tau);
eval(tau);

% Evaluate rne 
eval(tau1)
eval(tau2)
eval(tau3)