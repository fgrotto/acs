%% Evaluate some results (B, U, T) and dynamic model of the manipulator (Lagrangian formulation)

% You can choose the values for positions, velocites and accelerations
t1 = pi/4;
d2 = 0;
d3 = 0;
d_t1 = 0;
d_d2 = 0;
d_d3 = 0;
dd_t1 = 0;
dd_d2 = 0;
dd_d3 = 0;

% Gravity
g = 9.81;
pi = 3.14;

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
f_e1 = 0; f_e2 = 0; f_e3 = 0;
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
eval(tau)

% Evaluate rne
tau_rne = [tau1; tau2; tau3];
eval(tau_rne)