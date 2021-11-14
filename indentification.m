%% Dynamic model identification
% To run it, first run main.m to load all the necessary parameters and 
% set the values from evaluate results, then run the model indentification

% @TODO fix the following problems
% 1) The time derivative requires to substitute t1, d2 and d3 with time
%       dependent function and being able to derive wrt of t
% 2) How can I calculate the a_T and a_U? Figure out what is the relation
%       with the rne algorithm, otherwise derive it my hand
%

% Set values for the parameters pefore the computation of the dynamic model
t1 = pi/2;
d3 = 0.2;
d2 = 0.1;
d_t1 = 0;
d_d2 = 0;
d_d3 = 0;
dd_t1 = 0;
dd_d2 = 0;
dd_d3 = 0;
tau_required = [0.2; 0.4; 0.1];

% Compute the parameters vectors 
theta1 = [m1; m1*(-l1/2); m1*0; m1*0; L1I(1,1); L1I(1,2); L1I(1,3); L1I(2,2); L1I(2,3); L1I(3,3)];
theta2 = [m2; m2*0; m2*(l2/2); m2*0; L2I(1,1); L2I(1,2); L2I(1,3); L2I(2,2); L2I(2,3); L2I(3,3)];
theta3 = [m3; m3*0; m3*0; m3*(-l3/2); L3I(1,1); L3I(1,2); L3I(1,3); L3I(2,2); L3I(2,3); L3I(3,3)];

% You can choose the values for positions, velocites and accelerations
t1 = 0; %-pi/2
d3 = 0;
d2 = 0;
d_t1 = 0; %0.1
d_d2 = 0;
d_d3 = 0;
dd_t1 = 0;
dd_d2 = 0;
dd_d3 = 0;

% Compute the kinetic energy
T1 = 1/2*[d_t1; d_d2; d_d3]'*B1*[d_t1; d_d2; d_d3];
T2 = 1/2*[d_t1; d_d2; d_d3]'*B2*[d_t1; d_d2; d_d3];
T3 = 1/2*[d_t1; d_d2; d_d3]'*B3*[d_t1; d_d2; d_d3];

% Compute the a_T and a_U vectors to write the energies in linear fashion
a_T(1,:) = T1*(1/(theta1'*theta1))*theta1';
a_T(2,:) = T2*(1/(theta2'*theta2))*theta2';
a_T(3,:) = T3*(1/(theta3'*theta3))*theta3';

a_U(1,:) = U1*(1/(theta1'*theta1))*theta1';
a_U(2,:) = U2*(1/(theta2'*theta2))*theta2';
a_U(3,:) = U3*(1/(theta3'*theta3))*theta3';

% Compute the matrix y(n*np) using the partial derivatives 
y = zeros(dof,dof*10);
for j = 1:dof
   if j == 1 
        y(1,1:j+9) = eval(diff(diff(a_T(j,:),d_q(1))) - diff(a_T(j,:),q(1)) + diff(a_U(j,:),q(1))); 
   else 
        y(1,10*j:10*j+9) = eval(diff(diff(a_T(j,:),d_q(1))) - diff(a_T(j,:),q(1)) + diff(a_U(j,:),q(1)));
   end
end

for j = 2:dof
   y(2,10*j:10*j+9) = eval(diff(diff(a_T(j,:),d_q(2))) - diff(a_T(j,:),q(2)) + diff(a_U(j,:),q(2))); 
end

for j = 3:dof
   y(3,10*j:10*j+9) = eval(diff(diff(a_T(j,:),d_q(3))) - diff(a_T(j,:),q(3)) + diff(a_U(j,:),q(3))); 
end

% Compute the estimated parameters solving LSE with Moore-Penrose pseudo-inverse
theta_estimated = inv(y'*y)*y'*tau_required;