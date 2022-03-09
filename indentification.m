%% Dynamic model identification
% To run it, first run main.m to load all the necessary parameters and 
% set the values from evaluate results, then run the model indentification

% Extract from equations of motion
y_m3 = dd_t1*(+ (b3^2)/8 + d3^2  + l1^2  + (2*l3^2)/3 +  + 2*d3*l1  + (b3^2*cos(2*t1))/24 - (2*l3^2*cos(2*t1))/3) + d_t1*(d_t1*(+ (2*sin(2*t1)*l3^2)/3 - (b3^2*sin(2*t1))/24) + d_d3*(d3 + l1)) + (g*cos(t1)*(2*d3 + 2*l1))/2 + d_d3*d_t1*(d3 + l1);
y_m1 = dd_t1*(6*r1^2 + l1^2) + l1*g*cos(t1);
y_m2 = dd_t1*((b2^2)/8+ l1^2 + (7*l2^2)/24 - (b2^2*cos(2*t1))/24) + (l2^2*cos(2*t1))/24 + d_t1*(d_t1*((sin(2*t1)*b2^2)/24 - (l2^2*sin(2*t1))/24))+ 2*l1 *g*cos(t1);

% Set values for the parameters pefore the computation of the dynamic model
t1 = pi/4;
d3 = 0.2;
d2 = 0.1;
d_t1 = 0.1;
d_d2 = 0.1;
d_d3 = 0.1;
dd_t1 = 0.1;
dd_d2 = 0.1;
dd_d3 = 0.1;

% Compute the parameters vectors 
theta1 = [m1; m1*(-l1/2); m1*0; m1*0; L1I(1,1); L1I(1,2); L1I(1,3); L1I(2,2); L1I(2,3); L1I(3,3)];
theta2 = [m2; m2*0; m2*(l2/2); m2*0; L2I(1,1); L2I(1,2); L2I(1,3); L2I(2,2); L2I(2,3); L2I(3,3)];
theta3 = [m3; m3*0; m3*0; m3*(-l3); L3I(1,1); L3I(1,2); L3I(1,3); L3I(2,2); L3I(2,3); L3I(3,3)];
param_pi = [theta1; theta2; theta3];

Y1 = [y_m1, zeros(1,9), y_m2, zeros(1,9), y_m3, zeros(1,9)];
Y2 = [zeros(1,10),dd_d2,zeros(1,9),dd_d2,zeros(1,9)];
Y3 = [zeros(1,20),dd_d3 - d3*d_t1^2 - d_t1^2*l1 + g*sin(t1), zeros(1,9)];
Y = [Y1;Y2;Y3];

lagrangian_tau = eval(B * [dd_t1; dd_d2; dd_d3] + C * [d_t1;d_d3;d_d3] + G)
linear_tau = eval(Y*param_pi)

% % Compute the kinetic energy
% T1 = 1/2*[d_t1; d_d2; d_d3]'*B1*[d_t1; d_d2; d_d3];
% T2 = 1/2*[d_t1; d_d2; d_d3]'*B2*[d_t1; d_d2; d_d3];
% T3 = 1/2*[d_t1; d_d2; d_d3]'*B3*[d_t1; d_d2; d_d3];
% 
% % Compute the a_T and a_U vectors to write the energies in linear fashion
% a_T(1,:) = T1*(1/(theta1'*theta1))*theta1';
% a_T(2,:) = T2*(1/(theta2'*theta2))*theta2';
% a_T(3,:) = T3*(1/(theta3'*theta3))*theta3';
% 
% a_U(1,:) = U1*(1/(theta1'*theta1))*theta1';
% a_U(2,:) = U2*(1/(theta2'*theta2))*theta2';
% a_U(3,:) = U3*(1/(theta3'*theta3))*theta3';
% 
% % Compute the matrix y(n*np) using the partial derivatives 
% y = zeros(dof,dof*10);
% for j = 1:dof
%    if j == 1 
%         y(1,1:j+9) = eval(diff(diff(a_T(j,:),d_q(1))) - diff(a_T(j,:),q(1)) + diff(a_U(j,:),q(1))); 
%    else 
%         y(1,10*j:10*j+9) = eval(diff(diff(a_T(j,:),d_q(1))) - diff(a_T(j,:),q(1)) + diff(a_U(j,:),q(1)));
%    end
% end
% 
% for j = 2:dof
%    y(2,10*j:10*j+9) = eval(diff(diff(a_T(j,:),d_q(2))) - diff(a_T(j,:),q(2)) + diff(a_U(j,:),q(2))); 
% end
% 
% for j = 3:dof
%    y(3,10*j:10*j+9) = eval(diff(diff(a_T(j,:),d_q(3))) - diff(a_T(j,:),q(3)) + diff(a_U(j,:),q(3))); 
% end
% 
% % Compute the estimated parameters solving LSE with Moore-Penrose pseudo-inverse
% theta_estimated = inv(y'*y)*y'*tau_required;