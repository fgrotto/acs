%% Calculation of the C(q,d_q) matrix

q = [t1,d2,d3];
syms t1_dot;
syms d2_dot;
syms d3_dot
q_dot = [t1_dot, d2_dot, d3_dot];
dof = 3;
C = zeros(3, 'sym');
for i=1:dof
    for j=1:dof
       for k = 1:dof
            C(i,j) = C(i,j)+ 1/2*(diff(B(i,j),q(k))+diff(B(i,k),q(j))-diff(B(j,k),q(i)))*q_dot(k);
       end
    end
end