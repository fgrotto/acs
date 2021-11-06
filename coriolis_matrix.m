%% Calculation of the C(q,d_q) matrix
C = zeros(3, 'sym');
for i=1:dof
    for j=1:dof
       for k = 1:dof
            C(i,j) = C(i,j)+ 1/2*(diff(B(i,j),q(k))+diff(B(i,k),q(j))-diff(B(j,k),q(i)))*d_q(k);
       end
    end
end