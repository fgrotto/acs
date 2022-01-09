function J = Jacobian(q, param)
    t1 = q(1);
    d3 = q(3);
    l3 = param.l3;

    J = [[-sin(t1)*(d3 + l3), cos(t1), cos(t1)];
        [ cos(t1)*(d3 + l3), sin(t1), sin(t1)];
        [                 0,       0,       0];
        [                 0,       0,       0];
        [                 0,       0,       0];
        [                 1,       0,       0]];
    
    J = double(J);
end

