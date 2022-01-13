function Ja = AnalyticalJacobian(q, param) 
    l1 = param.l1;
    l3 = param.l3;

    t1 = q(1);
    d3 = q(3);
    
    % Compute Jacobian wrt of frame 0
    Ja=[[-sin(t1)*(d3 + l1 + l3), 0, cos(t1)]
        [ cos(t1)*(d3 + l1 + l3), 0, sin(t1)]
        [                      0, 1,       0]
        [                      1, 0,       0]
        [                      0, 0,       0]
        [                      0, 0,       0]];
    
    Ja = double(Ja);
end