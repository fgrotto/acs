function B = B_Lagrangian(q, param)
    t1 = q(1);
    d3 = q(3);
    
    m1 = param.m1;
    m2 = param.m2;
    m3 = param.m3;
    b2 = param.b2;
    b3 = param.b3;
    r1 = param.r1;
    l1 = param.l1;
    l2 = param.l2;
    l3 = param.l3;

    B = [[(b2^2*m2)/8 + (b3^2*m3)/8 + d3^2*m3 + l1^2*m1 + l1^2*m2 + l1^2*m3 + (7*l2^2*m2)/24 + (5*l3^2*m3)/12 + 6*m1*r1^2 + 2*d3*l1*m3 + d3*l3*m3 + l1*l3*m3 - (b2^2*m2*cos(2*t1))/24 + (b3^2*m3*cos(2*t1))/24 + (l2^2*m2*cos(2*t1))/24 - (l3^2*m3*cos(2*t1))/6,       0,  0];
    [                                                                                                                                                                                                                                                    0, m2 + m3,  0];
    [                                                                                                                                                                                                                                                    0,       0, m3]];
end

