function C = C_Lagrangian(q, d_q, param)
    t1 = q(1);
    d3 = q(3);

    d_t1 = d_q(1);
    d_d3 = d_q(3);

    l1 = param.l1;
    l2 = param.l2;
    l3 = param.l3;

    m2 = param.m2;
    m3 = param.m3;

    b2 = param.b2;
    b3 = param.b3;

    C = [[d_t1*((m2*sin(2*t1)*b2^2)/24 + (m3*sin(2*t1)*l3^2)/6 - (b3^2*m3*sin(2*t1))/24 - (l2^2*m2*sin(2*t1))/24) + d_d3*(d3*m3 + l1*m3 + (l3*m3)/2), 0, d_t1*(d3*m3 + l1*m3 + (l3*m3)/2)]
[                                                                                                                                         0, 0,                                0]
[                                                                                                         -d_t1*(d3*m3 + l1*m3 + (l3*m3)/2), 0,                                0]];
end

