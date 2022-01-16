function dot_J = Jacobian_dot(q,qd, param)
    t1 = q(1);
    d3 = q(3);
    l3 = param.l3;
    d_t1 = qd(1);
    d_d3 = qd(3);

    dot_J =[[- d_d3*sin(t1) - d_t1*cos(t1)*(d3 + l3), -d_t1*sin(t1), -d_t1*sin(t1)]
            [  d_d3*cos(t1) - d_t1*sin(t1)*(d3 + l3),  d_t1*cos(t1),  d_t1*cos(t1)]
            [                                      0,             0,             0]
            [                                      0,             0,             0]
            [                                      0,             0,             0]
            [                                      0,             0,             0]];
end


