function C = C_Lagrangian(q, d_q, param)
    d3 = q(3);

    d_t1 = d_q(1);
    d_d3 = d_q(3);

    l1 = param.l1;
    m3 = param.m3;

    C = [[ d_d3*(d3*m3 + l1*m3), 0, d_t1*(d3*m3 + l1*m3)]
        [                    0, 0,                    0]
        [-d_t1*(d3*m3 + l1*m3), 0,                    0]];
end

