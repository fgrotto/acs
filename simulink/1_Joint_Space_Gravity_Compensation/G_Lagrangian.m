function G = G_Lagrangian(q, param)
    t1 = q(1);
    d3 = q(3);

    g = param.g;
    l1 = param.l1;
    l3 = param.l3;
    m1 = param.m1;
    m2 = param.m2;
    m3 = param.m3;


    G = [ (g*cos(t1)*(2*d3*m3 + l1*m1 + 2*l1*m2 + 2*l1*m3 + l3*m3))/2;
                                                              0;
                                                   g*m3*sin(t1)];
end

