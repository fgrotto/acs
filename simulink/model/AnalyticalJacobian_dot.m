function d_Ja = AnalyticalJacobian_dot(q,qd,param)    
    l1 = param.l1;
    l3 = param.l3;

    t1 = q(1);
    d3 = q(3);
    d_t1 = qd(1);
    d_d3 = qd(3);
    
    % Use the time derivative of the analytical jacobian computed in the
    % previous sections and substitute the values
    d_Ja=[[- d_d3*sin(t1) - d_t1*cos(t1)*(d3 + l1 + l3), 0, -d_t1*sin(t1)]
        [  d_d3*cos(t1) - d_t1*sin(t1)*(d3 + l1 + l3), 0,  d_t1*cos(t1)]
        [                                           0, 0,             0]
        [                                           0, 0,             0]
        [                                           0, 0,             0]
        [                                           0, 0,             0]];
    
    d_Ja = double(d_Ja);
end