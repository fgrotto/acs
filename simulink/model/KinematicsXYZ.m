function x = KinematicsXYZ(q,param)
    l1 = param.l1;
    l2 = param.l2;
    l3 = param.l3;

    t1 = q(1);
    d2 = q(2);
    d3 = q(3);
    
    % H0_3 computed with the main.m
    H = [[ 0, -sin(t1), cos(t1), cos(t1)*(d3 + l3) + l1*cos(t1)]
        [ 0,  cos(t1), sin(t1), sin(t1)*(d3 + l3) + l1*sin(t1)]
        [-1,        0,       0,                        d2 + l2]
        [ 0,        0,       0,                              1]];

    
    % Return x with positions and orientations
    x = [ H(1:3,4); rotm2eul(H(1:3,1:3), 'XYZ')'];
end