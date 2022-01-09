function Jad = getJad(q, x_tilde, xd, param)
    l3 = param.l3;
    t1 = q(1);
    d3 = q(3);
    
    % Rotation matrix obtained from xd (fixed) in regulation problem
    Rd = eul2rotm(xd(4:6)','ZYZ');
    
    % Compute Ta(phi_de) in this case ZYZ
    ph = -x_tilde(4);
    th = -x_tilde(5);
    Ta =   [1, 0, 0, 0, 0, 0;
            0, 1, 0, 0, 0, 0;
            0, 0, 1, 0, 0, 0;
            0, 0, 0, 0, -sin(ph), cos(ph) * sin(th);
            0, 0, 0, 0, cos(ph), sin(ph) * sin(th);
            0, 0, 0, 1, 0, cos(th)];
        
%     Ta = [ones(3,3), zeros(3,3); zeros(3,3), eul2rotm(-x_tilde(4:6)', 'ZYZ')];
    
    % Geometric jacobian wrt of frame 0
    J = [-sin(t1)*(d3 + l3), cos(t1), cos(t1);
         cos(t1)*(d3 + l3), sin(t1), sin(t1);
                         0,       0,       0;
                         0,       0,       0;
                         0,       0,       0;
                         1,       0,       0];
    
    Jad = pinv(Ta)*[Rd', zeros(3,3); zeros(3,3), Rd']*J;
end