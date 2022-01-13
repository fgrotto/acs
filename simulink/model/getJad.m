function [Jad] = getJad(q, x_tilde, xd, param) 
    xe = -(x_tilde - xd);
    % Rotation matrix obtained from xd (fixed) in regulation problem
    Rd = eul2rotm(xd(4:6)','ZYZ')*eul2rotm(xe(4:6)','ZYZ');
    
    % Compute Ta(phi_de) in this case ZYZ
    ph = -x_tilde(4);
    th = -x_tilde(5);
    Ta =   [1, 0, 0, 0, 0, 0;
            0, 1, 0, 0, 0, 0;
            0, 0, 1, 0, 0, 0;
            0, 0, 0, 0, -sin(ph), cos(ph) * sin(th);
            0, 0, 0, 0, cos(ph), sin(ph) * sin(th);
            0, 0, 0, 1, 0, cos(th)];
    
    Jad = pinv(Ta)*[Rd', zeros(3,3); zeros(3,3), Rd']*Jacobian(q, param);
end