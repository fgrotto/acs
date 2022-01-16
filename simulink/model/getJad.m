function [Jad] = getJad(q, x_tilde, xd, xe, param) 
    % Rotation matrix obtained from xd (fixed) in regulation problem
    Rd = eul2rotm(-x_tilde(4:6)','ZYZ');
    angles = -x_tilde(4:6);
    
    
%     Rd = eul2rotm(xd(4:6)','ZYZ');
%     Re = eul2rotm(xe(4:6)','ZYZ');
%     
%     Rd_e = Rd'*Re;
%     angles = rotm2eul(Rd_e, 'ZYZ');
    
    % Compute Ta(phi_de) in this case ZYZ
    ph = -angles(1);
    th = -angles(2);
    Ta =   [1, 0, 0, 0, 0, 0;
            0, 1, 0, 0, 0, 0;
            0, 0, 1, 0, 0, 0;
            0, 0, 0, 0, -sin(ph), cos(ph) * sin(th);
            0, 0, 0, 0, cos(ph), sin(ph) * sin(th);
            0, 0, 0, 1, 0, cos(th)];
        
    Jad = pinv(Ta)*[Rd', zeros(3,3); zeros(3,3), Rd']*Jacobian(q, param);
end