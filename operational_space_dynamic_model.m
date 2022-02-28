% Operational space dynamic model
syms ph;
syms th;
Ta = [1 0, 0, 0, 0, 0;
    0 1, 0 0 0 0;
    0 0, 1 0 0 0;
    0 0, 0 0 -sin(ph) cos(ph) * sin(th);
    0 0, 0 0 cos(ph) sin(ph) * sin(th);
    0 0, 0 1 0 cos(th)];

% This should be identical to the one calculated by hand Ja
% Ja_transformed = Ta*J; Ja0_e and Ja
Ja_op = simplify(Ja0_e);

% Derive the dot_Ja
t = sym('t', 'real');
q_time = [symfun('t1(t)', t), symfun('d2(t)', t), symfun('d3(t)', t)];
dq_time = diff(q_time);
ddq_time = diff(dq_time);


Ja_time = subs(Ja_op, q, q_time);
dot_Ja_time = diff(Ja_time, t);
dot_Ja0_e = simplify(subs(dot_Ja_time, [dq_time,q_time], [d_q,q]));

syms f_e1 real;
syms f_e2 real;
syms f_e3 real;
syms mu_e1 real;
syms mu_e2 real;
syms mu_e3 real;
he = [f_e1; f_e2; f_e3; mu_e1; mu_e2; mu_e3];

% Calculate the dynamic model in operational space for the non-redundant
% manipulator in a nonsigular configuration

invJa = pinv(Ja_op);
Ba = simplify(invJa' * B * invJa);
Ca_dx = simplify(invJa' * C * d_q' - Ba * dot_Ja0_e * d_q');
ga = simplify(invJa' * [0; -g; 0]);
ue = Ta' * he;
% u = Ta'*h;