function [sys,x0,str,ts] = Manipulator(t,x,u,flag,param)

switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes();
  case 1
    sys=mdlDerivatives(t,x,u,param);
  case 3
    sys=mdlOutputs(t,x,u);
  case { 2, 4, 9 }
    sys = [];
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes()
    sizes = simsizes;
    sizes.NumContStates  = 6;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 6;
    sizes.NumInputs      = 9;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);
    x0  = zeros(6,1);
    str = [];
    ts  = [0 0];


function sys=mdlDerivatives(t, x, u,param)
    q = x(1:3);
    d_q = x(4:6);
    tau = u(1:3);
    he = u(4:9);

    B = double(B_Lagrangian(q, param));
    C = double(C_Lagrangian(q, d_q, param));
    G = double(G_Lagrangian(q, param));
    J = double(Jacobian(q, param));

    dd_q = inv(B)*(tau - C*d_q - G - J'*he);

    sys = [d_q; dd_q];

function sys=mdlOutputs(t, x, u)
    sys = x;
