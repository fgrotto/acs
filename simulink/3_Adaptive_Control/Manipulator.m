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
    dof = 1;
    sizes = simsizes;
    sizes.NumContStates  = 2*dof;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 2*dof;
    sizes.NumInputs      = dof; %3*dof
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);
    x0  = zeros(2,1);
    str = [];
    ts  = [0 0];


function sys=mdlDerivatives(t, x, u,param)
    q = x(1);
    d_q = x(2);
    tau = u(1);

    F = param.F;
    G = param.G;
    I = param.I;
    
    dd_q = inv(I)*(tau - F*d_q - G*sin(q));
    
    sys = [d_q; dd_q];

function sys=mdlOutputs(t, x, u)
    sys = x;
