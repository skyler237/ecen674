function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 13;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
psi = P.psi0;
phi = P.phi0;
theta = P.theta0;
e0_0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
e1_0 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
e2_0 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
e3_0 = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);

x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
%     P.phi0;...
%     P.theta0;...
%     P.psi0;...
    e0_0;...
    e1_0;...
    e2_0;...
    e3_0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, P)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    e0   = x(7);
    e1   = x(8);
    e2   = x(9);
    e3   = x(10);
    p    = x(11);
    q    = x(12);
    r    = x(13);
%     phi   = x(7);
%     theta = x(8);
%     psi   = x(9);
%     p     = x(10);
%     q     = x(11);
%     r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    lambda = 1000;
    e = [e0 e1 e2 e3];
    
    
    % TEST
%     e = e/norm(e);
  
%     pndot = (cos(theta)*cos(psi))*u + (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*v + (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*w;
%     pedot = (cos(theta)*sin(psi))*u + (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*v + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*w;
%     pddot = (-sin(theta))*u + (sin(phi)*cos(theta))*v + (cos(phi)*cos(theta))*w;
    pndot = (e0^2 + e1^2 - e2^2 - e3^2)*u + (2*(e1*e2 - e3*e0))*v + 2*(e1*e3 + e2*e0)*w;
    pedot = 2*(e1*e2 + e3*e0)*u + (e0^2 + e2^2 - e1^2 - e3^2)*v + 2*(e2*e3 - e1*e0)*w;
    pddot = 2*(e1*e3 - e2*e0)*u + 2*(e2*e3 + e1*e0)*v + (e0^2 +e3^2 - e1^2 -e2^2)*w;
    
    udot = r*v - q*w + (1/P.mass)*fx;
    vdot = p*w - r*u + (1/P.mass)*fy;
    wdot = q*u - p*v + (1/P.mass)*fz;
%     phidot = p + (sin(phi)*tan(theta))*q + (cos(phi)*tan(theta))*r;
%     thetadot = cos(phi)*q - sin(phi)*r;
%     psidot = (sin(phi)*sec(theta))*q + (cos(phi)*sec(theta))*r;
   e0dot = 1/2*(lambda*(1 - norm(e)^2)*e0 - p*e1 - q*e2 - r*e3);
   e1dot = 1/2*(p*e0 + lambda*(1-norm(e)^2)*e1 + r*e2 - q*e3);
   e2dot = 1/2*(q*e0 - r*e1 + lambda*(1 - norm(e)^2)*e2 + p*e3);
   e3dot = 1/2*(r*e0 + q*e1 - p*e2 + lambda*(1 - norm(e)^2)*e3);
   
   % TEST
   edot = [e0dot e1dot e2dot e3dot];
   edot = edot/norm(edot);
   
   e0dot = edot(1);
   e1dot = edot(2);
   e2dot = edot(3);
   e3dot = edot(4);

    pdot = P.gamma1*p*q - P.gamma2*q*r + P.gamma3*ell + P.gamma4*n;
    qdot = P.gamma5*p*r - P.gamma6*(p^2 - r^2) + (1/P.Jy)*m;
    rdot = P.gamma7*p*q - P.gamma1*q*r + P.gamma4*ell + P.gamma8*n;

% sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];
sys = [pndot; pedot; pddot; udot; vdot; wdot; e0dot; e1dot; e2dot; e3dot; pdot; qdot; rdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    e0   = x(7);
    e1   = x(8);
    e2   = x(9);
    e3   = x(10);
    p    = x(11);
    q    = x(12);
    r    = x(13);
    
    phi   = atan2(2*(e0*e1 + e2*e3),(e0^2 + e3^2 - e1^2 - e2^2));
    theta_term = 2*(e0*e2 - e1*e3);
    if(theta_term > 1)
        theta_term = 1;
    elseif(theta_term < -1)
        theta_term = -1;
    end
    theta = asin(theta_term);
    psi   = atan2(2*(e0*e3 + e1*e2), (e0^2 + e1^2 - e2^2 - e3^2));
xout = [pn; pe; pd; u; v; w; phi; theta; psi; p; q; r];
if(isreal(xout))
   sys = xout;
else
   display(xout)
end

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
