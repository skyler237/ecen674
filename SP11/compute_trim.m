function [x_trim,u_trim] = compute_trim(filename, Va, gamma, R)
% Va is the desired airspeed (m/s)
% gamma is the desired flight path angle (radians)
% R is the desired radius (m) - use (+) for right handed orbit, 
%                                   (-) for left handed orbit


x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];

% % Convert to quaternions
% psi = x0(7);
% phi = x0(8);
% theta = x0(9);
% e0_0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
% e1_0 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
% e2_0 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
% e3_0 = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);
% x0 = [0; 0; Va; 0; 0; 0; e0_0; e1_0; e2_0; e3_0; 0; 0; 0];

ix = [];
u0 = [0; 0; 0; 0.5];
iu = [];
y0 = [Va; 0; 0];
iy = [1,3];
dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va*cos(gamma)/R; 0; 0; 0];

% % Convert to quaternions
% phidot = dx0(7);
% thetadot = dx0(8);
% psidot = dx0(9);
% e0dot = cos(psidot/2)*cos(thetadot/2)*cos(phidot/2) + sin(psidot/2)*sin(thetadot/2)*sin(phidot/2);
% e1dot = cos(psidot/2)*cos(thetadot/2)*sin(phidot/2) - sin(psidot/2)*sin(thetadot/2)*cos(phidot/2);
% e2dot = cos(psidot/2)*sin(thetadot/2)*cos(phidot/2) + sin(psidot/2)*cos(thetadot/2)*sin(phidot/2);
% e3dot = sin(psidot/2)*cos(thetadot/2)*cos(phidot/2) - cos(psidot/2)*sin(thetadot/2)*sin(phidot/2);
% dx0 = [0; 0; Va*sin(gamma); 0; 0; 0; e0dot; e1dot; e2dot; e3dot; 0; 0; 0];

% idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12; 13];
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% compute trim conditions
[x_trim,u_trim,y_trim,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);

% check to make sure that the linearization worked (should be small)
norm(dx_trim(3:end)-dx0(3:end))


% function [x_trim,u_trim] = compute_trim(filename, Va, gamma, R)
% % Va is the desired airspeed (m/s)
% % gamma is the desired flight path angle (radians)
% % R is the desired radius (m) - use (+) for right handed orbit, 
% %                                   (-) for left handed orbit
% 
% 
% % add stuff here
% x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
% u0 = [0; 0; 0; 1];
% y0 = [Va; 0; 0];
% ix = [];
% iu = [];
% iy = [1, 3];
% dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va*cos(gamma)/R; 0; 0; 0];
% idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];
% 
% % compute trim conditions
% [x_trim,u_trim,y_trim,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);
% 
% % check to make sure that the linearization worked (should be small)
% norm(dx_trim(3:end)-dx0(3:end))

