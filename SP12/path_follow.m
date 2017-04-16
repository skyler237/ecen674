% path follow
%  - follow straight line path or orbit
%
% Modified:
%   3/25/2010  - RB
%   6/5/2010   - RB
%   11/08/2010 - RB
%   14/11/2014 - RWB
%
% input is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d   - desired airspeed
%   r      - inertial position of start of waypoint path
%   q      - unit vector that defines inertial direction of waypoint path
%   c      - center of orbit
%   rho    - radius of orbit
%   lambda - direction of orbit (+1 for CW, -1 for CCW)
%   xhat   - estimated MAV states (pn, pe, pd, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi)
%
% output is:
%  Va_c - airspeed command
%  h_c  - altitude command
%  chi_c - heading command
%  phi_ff - feed forward roll command
%
function out = path_follow(in,P)
  
  NN = 0;
  flag      = in(1+NN);
  Va_d      = in(2+NN);
  r_path    = [in(3+NN); in(4+NN); in(5+NN)];
  q_path    = [in(6+NN); in(7+NN); in(8+NN)];
  c_orbit   = [in(9+NN); in(10+NN); in(11+NN)];
  rho_orbit = in(12+NN);
  lam_orbit = in(13+NN);
  NN = NN + 13;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  Va        = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  phi       = in(7+NN);
  theta     = in(8+NN);
  chi       = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
   r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  NN = NN + 16;
  t         = in(1+NN);
  
  p = [pn; pe; -h];
  
  h_c = -P.pd0;
  chi_c = P.psi0;
  phi_ff = phi;
  
  switch flag,
      case 1, % follow straight line path specified by r and q
          r = r_path;
          
          chi_q = atan2(q_path(2),q_path(1));
          chi_err = (chi_q - chi);
          while(abs(chi_err) > pi)
             chi_q = chi_q - sign(chi_err)*2*pi;
             chi_err = chi_q - chi;
          end
          
          R_i_to_p = [ cos(chi_q), sin(chi_q), 0;...
                      -sin(chi_q), cos(chi_q), 0;...
                       0,          0,          1];
          e_p_i = p - r;
          e_p = R_i_to_p*(e_p_i);
                    
          chi_c = chi_q - P.chi_inf*(2/pi)*atan(P.k_path*e_p(2));
          
          k_i = [0; 0; -1];
          n = cross(q_path,k_i);
          if(norm(n) > 0)
            n = n/norm(n);    
          end
          s = e_p_i - dot(e_p_i,n)*n;
          h_c = -r(3) + sqrt(s(1)^2 + s(2)^2)*(q_path(3)/(sqrt(q_path(1)^2 + q_path(2)^2)));
          
          psidot_d = Va/500.0;
          phi_ff = atan(Va*psidot_d/P.gravity);
           
      case 2, % follow orbit specified by c, rho, lam
           
          varphi = atan2(p(2) - c_orbit(2), p(1) - c_orbit(1));
          err = varphi - chi;
          while(abs(err) > pi)
             varphi = varphi - sign(err)*2*pi; 
             err = varphi - chi;
          end
%           chi_0 = varphi + lam_orbit*pi/2;
          d = norm(p - c_orbit);
          
          chi_c = varphi + lam_orbit*(pi/2 + atan(P.k_orbit*(d - rho_orbit)/rho_orbit));
          h_c = -c_orbit(3);
          
          psidot_d = Va/rho_orbit;
          phi_ff = atan(Va*psidot_d/P.gravity);
  end
  
  % command airspeed equal to desired airspeed
  Va_c = Va_d;
  
  % create output
  out = [Va_c; h_c; chi_c; phi_ff];
end


