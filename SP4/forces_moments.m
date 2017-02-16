% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % Parameter values
    mass = P.mass;
    g = P.gravity;
    
    % compute wind data in NED
    V_wind_body = rotate([w_ns; w_es; w_ds], phi, theta, psi) + [u_wg; v_wg; w_wg];
    w_n = V_wind_body(1);
    w_e = V_wind_body(2);
    w_d = V_wind_body(3);
    
    % compute air data
    V_airspeed_body = [u; v; w] - V_wind_body;
    max_airspeed = 10000; % Just to create some cap on the max value
    V_airspeed_body(1) = saturate(V_airspeed_body(1), 0.000, max_airspeed);
    V_airspeed_body(2) = saturate(V_airspeed_body(2), 0.000, max_airspeed);
    V_airspeed_body(3) = saturate(V_airspeed_body(3), 0.000, max_airspeed);
    if(norm(V_airspeed_body) > max_airspeed)
       V_airspeed_body = V_airspeed_body/norm(V_airspeed_body)*max_airspeed; 
    end
    
    if(norm(V_airspeed_body) ~= 0)
        Va = norm(V_airspeed_body);
    else 
        Va = 0.0001;
    end
         
    
    
%     alpha = atan2(V_airspeed_body(3),V_airspeed_body(2))
    if (V_airspeed_body(1) == 0)
        if (V_airspeed_body(3) == 0)
           alpha = 0;
        else
           alpha = sign(V_airspeed_body(3))*pi;  
        end       
    else
        alpha = atan(V_airspeed_body(3)/V_airspeed_body(1));
    end
    
    beta = asin(V_airspeed_body(2)/Va);
    check_beta = beta

    % compute external forces and torques on aircraft
    F_grav = [ -mass*g*sin(theta);...
                mass*g*cos(theta)*sin(phi);...
                mass*g*cos(theta)*cos(phi)]
    
    % Aerodynamic definitions
    AR = P.b^2/P.S_wing;
    sigmoid = (1 + exp(-P.M*(alpha - P.alpha0)) + exp(P.M*(alpha + P.alpha0)))/...
              ((1 + exp(-P.M*(alpha - P.alpha0)))*(1 + exp(P.M*(alpha + P.alpha0))));
    C_D = P.C_D_p + (P.C_L_0 + P.C_L_alpha*alpha)^2/(pi*P.e*AR);
    C_L = (1 - sigmoid)*(P.C_L_0 + P.C_L_alpha*alpha) + sigmoid*(2*sign(alpha)*(sin(alpha))^2*cos(alpha));
    C_X = -C_D*cos(alpha) + C_L*(alpha);
    C_X_q = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha);
    C_X_delta_e = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
    C_Z = -C_D*sin(alpha) - C_L*cos(alpha);
    C_Z_q = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
    C_Z_delta_e = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);
    
    F_aero = 1/2*P.rho*Va^2*P.S_wing*...
        [C_X + C_X_q*(P.c/(2*Va))*q + C_X_delta_e*delta_e;...
        P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*(P.b/(2*Va))*p + P.C_Y_r*(P.b/(2*Va))*r + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r;...
        C_Z + C_Z_q*(P.c/(2*Va))*q + C_Z_delta_e*delta_e ]
    
    F_prop = 1/2*P.rho*P.S_prop*P.C_prop*[(P.k_motor*delta_t)^2 - Va^2; ...
                                            0;...
                                            0];
    
    Force = F_grav + F_aero + F_prop
    check_force = Force;
    
    T_aero = 1/2*P.rho*Va^2*P.S_wing*...
        [P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*(P.b/(2*Va))*p + P.C_ell_r*(P.b/(2*Va))*r + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);...
        P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*(P.c/(2*Va))*q + P.C_m_delta_e*delta_e);...
        P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*(P.b/(2*Va))*p + P.C_n_r*(P.b/(2*Va))*r + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r)];
    T_prop = [-P.k_T_P*(P.k_Omega*delta_t)^2;...
                0;...
                0];
    
    Torque = T_aero + T_prop
    
    Check_force_real = 0;
    Check_force_finite = sum(isinf(Force));
    Check_Torque_real = 0;
    Check_Torque_finite = sum(isinf(Torque));
    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];
end


%%%%%%%%%%%%%%%%%%%%%%%
function vec=rotate(vec,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose

  % rotate vector
  vec = R*vec;
  
end

function [sat_value] = saturate(value, min, max)
    if(value > max)
        sat_value = max;
    elseif (value < min)
        sat_value = min;
    else
        sat_value = value;
    end
end