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
    V_wind_body = rotate_i2b([w_ns; w_es; w_ds], phi, theta, psi) + [u_wg; v_wg; w_wg];
    V_wind_inertial = rotate_b2i([u_wg; v_wg; w_wg], phi, theta, psi) + [w_ns; w_es; w_ds];
    w_n = V_wind_inertial(1);
    w_e = V_wind_inertial(2);
    w_d = V_wind_inertial(3);
    
    % compute air data
    V_airspeed_body = [u; v; w] - V_wind_body;
    ur = V_airspeed_body(1);
    vr = V_airspeed_body(2);
    wr = V_airspeed_body(3);
%     max_airspeed = 10000; % Just to create some cap on the max value
%     V_airspeed_body(1) = saturate(V_airspeed_body(1), 0.000, max_airspeed);
%     V_airspeed_body(2) = saturate(V_airspeed_body(2), 0.000, max_airspeed);
%     V_airspeed_body(3) = saturate(V_airspeed_body(3), 0.000, max_airspeed);
%     if(norm(V_airspeed_body) > max_airspeed)
%        V_airspeed_body = V_airspeed_body/norm(V_airspeed_body)*max_airspeed; 
%     end
%     
%     if(norm(V_airspeed_body) ~= 0)
%         Va = norm(V_airspeed_body);
%     else 
%         Va = 0.0001;
%     end
%          
%     
%     
% %     alpha = atan2(V_airspeed_body(3),V_airspeed_body(2))
%     if (V_airspeed_body(1) == 0)
%         if (V_airspeed_body(3) == 0)
%            alpha = 0;
%         else
%            alpha = sign(V_airspeed_body(3))*pi;  
%         end       
%     else
%         alpha = atan(V_airspeed_body(3)/V_airspeed_body(1));
%     end
%     
%     beta = asin(V_airspeed_body(2)/Va);
%     check_beta = beta;

    Va = sqrt(ur^2 + vr^2 + wr^2);
%     if(Va == 0)
% %         Va = 0.01;
%         ur = 1e-10;
%         vr = 1e-10;
%         wr = 1e-10;
%         Va = sqrt(ur^2 + vr^2 + wr^2);
%     end
    alpha = atan2(wr, ur);
    beta = atan2(vr, sqrt(ur^2 + wr^2));

    % compute external forces and torques on aircraft
    F_grav = [ -mass*g*sin(theta);...
                mass*g*cos(theta)*sin(phi);...
                mass*g*cos(theta)*cos(phi)];
    
    % Aerodynamic definitions    
    sa = sin(alpha);
    ca = cos(alpha);
    AR = P.b^2/P.S_wing;
    sigmoid = (1 + exp(-P.M*(alpha - P.alpha0)) + exp(P.M*(alpha + P.alpha0)))/...
              ((1 + exp(-P.M*(alpha - P.alpha0)))*(1 + exp(P.M*(alpha + P.alpha0))));
    C_D = P.C_D_0 + (P.C_L_0 + P.C_L_alpha*alpha)^2/(pi*P.e*AR);
%     C_L = (1 - sigmoid)*(P.C_L_0 + P.C_L_alpha*alpha) + sigmoid*(2*sign(alpha)*(sa)^2*ca);
    C_L = (1 - sigmoid)*(P.C_L_0 + P.C_L_alpha*alpha);
    if alpha>=0, 
        C_L = C_L + sigmoid*2*sa*sa*ca;
    else
        C_L = C_L - sigmoid*2*sa*sa*ca;
    end
    C_X = -C_D*ca + C_L*sa;
    C_X_q = -P.C_D_q*ca + P.C_L_q*sa;
    C_X_delta_e = -P.C_D_delta_e*ca + P.C_L_delta_e*sa;
    C_Z = -C_D*sa - C_L*ca;
    C_Z_q = -P.C_D_q*sa - P.C_L_q*ca;
    C_Z_delta_e = -P.C_D_delta_e*sa - P.C_L_delta_e*ca;
    
    if (Va ~= 0)
        F_aero = 1/2*P.rho*Va^2*P.S_wing*...
            [C_X + C_X_q*(P.c/(2*Va))*q + C_X_delta_e*delta_e;...
            P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*(P.b/(2*Va))*p + P.C_Y_r*(P.b/(2*Va))*r + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r;...
            C_Z + C_Z_q*(P.c/(2*Va))*q + C_Z_delta_e*delta_e ];
    else
        F_aero = 0;
    end

        F_prop = 1/2*P.rho*P.S_prop*P.C_prop*[(P.k_motor*delta_t+Va)^2 - Va^2; ...
                                                0;...
                                                0];
    
                                            

    Force = F_grav + F_aero + F_prop;
    my_force = Force;

%     qbar = 0.5*P.rho*Va^2;
%     CL = C_L;
%     CD = C_D;
% 
%     % Beard implementation
%     Force = F_grav;
%     Force(1) = Force(1) + qbar*P.S_wing*(-CD*ca + CL*sa);
%     Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_q*ca + P.C_L_q*sa)*P.c*q/(2*Va);
% 
%     Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta);
%     Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_p*p + P.C_Y_r*r)*P.b/(2*Va);
% 
%     Force(3) = Force(3) + qbar*P.S_wing*(-CD*sa - CL*ca);
%     Force(3) = Force(3) + qbar*P.S_wing*(-P.C_D_q*sa - P.C_L_q*ca)*P.c*q/(2*Va);
%     Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_delta_e*ca+P.C_L_delta_e*sa)*delta_e;
%     Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
%     Force(3) = Force(3) + qbar*P.S_wing*(-P.C_D_delta_e*sa-P.C_L_delta_e*ca)*delta_e;
%     motor_temp = (P.k_motor*delta_t+Va)^2-Va^2; % revised model from book
%     Force(1) = Force(1) + 0.5*P.rho*P.S_prop*P.C_prop*motor_temp;
%     beard_force = Force;
    
    if(Va ~= 0)
        T_aero = 1/2*P.rho*Va^2*P.S_wing*...
            [P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*(P.b/(2*Va))*p + P.C_ell_r*(P.b/(2*Va))*r + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);...
            P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*(P.c/(2*Va))*q + P.C_m_delta_e*delta_e);...
            P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*(P.b/(2*Va))*p + P.C_n_r*(P.b/(2*Va))*r + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r)];
    else
        T_aero = 0; 
    end
    
    T_prop = [-P.k_T_P*(P.k_Omega*delta_t)^2;...
                0;...
                0];
    
    Torque = T_aero + T_prop;
    my_torque = Torque;
    
%     % Beard implementation
%     Torque(1) = qbar*P.S_wing*P.b*(P.C_ell_0 + P.C_ell_beta*beta);
%     Torque(1) = Torque(1) + qbar*P.S_wing*P.b*(P.C_ell_p*p + P.C_ell_r*r)*P.b/(2*Va);
% 
%     Torque(2) = qbar*P.S_wing*P.c*(P.C_m_0 + P.C_m_alpha*alpha);
%     Torque(2) = Torque(2) + qbar*P.S_wing*P.c*P.C_m_q*P.c*q/(2*Va);
% 
%     Torque(3) = qbar*P.S_wing*P.b*(P.C_n_0 + P.C_n_beta*beta);
%     Torque(3) = Torque(3) + qbar*P.S_wing*P.b*(P.C_n_p*p + P.C_n_r*r)*P.b/(2*Va);
%     
%     Torque(1) = Torque(1) + qbar*P.S_wing*P.b*(P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);
%     Torque(2) = Torque(2) + qbar*P.S_wing*P.c*P.C_m_delta_e*delta_e;
%     Torque(3) = Torque(3) + qbar*P.S_wing*P.b*(P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);
%     beard_torque = Torque;
    
%     diff_force = beard_force - my_force;
%     diff_torque = beard_torque - my_torque;
%     check_difference = diff_torque;
    
    Check_force_real = 0;
    Check_force_finite = sum(isinf(Force));
    Check_Torque_real = 0;
    Check_Torque_finite = sum(isinf(Torque));
    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];
end


%%%%%%%%%%%%%%%%%%%%%%%
function vec=rotate_i2b(vec,phi,theta,psi)

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

function vec=rotate_b2i(vec,phi,theta,psi)

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
  vec = R'*vec;
  
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
