function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   9/30/2014 - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    autopilot_version = 2;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 3,
           [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_tuning
%   - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    mode = 5;
    switch mode
        case 1, % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            delta_a = roll_hold(phi_c, phi, p, t, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2, % tune the course loop
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, t, P);
            else
                phi_c   = course_hold(chi_c, chi, r, 0, t, P);
            end                
            delta_a = roll_hold(phi_c, phi, p, t, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3, % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, t, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, t, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, t, P);
            delta_a = roll_hold(phi_c, phi, p, t, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4, % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, t, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, t, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, t, P);
            delta_e = pitch_hold(theta_c, theta, q, t, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5, % tune the pitch to altitude loop 
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, t, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, t, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, t, P);
            delta_e = pitch_hold(theta_c, theta, q, t, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
     end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)
    global Va_current
    Va_current = Va;
    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, t, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, t, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, t, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0,
        if h<=P.altitude_take_off_zone,     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone, 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone, 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    end
    
    full_throttle = 0.3;
    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            delta_t = full_throttle; % Full throttle
            theta_c = 45.0*pi/180.0; % Consistent angle
            % State transition
            if (h > P.altitude_take_off_zone)
               altitude_state = 2; 
            else
               altitude_state = 1;
            end
        case 2,  % climb zone
            delta_t = full_throttle;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            % State transition
            if (h > h_c - P.altitude_hold_zone)
                altitude_state = 4; % Hold zone
            elseif (h <= P.altitude_take_off_zone)
                altitude_state = 1; % Take-off zone
            else
                altitude_state = 2; % Climb zone
            end
        case 3, % descend zone
            delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if (h < h_c + P.altitude_hold_zone)
                altitude_state = 4; % Hold zone
            else
                altitude_state = 3; % Descend zone
            end
                
        case 4, % altitude hold zone
            delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
            theta_c = altitude_hold(h_c, h, initialize_integrator, P);
            if (h >= h_c + P.altitude_hold_zone)
                altitude_state = 3; % Descend zone
            elseif (h < h_c - P.altitude_hold_zone)
                altitude_state = 2; % Climb zone
            else
                altitude_state = 4; % Hold zone
            end
    end
    
    delta_e = pitch_hold(theta_c, theta, q, t, P);
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_TECS
%   - longitudinal autopilot based on total energy control systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, t, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, t, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, t, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot based on total energy control
    
    
    delta_e = 0;
    delta_t = 0;
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end
   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Roll hold function
function delta_a = roll_hold(phi_c, phi, p, t, P) 
    % Design Parameters
    delta_a_max = P.delta_a_max;
    
    % Control constants
    kp_phi = P.kp_phi;
    kd_phi = P.kd_phi;
    ki_phi = P.ki_phi;
    
    % Compute integral
    error = phi_c - phi;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
       integrator = 0;
       error_d1   = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;
    
    % Output
    delta_a = kp_phi*(phi_c - phi) + ki_phi*integrator - kd_phi*p;
    delta_a = sat(delta_a, delta_a_max, -delta_a_max);
end

% Course hold function
function phi_c = course_hold(chi_c, chi, r, isInit, t, P)
    global Va_current
    % Design Parameters
    phi_c_max = P.phi_c_max;  
    
    % Test
%     Vg = Va_current;
%     P.kp_chi = 2*P.zeta_chi*P.wn_chi*Vg/P.gravity;
%     P.ki_chi = P.wn_chi^2*Vg/P.gravity;
    
    % Control constants
    wn_chi = P.wn_chi;
    kp_chi = P.kp_chi;
    ki_chi = P.ki_chi;
    kd_chi = P.kd_chi;
    
    
    
    % Compute integral
    error = chi_c - chi;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if isInit
       integrator = 0;
       error_d1   = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;
    
    % Output
    phi_c = kp_chi*(chi_c - chi) + ki_chi*integrator;
    phi_c = sat(phi_c, phi_c_max, -phi_c_max);
end
    
function theta_c = airspeed_with_pitch_hold(Va_c, Va, isInit, P)
    % Design Parameters
    theta_c_max = P.theta_c_max;   
    
    
    % Control constants
    ki_V2 = P.ki_V2;
    kp_V2 = P.kp_V2;
    
    % Compute integral
    error = Va_c - Va;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if isInit
       integrator = 0;
       error_d1   = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;
    
    % Output
    theta_c = kp_V2*(Va_c - Va) + ki_V2*integrator;
    theta_c = sat(theta_c, theta_c_max, -theta_c_max);
end
    
function delta_e = pitch_hold(theta_c, theta, q, t, P)
    % Design Parameters
    delta_e_max = P.delta_e_max; 
    
    % Control constants
    kp_theta = P.kp_theta;  
    kd_theta = P.kd_theta;
    ki_theta = P.ki_theta;
    
    % Compute integral
    error = theta_c - theta;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
       integrator = 0;
       error_d1   = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;
    
    % Output
    delta_e = kp_theta*(theta_c - theta) + ki_theta*integrator - kd_theta*q;
    delta_e = sat(delta_e, delta_e_max, -delta_e_max);
end
    
function delta_t = airspeed_with_throttle_hold(Va_c, Va, isInit, P)
% Design Parameters
    delta_t_max = P.delta_t_max;    
    
    % Control constants
    ki_V1 = P.ki_V1;
    kp_V1 = P.kp_V1;
    
    % Compute integral
    error = Va_c - Va;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if isInit
       integrator = 0;
       error_d1   = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;
    
    % Output
    delta_t_trim = P.u_trim(4);
    delta_t = delta_t_trim + kp_V1*(Va_c - Va) + ki_V1*integrator;
    delta_t = sat(delta_t, delta_t_max, 0);
end
    
function theta_c = altitude_hold(h_c, h, isInit, P)
    % Design Parameters
    theta_c_max = P.theta_c_max;      
    
    % Control constants
    ki_h = P.ki_h;
    kp_h = P.kp_h;
    
    % Compute integral
    error = h_c - h;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if isInit
       integrator = 0;
       error_d1   = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;
    
    % Output
    theta_c_unsat = kp_h*(h_c - h) + ki_h*integrator;
    theta_c = sat(theta_c_unsat, theta_c_max, -theta_c_max);
    
    % integrator anti-windup
    if ki_h~=0,
        integrator = integrator + P.Ts/ki_h*(theta_c - theta_c_unsat);
    end
end

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 