clear all
P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;

Jx = P.Jx;
Jy = P.Jy;
Jz = P.Jz;
Jxz = P.Jxz;
P.gamma = (Jx*Jz - Jxz^2);
P.gamma1 = (Jxz*(Jx - Jy + Jz))/P.gamma;
P.gamma2 = (Jz*(Jz - Jy) + Jxz^2)/P.gamma;
P.gamma3 = Jz/P.gamma;
P.gamma4 = Jxz/P.gamma;
P.gamma5 = (Jz - Jx)/Jy;
P.gamma6 = Jxz/Jy;
P.gamma7 = ((Jx - Jy)*Jx + Jxz^2)/P.gamma;
P.gamma8 = Jx/P.gamma;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
%Added
P.C_p_0 = P.gamma3*P.C_ell_0 + P.gamma4*P.C_n_0;
P.C_p_beta = P.gamma3*P.C_ell_beta + P.gamma4*P.C_n_beta;
P.C_p_p = P.gamma3*P.C_ell_p + P.gamma4*P.C_n_p;
P.C_p_r = P.gamma3*P.C_ell_r + P.gamma4*P.C_n_r;
P.C_p_delta_a = P.gamma3*P.C_ell_delta_a + P.gamma4*P.C_n_delta_a;
P.C_p_delta_r = P.gamma3*P.C_ell_delta_r + P.gamma4*P.C_n_delta_r;
P.C_r_0 = P.gamma4*P.C_ell_0 + P.gamma8*P.C_n_0;
P.C_r_beta = P.gamma4*P.C_ell_beta + P.gamma8*P.C_n_beta;
P.C_r_p = P.gamma4*P.C_ell_p + P.gamma8*P.C_n_p;
P.C_r_r = P.gamma4*P.C_ell_r + P.gamma8*P.C_n_r;
P.C_r_delta_a = P.gamma4*P.C_ell_delta_a + P.gamma8*P.C_n_delta_a;
P.C_r_delta_r = P.gamma4*P.C_ell_delta_r + P.gamma8*P.C_n_delta_r;
% =======
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% wind parameters
P.wind_n = 3;%3;
P.wind_e = -3;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;


% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 30;
gamma = 0;  % desired flight path angle (radians)
R     = inf;         % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
P.Ts = 0.01;
P.Ts_gps = 1.0;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -100;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -100;  % initial Down position (negative altitude)
P.h0     = -P.pd0;
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

[P.a_phi1, P.a_phi2, P.a_beta1, P.a_beta2, P.a_theta1, P.a_theta2, P.a_theta3, P.a_V1, P.a_V2, P.a_V3]...
    = compute_tf_constants(x_trim, u_trim, P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);


% ========== Control Constants ===============
% <<<<<<< Roll hold >>>>>>>
% Design parameters
P.phi_max = 50.0*pi/180.0;
P.delta_a_max = 30.0*pi/180.0; 
e_phi_max = 120*pi/180.0;
zeta_phi = 0.95;
% Control constants
P.kp_phi = P.delta_a_max/e_phi_max;  
P.wn_phi = sqrt(abs(P.a_phi2)*P.delta_a_max/e_phi_max);    
P.kd_phi = (2*zeta_phi*P.wn_phi - P.a_phi1)/P.a_phi2;
P.ki_phi = 0.04;

% <<<<<<< Course hold >>>>>>>
% Design Parameters
P.phi_c_max = 45*pi/180.0;
P.W_chi = 9;
P.zeta_chi = 3.0;  
% Control constants
Vg = P.Va0;
P.wn_chi = (1/P.W_chi)*P.wn_phi;
P.kp_chi = 2*P.zeta_chi*P.wn_chi*Vg/P.gravity;
P.ki_chi = P.wn_chi^2*Vg/P.gravity;
P.kd_chi = 0.0;

% <<<<<<< Pitch hold >>>>>>>
% Design Parameters
P.delta_e_max = 45.0*pi/180.0; 
e_theta_max = 10*pi/180.0;
zeta_theta = 0.3;
% Control constants
P.kp_theta = P.delta_e_max/e_theta_max*sign(P.a_theta3);  
P.wn_theta = sqrt(P.a_theta2 + (P.delta_e_max/e_theta_max)*abs(P.a_theta3)); % less than or equal
P.kd_theta = (2*zeta_theta*P.wn_theta - P.a_theta1)/P.a_theta3;
P.ki_theta = -0.8;
P.K_DC_theta = P.kp_theta*P.a_theta3/(P.a_theta2 + P.kp_theta*P.a_theta3);

% <<<<<<< Airspeed with pitch hold >>>>>>>
% Design Parameters
P.theta_c_max = 60*pi/180.0;
W_V2 = 15;
zeta_V2 = 3.0;   
% Control constants
P.wn_V2 = (1/W_V2)*P.wn_theta;
P.ki_V2 = -1*P.wn_V2^2/(P.K_DC_theta*P.gravity);
P.kp_V2 = (P.a_V1 - 2*zeta_V2*P.wn_V2)/(P.K_DC_theta*P.gravity);
% P.ki_V2 = 0.1;
% P.kp_V2 = 0.1;

% <<<<<<< Airspeed with throttle hold >>>>>>>
% Design Parameters
P.delta_t_max = 1.0;
P.wn_V1 = 0.25;
zeta_V1 = 3.0;   
% Control constants
P.ki_V1 =  P.wn_V1^2/(P.a_V2);
P.kp_V1 = (2*zeta_V1*P.wn_V1 - P.a_V1)/(P.a_V2);

% <<<<<<< Altitude hold >>>>>>>
% Design Parameters
W_h = 7;
zeta_h = 0.5; 
% Control constants
Va = P.Va0;
P.wn_h = (1/W_h)*P.wn_theta;
P.ki_h = P.wn_h^2/(P.K_DC_theta*Va);
P.kp_h = (2*zeta_h*P.wn_h)/(P.K_DC_theta*Va);


% ============= Autopilot parameters ================
P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 30;


% ============= Sensor parameters ================
P.sigma_accel_x = 0.0025;
P.sigma_accel_y = 0.0025;
P.sigma_accel_z = 0.0025;

P.sigma_gyro_x = 0.13;
P.sigma_gyro_y = 0.13;
P.sigma_gyro_z = 0.13;

P.sigma_gps_n = .21;
P.sigma_gps_e = .21;
P.sigma_gps_h = .40;
P.k_gps = 1100;
P.sigma_gps_Vg = 0.05;

P.sigma_static_press = 0.01;
P.sigma_diff_press = 0.002;
P.beta_static_press = 0.125;
P.beta_diff_press = 0.020;


P.bias_gyro_x = 0;
P.bias_gyro_y = 0;
P.bias_gyro_z = 0;

% ============= Estimator parameters ================
% Attitude noise

% P.xi_phi = 1e-4;
% P.xi_theta = 1e-4;
% xi_att = [P.xi_phi, P.xi_theta];
% P.Q_att = diag(xi_att.^2);
P.Q_att = diag([1e-9, 1e-9]);

P.xi_p = P.sigma_gyro_x;
P.xi_q = P.sigma_gyro_y;
P.xi_r = P.sigma_gyro_z;
P.xi_Va = P.sigma_diff_press;
% xi_u = 0*(1/10^7)*[P.xi_p, P.xi_q, P.xi_r, P.xi_Va];
xi_u = [P.xi_p, P.xi_q, P.xi_r, P.xi_Va];
P.Q_u = diag(xi_u.^2);

P.R_accel_x = P.sigma_accel_x^2;
P.R_accel_y = P.sigma_accel_y^2;
P.R_accel_z = P.sigma_accel_z^2;

P.LPF_gyro_alpha = 0.7;
P.LPF_diff_press_alpha = 0.1;
P.LPF_static_press_alpha = 0.1;

% Position noise
scale = 1e2;
xi_pn = 1*scale;
xi_pe = 1*scale;
xi_Vg = 1e4;
xi_chi = 1e-6;
xi_wn = 1e4;
xi_we = 1e4;
xi_psi = 1e-1;
xi_pos = [xi_pn, xi_pe, xi_Vg, xi_chi, xi_wn, xi_we, xi_psi];
P.Q_pos = diag(xi_pos.^2);

% eta_gps_n = P.sigma_gps_n^2;
% eta_gps_e = P.sigma_gps_e^2;
% eta_gps_Vg = P.sigma_gps_Vg^2;
% eta_gps_chi = (P.sigma_gps_Vg/P.Va0)^2; % Does this work??
% eta_wind_n = 1e-2;
% eta_wind_e = 1e-2;
% eta_pos = [eta_gps_n, eta_gps_e, eta_gps_Vg, eta_gps_chi, eta_wind_n, eta_wind_e];
% P.R_pos = diag(eta_pos.^2);

% =============== Guidance model parameters ====================
P.b_chidot = 0.9;
P.b_chi = 0.6; % P.kp_chi = 2.96
P.b_hdot = 0.8;
P.b_h = 0.8;       % P.kp_h = 0.0372
P.b_Va = 7.0;
% P.b_phi = 70.5e1; % P.kp_phi = 0.25

% =============== Path following parameters ====================
P.k_path = 0.02;
P.k_orbit = 2;
P.chi_inf = pi/4;
P.gamma_max = pi/4;


% =============== path manager parameters ====================
% number of waypoints in data structure
P.size_waypoint_array = 100;
P.R_min = P.Va0^2/P.gravity/tan(P.phi_max);

% create random city map
city_width      = 2000;  % the city is of size (width)x(width)
building_height = 300;   % maximum height of buildings
%building_height = 1;   % maximum height of buildings (for camera)
num_blocks      = 5;    % number of blocks in city
street_width    = .8;   % percent of block that is street.
P.map = createWorld(city_width, building_height, num_blocks, street_width);

