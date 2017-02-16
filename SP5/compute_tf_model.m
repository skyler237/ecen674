function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input


pn    = x_trim(1);
pe    = x_trim(2);
pd    = x_trim(3);
u     = x_trim(4);
v     = x_trim(5);
w     = x_trim(6);
phi   = x_trim(7);
theta = x_trim(8);
psi   = x_trim(9);
p     = x_trim(10);
q     = x_trim(11);
r     = x_trim(12);

delta_e = u_trim(1);
delta_a = u_trim(2);
delta_r = u_trim(3);
delta_t = u_trim(4);

% compute air data
V_airspeed_body = [u; v; w];
max_airspeed = 10000; % Just to create some cap on the max value
if(norm(V_airspeed_body) > max_airspeed)
   V_airspeed_body = V_airspeed_body/norm(V_airspeed_body)*max_airspeed; 
end

if(norm(V_airspeed_body) ~= 0)
    Va = norm(V_airspeed_body);
else 
    Va = 0.0001;
end

if (V_airspeed_body(1) == 0)
   alpha = sign(V_airspeed_body(3))*pi; 
else
    alpha = atan(V_airspeed_body(3)/V_airspeed_body(1));
end

beta = asin(V_airspeed_body(2)/Va);

a_phi1 = -1/2*P.rho*Va^2*P.S_wing*P.b*P.C_p_p*(P.b/(2*Va));
a_phi2 = 1/2*P.rho*Va^2*P.S_wing*P.b*P.C_p_delta_a;
d_phi1 = q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
% d_phi2 = P.gamma1*p*q - P.gamma2*q*r + 1/2*P.rho*Va^2*P.S_wing*P.b*...
%             (P.C_p_0 + P.C_p_beta*beta - P.C_p_p*(P.b/(2*Va))*d_phi1 + P.C_p_r*(P.b*r/(2*Va)) + P.C_p_delta_r*delta_r) + d_phi1;

d_chi = tan(phi) - phi;

a_beta1 = -1.0*(P.rho*Va*P.S_wing)/(2*P.mass)*P.C_Y_beta;
a_beta2 = (P.rho*Va*P.S_wing)/(2*P.mass)*P.C_Y_delta_r;
d_beta = (1/Va)*(p*w - r*u + P.gravity*cos(theta)*sin(phi)) + (P.rho*Va*P.S_wing)/(2*P.mass)*...
    (P.C_Y_0 + P.C_Y_p*(P.b*p/(2*Va)) + P.C_Y_r*(P.b*r/(2*Va)) + P.C_Y_delta_a*delta_a);

a_theta1 = -1.0*(P.rho*Va^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_q*P.c/(2*Va);
a_theta2 = -1.0*(P.rho*Va^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_alpha;
a_theta3 = (P.rho*Va^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_delta_e;

a_V1 = (P.rho*Va*P.S_wing)/P.mass*(P.C_D_0 + P.C_D_alpha*alpha + P.C_D_delta_e*delta_e) + ...
    (P.rho*P.S_prop)/P.mass*P.C_prop*Va;
a_V2 = (P.rho*P.S_prop)/P.mass*P.C_prop*P.k_motor^2*delta_t;
a_V3 = P.gravity;
        
Va_trim = Va;
phi_trim = phi;
theta_trim = theta;
psi_trim = psi;
    
% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

