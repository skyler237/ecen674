function [a_phi1, a_phi2, a_beta1, a_beta2, a_theta1, a_theta2, a_theta3, a_V1, a_V2, a_V3]...
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
ur = V_airspeed_body(1);
vr = V_airspeed_body(2);
wr = V_airspeed_body(3);

Va = sqrt(ur^2 + vr^2 + wr^2);

alpha = atan2(wr, ur);
beta = atan2(vr, sqrt(ur^2 + wr^2));

if (Va ~= 0)
    a_phi1 = -1/2*P.rho*Va^2*P.S_wing*P.b*P.C_p_p*(P.b/(2*Va));
    a_phi2 = 1/2*P.rho*Va^2*P.S_wing*P.b*P.C_p_delta_a;
else
    a_phi1 = 0;
    a_phi2 = 0;
end
% d_phi1 = q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
% d_phi2 = P.gamma1*p*q - P.gamma2*q*r + 1/2*P.rho*Va^2*P.S_wing*P.b*...
%             (P.C_p_0 + P.C_p_beta*beta - P.C_p_p*(P.b/(2*Va))*d_phi1 + P.C_p_r*(P.b*r/(2*Va)) + P.C_p_delta_r*delta_r) + d_phi1;

% d_chi = tan(phi) - phi;

a_beta1 = -1.0*(P.rho*Va*P.S_wing)/(2*P.mass)*P.C_Y_beta;
a_beta2 = (P.rho*Va*P.S_wing)/(2*P.mass)*P.C_Y_delta_r;
% d_beta = (1/Va)*(p*w - r*u + P.gravity*cos(theta)*sin(phi)) + (P.rho*Va*P.S_wing)/(2*P.mass)*...
%     (P.C_Y_0 + P.C_Y_p*(P.b*p/(2*Va)) + P.C_Y_r*(P.b*r/(2*Va)) + P.C_Y_delta_a*delta_a);

if (Va ~= 0.0)
    a_theta1 = -1.0*(P.rho*Va^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_q*P.c/(2*Va);
    a_theta2 = -1.0*(P.rho*Va^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_alpha;
    a_theta3 = (P.rho*Va^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_delta_e;
else
    a_theta1 = 0;
    a_theta2 = 0;
    a_theta3 = 0;
end

a_V1 = (P.rho*Va*P.S_wing)/P.mass*(P.C_D_0 + P.C_D_alpha*alpha + P.C_D_delta_e*delta_e) + ...
    (P.rho*P.S_prop)/P.mass*P.C_prop*Va;
a_V2 = (P.rho*P.S_prop/P.mass)*P.C_prop*P.k_motor^2*delta_t;
a_V3 = P.gravity;
        
Va_trim = Va;
phi_trim = phi;
theta_trim = theta;
psi_trim = psi;

