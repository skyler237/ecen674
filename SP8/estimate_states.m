% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%


% Notes:...
% * Fix chi/psi estimation (maybe a Vg problem?)
% * Fix throttle autopilot control
% * Loosen up phi/roll control
% * Figure out position spikes (maybe byproduct of course problems?)

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
   persistent u_prev
   persistent xhat_att
   persistent xhat_pos
   persistent P_att
   persistent P_pos
   
   persistent p_prev
   persistent q_prev
   persistent r_prev
   persistent static_press_prev
   persistent diff_press_prev
   
    if(t == 0)
       % Initialization
        % Attitude
        xhat_att = zeros(2,1);
        xhat_pos = [0; 0; 0.1; 0; 0; 0; 0];
        u_prev = uu;
        P_att = diag([(15*pi/180)^2, (15*pi/180)^2]);

        p_prev = y_gyro_x;
        q_prev = y_gyro_y;
        r_prev = y_gyro_z;
        diff_press_prev = y_diff_pres;
        static_press_prev = y_static_pres;

        % Position
        pos_covariance = [20, 20, 10, 10*pi/180, 10, 10, 10*pi/180];
        P_pos = diag(pos_covariance.^2);
    end
    
    % Check for new measurements
    new_gps = 0;
        
    meas_diff = uu - u_prev;
    if(meas_diff(9))
       new_gps = 1; 
    end       
        
    % ====== Attitude estimation ======
    % u
    p = y_gyro_x;
    q = y_gyro_y;
    r = y_gyro_z;
    diff_press = y_diff_pres;
    static_press = y_static_pres;  
    Va = sqrt((2/P.rho)*y_diff_pres);
    
    
    phat = LPF(p, p_prev, P.LPF_gyro_alpha);
    qhat = LPF(q, q_prev, P.LPF_gyro_alpha);
    rhat = LPF(r, r_prev, P.LPF_gyro_alpha);
    Vahat = sqrt((2/P.rho)*LPF(diff_press, diff_press_prev, P.LPF_diff_press_alpha));
    hhat = LPF(static_press, static_press_prev, P.LPF_static_press_alpha)/(P.rho*P.gravity);
    p_prev = p;
    q_prev = q;
    r_prev = r;    
    diff_press_prev = diff_press;
    static_press_prev = static_press;

    % x
    phi = xhat_att(1);
    theta = xhat_att(2);
    sp = sin(phi);
    cp = cos(phi);
    st = sin(theta);
    ct = cos(theta);
    tt = tan(theta);        

    f_att = [p + q*sp*tt + r*cp*tt;...
           q*cp - r*sp];
    G = [1, sp*tt, cp*tt, 0;...
           0, cp,    -sp,   0];        

    % Prediction
    N = 10;
    for i=1:N
        xhat_att = xhat_att + (P.Ts/N)*f_att;   

        % Recompute trig values
        phi = xhat_att(1);
        theta = xhat_att(2);
        sp = sin(phi);
        cp = cos(phi);
        st = sin(theta);
        ct = cos(theta);
        tt = tan(theta);

        f_att = [p + q*sp*tt + r*cp*tt;...
                q*cp - r*sp];
        G = [1, sp*tt, cp*tt, 0;...
             0, cp,    -sp,   0];   

        % Jacobians
        df_att = [q*cp*tt - r*sp*tt,     (q*sp - r*cp)/(ct^2);...
             -q*sp - r*cp,           0];
        A = df_att;
        P_att = P_att + (P.Ts/N)*(A*P_att + P_att*A' + G*P.Q_u*G' + P.Q_att);
    end


    % Correction        
    h_att = [q*Va*st + P.gravity*st;...
           r*Va*ct - p*Va*st - P.gravity*ct*sp;...
           -q*Va*ct - P.gravity*ct*cp];
    dh_att = [0,                 q*Va*ct + P.gravity*ct;...
         -P.gravity*cp*ct,  -r*Va*st - p*Va*ct + P.gravity*sp*st;...
         P.gravity*sp*ct,   (q*Va + P.gravity*cp)*st];
%         C_x = dh_att(1,:);
%         L_x = P_att*C_x'/(P.R_accel_x + C_x*P_att*C_x');
%         P_att = (eye(2) - L_x*C_x)*P_att;
%         xhat_att = xhat_att + L_x*(y_accel_x - h_att(1,:));
%         
%         C_y = dh_att(2,:);
%         L_y = P_att*C_y'/(P.R_accel_y + C_y*P_att*C_y');
%         P_att = (eye(2) - L_y*C_y)*P_att;
%         xhat_att = xhat_att + L_y*(y_accel_y - h_att(2,:));
%         
%         C_z = dh_att(3,:);
%         L_z = P_att*C_z'/(P.R_accel_z + C_z*P_att*C_z');        
%         P_att = (eye(2) - L_z*C_z)*P_att;
%         xhat_att = xhat_att + L_z*(y_accel_z - h_att(3,:));

    R_att = diag([P.R_accel_x, P.R_accel_y, P.R_accel_z]);
    y_accel = [y_accel_x; y_accel_y; y_accel_z];
    C_att = dh_att;
    L_att = (1/10000)*P_att*C_att'*inv(R_att + C_att*P_att*C_att'); % HACK   
    P_att = (eye(2) - L_att*C_att)*P_att;
    xhat_att = xhat_att + L_att*(y_accel - h_att);
    
    phihat = xhat_att(1);
    thetahat = xhat_att(2);

    % ====== GPS smoothing ======    
    
        % u
        Va = Vahat;
        q = qhat;
        r = rhat;
        phi = phihat;
        theta = thetahat;
        cp = cos(phi);
        sp = sin(phi);
        tp = tan(phi);
        ct = cos(theta);
        st = sin(theta); 
        
        % xhat
        pn = xhat_pos(1);
        pe = xhat_pos(2);
        Vg = xhat_pos(3);
        chi = xhat_pos(4);
        cx = cos(chi);
        sx = sin(chi);
        wn = xhat_pos(5);
        we = xhat_pos(6);
        psi = xhat_pos(7);
        cs = cos(psi);
        ss = sin(psi);        
        psidot = q*(sp/ct) + r*(cp/ct);
        
        f_pos = [Vg*cx;
                 Vg*sx;
                 ((Va*cs + wn)*(-Va*psidot*ss) + (Va*ss + we)*(Va*psidot*cs))/Vg;
                 (P.gravity/Vg)*tp*cos(chi - psi);
                 0;
                 0;
                 psidot];
        
        % Prediction
        N = 100;
        for i=1:N
            xhat_pos = xhat_pos + (P.Ts_gps/N)*f_pos;

            % Recompute trig values
            Vg = xhat_pos(3);
            chi = xhat_pos(4);
            cx = cos(chi);
            sx = sin(chi);
            wn = xhat_pos(5);
            we = xhat_pos(6);
            psi = xhat_pos(7);
            cs = cos(psi);
            ss = sin(psi);        
            psidot = q*(sp/ct) + r*(cp/ct);
            Vgdot = ((Va*cs + wn)*(-Va*psidot*ss) + (Va*ss + we)*(Va*psidot*cs))/Vg;

            f_pos = [Vg*cx;
                     Vg*sx;
                     Vgdot;
                     (P.gravity/Vg)*tp*cos(chi - psi);
                     0;
                     0;
                     psidot];  

            % Jacobian
            dVgdot_dpsi = (-psidot*Va*(wn*cs + we*ss))/Vg;
            dchidot_dVg = (-P.gravity/Vg^2)*tp*cos(chi - psi);
            dchidot_dchi = (-P.gravity/Vg)*tp*sin(chi - psi);
            dchidot_dpsi = (P.gravity/Vg)*tp*sin(chi - psi);
            df_pos = [0, 0, cx,         -Vg*sx,         0,              0,              0;
                      0, 0, sx,         Vg*cx,          0,              0,              0;
                      0, 0, -Vgdot/Vg,  0,              -psidot*Va*ss,  psidot*Va*cs,   dVgdot_dpsi;
                      0, 0, dchidot_dVg, dchidot_dchi,  0,              0,              dchidot_dpsi;
                      0, 0, 0,          0,              0,              0,              0;
                      0, 0, 0,          0,              0,              0,              0;
                      0, 0, 0,          0,              0,              0,              0];
                      
            A = df_pos;
            P_pos = P_pos + (P.Ts_gps/N)*(A*P_pos + P_pos*A' + P.Q_pos);
        end
        
   if(new_gps)
        % Correction       
        h_pos = [pn;
                 pe;
                 Vg;
                 chi;
                 Va*cs + wn - Vg*cx;
                 Va*ss + we - Vg*sx];
             
        dh_pos = [1, 0,  0,   0,     0, 0,  0;
                  0, 1,  0,   0,     0, 0,  0;
                  0, 0,  1,   0,     0, 0,  0;
                  0, 0,  0,   1,     0, 0,  0;
                  0, 0, -cx,  Vg*sx, 1, 0, -Va*ss;
                  0, 0, -sx, -Vg*cx, 0, 1,  Va*cs];

        y_wind_n = 0;
        y_wind_e = 0;
        y_gps = [y_gps_n; y_gps_e; y_gps_Vg; y_gps_course; y_wind_n; y_wind_e];
        
        %TEST
        eta_gps_n = P.sigma_gps_n^2;
        eta_gps_e = P.sigma_gps_e^2;
        eta_gps_Vg = P.sigma_gps_Vg^2;
        eta_gps_chi = (P.sigma_gps_Vg/Vahat)^2;
        eta_wind_n = 1000;
        eta_wind_e = 1000;
        eta_pos = [eta_gps_n, eta_gps_e, eta_gps_Vg, eta_gps_chi, eta_wind_n, eta_wind_e];
        R_pos_test = diag(eta_pos.^2);
        
        C_pos = dh_pos;
        L_pos = P_pos*C_pos'*inv(R_pos_test + C_pos*P_pos*C_pos'); % R test
        P_pos = (eye(7) - L_pos*C_pos)*P_pos;
        xhat_pos = xhat_pos + L_pos*(y_gps - h_pos);        
    end
        
        

   
    % Combine estimated states
    pnhat  =  xhat_pos(1);
    pehat  =  xhat_pos(2);    
    Vghat  =  xhat_pos(3);
    chihat =  xhat_pos(4); 
    wnhat  =  xhat_pos(5);
    wehat  =  xhat_pos(6);
    psihat =  xhat_pos(7);

    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;

    xhat = [...
    pnhat;...
    pehat;...
    hhat;...
    Vahat;...
    alphahat;...
    betahat;...
    phihat;...
    thetahat;...
    chihat;...
    phat;...
    qhat;...
    rhat;...
    Vghat;...
    wnhat;...
    wehat;...
    psihat;...
    bxhat;...
    byhat;...
    bzhat;...
    ];
    
end

function output = LPF(state, state_prev, alpha)
    output = alpha*state_prev + (1-alpha)*state;
end

