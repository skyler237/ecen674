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
   persistent P_
   
   if(t == 0)
       % Initialization
        xhat_att = zeros(2,1);
        xhat_pos = zeros(7,1);
        u_prev = uu;
        P_ = diag([(15*pi/180)^2, (15*pi/180)^2]);
   end
        % Check for new measurements
        new_gyro = 0;
        new_accel = 0;
        new_press = 0;
        new_gps = 0;
        
        meas_diff = uu - u_prev;
%         if(meas_diff(1))
%             new_gyro = 1;
%         end
%         if(meas_diff(4))
%             new_accel = 1;
%         end
%         if(meas_diff(7))
%            new_press = 1; 
%         end
        if(meas_diff(9))
           new_gps = 1; 
        end       
        
        % ====== Attitude estimation ======
        % u
        p = y_gyro_x;
        q = y_gyro_y;
        r = y_gyro_z;
        Va = sqrt(2/P.rho*y_diff_pres);        
        
        % x
        phi = xhat_att(1);
        theta = xhat_att(2);
        sp = sin(phi);
        cp = cos(phi);
        st = sin(theta);
        ct = cos(theta);
        tt = tan(theta);        
        
        f_xu = [p + q*sp*tt + r*cp*tt;...
               q*cp - r*sp];
        G = [1, sp*tt, cp*tt, 0;...
               0, cp,    -sp,   0];        
        
        % Prediction
        N = 15;
        for i=1:N
            xhat_att = xhat_att + (P.Ts/N)*f_xu;   
        
            % Recompute trig values
            phi = xhat_att(1);
            theta = xhat_att(2);
            sp = sin(phi);
            cp = cos(phi);
            st = sin(theta);
            ct = cos(theta);
            tt = tan(theta);
            
            f_xu = [p + q*sp*tt + r*cp*tt;...
                    q*cp - r*sp];
            G = [1, sp*tt, cp*tt, 0;...
                 0, cp,    -sp,   0];   

            % Jacobians
            df = [q*cp*tt - r*sp*tt,     (q*sp - r*cp)/(ct^2);...
                 -q*sp - r*cp,           0];
            A = df;
            P_ = P_ + (P.Ts/N)*(A*P_ + P_*A' + G*P.Q_u*G' + P.Q_att);
        end
       
   
        % Correction        
        h_xu = [q*Va*st + P.gravity*st;...
               r*Va*ct - p*Va*st - P.gravity*ct*sp;...
               -q*Va*ct - P.gravity*ct*cp];
        dh = [0,                 q*Va*ct + P.gravity*ct;...
             -P.gravity*cp*ct,  -r*Va*st - p*Va*ct + P.gravity*sp*st;...
             P.gravity*sp*ct,   (q*Va + P.gravity*cp)*st];
%         C_x = dh(1,:);
%         L_x = P_*C_x'/(P.R_accel_x + C_x*P_*C_x');
%         P_ = (eye(2) - L_x*C_x)*P_;
%         xhat_att = xhat_att + L_x*(y_accel_x - h_xu(1,:));
%         
%         C_y = dh(2,:);
%         L_y = P_*C_y'/(P.R_accel_y + C_y*P_*C_y');
%         P_ = (eye(2) - L_y*C_y)*P_;
%         xhat_att = xhat_att + L_y*(y_accel_y - h_xu(2,:));
%         
%         C_z = dh(3,:);
%         L_z = P_*C_z'/(P.R_accel_z + C_z*P_*C_z');        
%         P_ = (eye(2) - L_z*C_z)*P_;
%         xhat_att = xhat_att + L_z*(y_accel_z - h_xu(3,:));
        
        R_att = diag([P.R_accel_x, P.R_accel_y, P.R_accel_z]);
        y_accel = [y_accel_x; y_accel_y; y_accel_z];
        C_att = dh;
        L_att = P_*C_att'*inv(R_att + C_att*P_*C_att');        
        P_ = (eye(2) - L_att*C_att)*P_;
        xhat_att = xhat_att + L_att*(y_accel - h_xu);
        
        % ====== GPS smoothing ======

   
    % Combine estimated states
    pnhat = 0;
    pehat = 0;
    hhat = 0;
    Vahat = Va;
    phihat = xhat_att(1);
    thetahat = xhat_att(2);
    chihat = 0;
    phat = p;
    qhat = q;
    rhat = r;
    Vghat = 0;
    wnhat = 0;
    wehat = 0;
    psihat = 0;

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

