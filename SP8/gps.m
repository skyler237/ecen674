% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
    
    % Compute drift
    persistent nu_n
    persistent nu_e
    persistent nu_h
    
    if t==0
        nu_n = 0;
        nu_e = 0;
        nu_h = 0;
    end
    
    nu_n = exp(-P.k_gps*P.Ts_gps)*nu_n + P.sigma_gps_n*randn;
    nu_e = exp(-P.k_gps*P.Ts_gps)*nu_e + P.sigma_gps_e*randn;
    nu_h = exp(-P.k_gps*P.Ts_gps)*nu_h + P.sigma_gps_h*randn;
 
    % construct North, East, and altitude GPS measurements
%     y_gps_n = pn;
%     y_gps_e = pe; 
%     y_gps_h = -pd;
    y_gps_n = pn + nu_n;
    y_gps_e = pe + nu_e; 
    y_gps_h = -pd + nu_h; 
    
    % construct groundspeed and course measurements
%     y_gps_Vg     = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2);
%     y_gps_course = atan2(Va*sin(psi)+we, Va*cos(psi)+wn);
    y_gps_Vg     = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2) + P.sigma_gps_Vg*randn;
    sigma_gps_chi = P.sigma_gps_Vg/Va;
    y_gps_course = atan2(Va*sin(psi)+we, Va*cos(psi)+wn) + sigma_gps_chi*randn;

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



