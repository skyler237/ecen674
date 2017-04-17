% dubinsParameters
%   - Find Dubin's parameters between two configurations
%
% input is:
%   start_node  - [wn_s, we_s, wd_s, chi_s, 0, 0]
%   end_node    - [wn_e, wn_e, wd_e, chi_e, 0, 0]
%   R           - minimum turn radius
%
% output is:
%   dubinspath  - a matlab structure with the following fields
%       dubinspath.ps   - the start position in re^3
%       dubinspath.chis - the start course angle
%       dubinspath.pe   - the end position in re^3
%       dubinspath.chie - the end course angle
%       dubinspath.R    - turn radius
%       dubinspath.L    - length of the Dubins path
%       dubinspath.cs   - center of the start circle
%       dubinspath.lams - direction of the start circle
%       dubinspath.ce   - center of the end circle
%       dubinspath.lame - direction of the end circle
%       dubinspath.w1   - vector in re^3 defining half plane H1
%       dubinspath.q1   - unit vector in re^3 along straight line path
%       dubinspath.w2   - vector in re^3 defining position of half plane H2
%       dubinspath.w3   - vector in re^3 defining position of half plane H3
%       dubinspath.q3   - unit vector defining direction of half plane H3
% 

function [dubinspath, success] = dubinsParameters(start_node, end_node, R)

  ell = norm(start_node(1:2)-end_node(1:2));
  if ell<2*R,
      disp('The distance between nodes must be larger than 2R.');
      dubinspath = [];
      success = 0;
  else
      
    
    
    ps   = start_node(1:3)';
    chis = start_node(4);
    pe   = end_node(1:3)';
    chie = end_node(4);
    
    chi_vec_s = [cos(chis); sin(chis); 0];
    chi_vec_e = [cos(chie); sin(chie); 0];
    
    crs = ps + R*rotz(pi/2)*chi_vec_s;
    cls = ps + R*rotz(-pi/2)*chi_vec_s;
    cre = pe + R*rotz(pi/2)*chi_vec_e;
    cle = pe + R*rotz(-pi/2)*chi_vec_e;
   
    % compute L1
    c_dist = cre - crs;
    theta = atan2(c_dist(2),c_dist(2));
    L1 = norm(c_dist)*R*(amod(amod(theta - pi/2) - amod(chis - pi/2)) + ...
                         amod(amod(chie - pi/2) - amod(theta - pi/2)));
    % compute L2
    c_dist = cle - crs;
    ell = norm(c_dist);    
    theta = atan2(c_dist(2),c_dist(2));
    theta2 = theta - pi/2 + asin(2*R/ell);
    if ~isreal(theta2)
      L2 = 9999; 
    else
      L2 = sqrt(ell^2 - 4*R^2) + R*amod(amod(theta2) - amod(chis - pi/2)) +...
                                 R*amod(amod(theta2 + pi) - amod(chie + pi/2));
    end
    % compute L3
    c_dist = cre - cls;
    ell = norm(c_dist);    
    theta = atan2(c_dist(2),c_dist(2));
    theta2 = acos(2*R/ell);
    if ~isreal(theta2)
      L3 = 9999;
    else
      L3 = sqrt(ell^2 - 4*R^2) + R*amod(amod(chis + pi/2) - amod(theta+theta2)) +...
                                 R*amod(amod(chie - pi/2) - amod(theta+theta2-pi));
    end
    % compute L4
    c_dist = cle - cls;  
    theta = atan2(c_dist(2),c_dist(2));
    L4 = norm(c_dist) + R*amod(amod(chis + pi/2) - amod(theta+pi/2)) + ...
                        R*amod(amod(theta+pi/2) - amod(chie + pi/2));
    % L is the minimum distance
    [L,idx] = min([L1,L2,L3,L4]);
    e1 = [1; 0; 0];
    switch(idx),
        case 1,
            cs = crs;
            lams = 1;
            ce = cre;
            lame = 1;
            q1 = (ce - cs)/norm(ce - cs);
            w1 = cs + R*rotz(-pi/2)*q1;
            w2 = ce + R*rotz(-pi/2)*q1;
        case 2,   
            cs = crs;
            lams = 1;
            ce = cle;
            lame = -1;
            c_dist = ce - cs;
            ell = norm(c_dist);
            theta = atan2(c_dist(2), c_dist(1));
            theta2 = theta - pi/2 + asin(2*R/ell);
            q1 = rotz(theta2 + pi/2)*e1;
            w1 = cs + R*rotz(theta2)*e1;
            w2 = ce + R*rotz(theta2 + pi)*e1;
        case 3,
            cs = cls;
            lams = -1;
            ce = cre;
            lame = 1;
            c_dist = ce - cs;
            ell = norm(c_dist);
            theta = atan2(c_dist(2), c_dist(1));
            theta2 = acos(2*R/ell);
            q1 = rotz(theta + theta2 - pi/2)*e1;
            w1 = cs + R*rotz(theta + theta2)*e1;
            w2 = ce + R*rotz(theta + theta2 - pi)*e1;
         case 4,
            cs = cls;
            lams = -1;
            ce = cle;
            lame = -1;
            q1 = (ce - cs)/norm(ce - cs);
            w1 = cs + R*rotz(pi/2)*q1;
            w2 = ce + R*rotz(pi/2)*q1;
    end
    w3 = pe;
    q3 = rotz(chie)*e1;
    
    % assign path variables
    dubinspath.ps   = ps;
    dubinspath.chis = chis;
    dubinspath.pe   = pe;
    dubinspath.chie = chie;
    dubinspath.R    = R;
    dubinspath.L    = L;
    dubinspath.cs   = cs;
    dubinspath.lams = lams;
    dubinspath.ce   = ce;
    dubinspath.lame = lame;
    dubinspath.w1   = w1;
    dubinspath.q1   = q1;
    dubinspath.w2   = w2;
    dubinspath.w3   = w3;
    dubinspath.q3   = q3;
    success = 1;
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% rotz(theta)
%%   rotation matrix about the z axis.
function R = rotz(theta)
    R = [...
        cos(theta), -sin(theta), 0;...
        sin(theta), cos(theta), 0;...
        0, 0, 1;...
        ];
end

% Angle mod
function output = amod(theta)
    output = mod(theta,2*pi);
end