%% pathRRT
%%  - create a path from a start node to an end node
%%    using the RRT algorithm.
%%  - RRT = Rapidly-exploring Random Tree
%%  
%% Last Modified - 6/8/2006 - R. Beard
%%               - 4/15/2010 - R. Beard

function path_out=planRRTDubins(wpp_start, wpp_end, R, map)

    % standard length of path segments
%     segmentLength = 100;
    segmentLength = 2*R;

    % desired down position is down position of end node
    pd = wpp_end(3);
    chi = 0;
    
    % specify start and end nodes from wpp_start and wpp_end
    start_node = [wpp_start(1), wpp_start(2), pd, chi, 0, 0, 0];
    end_node = [wpp_end(1), wpp_end(2), pd, chi, 0, 0, 0];
    % format:  [N, E, D, chi, cost, parent_idx, flag_connect_to_goal]

    % establish tree starting with the start node
    tree = start_node;
    
    % check to see if start_node connects directly to end_node
    if ( (collision(start_node,end_node,R,map)==0) )
        tmp = end_node(1:3)-start_node(1:3);
        chi = atan2(tmp(2), tmp(1));
        end_node(4) = chi;
        path = [start_node; end_node];
    else
        numPaths = 0;
        while numPaths<3,
            [tree,flag] = extendTree(tree,end_node,segmentLength,R,map,pd,chi);
            numPaths = numPaths + flag;
        end
    end

    % find path with minimum cost to end_node
    path = findMinimumPath(tree,end_node);
    path_out = smoothPath(path,R,map);
    plotmap(map,path,path_out,tree);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% generateRandomNode
%%   create a random node (initialize)
function node=generateRandomNode(map,pd,chi)

    % randomly pick configuration
    pn       = map.width*rand;
    pe       = map.width*rand;
    pd       = pd; % constant altitute paths
    cost     = 0;
    node     = [pn, pe, pd, chi, cost, 0, 0];
    % format:  [N, E, D, chi, cost, parent_idx, flag_connect_to_goal]
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% collision
%%   check to see if a node is in collsion with obstacles
function collision_flag = collision(start_node, end_node, R, map)

    collision_flag = 0;
    if (norm(start_node(1:3)-end_node(1:3))<2*R )
        % Consider a too short path to be a collision
        collision_flag = 1;
    else

        [X,Y,Z] = pointsAlongDubinsPath(start_node, end_node, R, 0.1);

        for i = 1:length(X),
            if Z(i) >= downAtNE(map, X(i), Y(i)),
                collision_flag = 1;
            end
        end
    
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pointsAlongDubinsPath
%%   Find points along dubins path separted by Del (to be used in
%%   collision detection)
function [X,Y,Z] = pointsAlongDubinsPath(start_node, end_node, R, Del)
    
    % Get dubins path parameters
    dubinspath = dubinsParameters(start_node, end_node, R);
    
    % Get points along beginning arc
    [arc1X, arc1Y, arc1Z] = pointsAlongArc(dubinspath.ps, dubinspath.w1, dubinspath.cs, dubinspath.R, dubinspath.lams, Del);
    
    % Get points along line
    [lineX, lineY, lineZ] = pointsAlongLine(dubinspath.w1, dubinspath.w2, Del);
    
    % Get points along ending arc
    [arc2X, arc2Y, arc2Z] = pointsAlongArc(dubinspath.w2, dubinspath.pe, dubinspath.ce, dubinspath.R, dubinspath.lame, Del);
    

    X = [arc1X, lineX, arc2X];
    Y = [arc1Y, lineY, arc2Y];
    Z = [arc1Z, lineZ, arc2Z];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pointsAlongArc
%%   Find points along dubins path separted by Del (to be used in
%%   collision detection)
function [X,Y,Z] = pointsAlongArc(start_pos, end_pos, center, R, lam, Del)

    % Initialize starting point
    X = [start_pos(1)];
    Y = [start_pos(2)];
    Z = [start_pos(3)];
    
    c2beg = start_pos - center;
    c2end = end_pos - center;
    
    beg_angle = atan2(c2beg(2), c2beg(1));
    end_angle = atan2(c2end(2), c2end(1));
    
    delta_theta = mod(lam*(end_angle - beg_angle),2*pi);
    
    thetaDel = Del/R;

    w = start_pos(1:3);
    theta = beg_angle;
    for i=2:floor(delta_theta/thetaDel)
        theta = theta + thetaDel;
        w(1) = cos(theta);
        w(2) = sin(theta);
        X = [X, w(1)];
        Y = [Y, w(2)];
        Z = [Z, w(3)];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pointsAlongLine
%%   Find points along straight-line path separted by Del (to be used in
%%   collision detection)
function [X,Y,Z] = pointsAlongLine(start_node, end_node, Del)

    % Initialize starting point
    X = [start_node(1)];
    Y = [start_node(2)];
    Z = [start_node(3)];
    
    % Get dubins path parameters
    
    % Get points from beginning arc
    
    q = [end_node(1:3)-start_node(1:3)];
    L = norm(q);
    q = q/L;
    
    w = start_node(1:3);
    for i=2:floor(L/Del),
        w = w + Del*q;
        X = [X, w(1)];
        Y = [Y, w(2)];
        Z = [Z, w(3)];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% downAtNE
%%   find the world down coordinate at a specified (n,e) location
function down = downAtNE(map, n, e)

      [d_n,idx_n] = min(abs(n - map.buildings_n));
      [d_e,idx_e] = min(abs(e - map.buildings_e));

      if (d_n<=map.BuildingWidth) && (d_e<=map.BuildingWidth),
          down = -map.heights(idx_e,idx_n);
      else
          down = 0;
      end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% extendTree
%%   extend tree by randomly selecting point and growing tree toward that
%%   point
function [new_tree,flag] = extendTree(tree,end_node,segmentLength,R,map,pd,chi)

  flag1 = 0;
  while flag1==0,
    % select a random point
    randomNode=generateRandomNode(map,pd,chi);
    
    % find leaf on node that is closest to randomPoint
    tmp = tree(:,1:3)-ones(size(tree,1),1)*randomNode(1:3);
    [dist,idx] = min(diag(tmp*tmp'));
    L = min(sqrt(dist), segmentLength); 
    
    tmp = randomNode(1:3)-tree(idx,1:3);
    chi = atan2(tmp(2), tmp(1));
%     new_point = tree(idx,1:3)+L*(tmp/norm(tmp));
    new_point = tree(idx,1:3)+tmp;
    tmp_node = [new_point, chi];

%     fprintf('extend A\n')
    if collision(tree(idx,:), tmp_node, R, map)==0,
      dubinspath = dubinsParameters(tree(idx,:), tmp_node, R);
%       fprintf('extend B\n')
      cost     = tree(idx,5) + dubinspath.L;
      new_node = [new_point, chi, cost, idx, 0]; 
      new_tree = [tree; new_node];
      flag1=1;
    end
  end
  
%   fprintf('extend C\n')
  
  % check to see if new node connects directly to end_node
  if ((collision(new_node,end_node, R, map)==0) )
    flag = 1;
    new_tree(end,7)=1;  % mark node as connecting to end.
    tmp = end_node(1:3)-new_node(1:3);
    chi = atan2(tmp(2), tmp(1));
    end_node(4) = chi;
  else
    flag = 0;
  end
  
end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% findMinimumPath
%%   find the lowest cost path to the end node
function path = findMinimumPath(tree,end_node)
    
    % find nodes that connect to end_node
    connectingNodes = [];
    for i=1:size(tree,1),
        if tree(i,7)==1,
            connectingNodes = [connectingNodes; tree(i,:)];
        end
    end

    % find minimum cost last node
    [tmp,idx] = min(connectingNodes(:,5));

    
    % construct lowest cost path
    path = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx,6);
    while parent_node>1,
        parent_node = tree(parent_node,6);
        path = [tree(parent_node,:); path];
    end
    
end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% smoothPath
%%   smooth the waypoint path 
function newPath = smoothPath(path, R, map)

    newPath = path(1,:); % add the start node 
    ptr =2;  % pointer into the path
    while ptr <= size(path,1)-1,
        if collision(newPath(end,:), path(ptr+1,:), R, map)~=0, % if there is a collision
            newPath = [newPath; path(ptr,:)];  % add previous node
        end
        ptr=ptr+1;
    end
    newPath = [newPath; path(end,:)];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotmap
%%   plot obstacles and path
function plotmap(map,path,smoothedPath,tree)
  
    % setup plot
    figure(3), clf
    axis([0,map.width,0,map.width,0,2*map.MaxHeight]);
    xlabel('E')
    ylabel('N')
    zlabel('h')
    hold on
  
    % plot buildings 
    V = [];
    F = [];
    patchcolors = [];
    count = 0;
    for i=1:map.NumBlocks,
        for j=1:map.NumBlocks,
            [Vtemp,Ftemp,patchcolorstemp] = buildingVertFace(map.buildings_n(i),...
                map.buildings_e(j),map.BuildingWidth,map.heights(j,i));
            V = [V; Vtemp];
            Ftemp = Ftemp + count;
            F = [F; Ftemp];
            count = count + 8;
            patchcolors = [patchcolors;patchcolorstemp];
        end
    end
  
    patch('Vertices', V, 'Faces', F,...
                     'FaceVertexCData',patchcolors,...
                     'FaceColor','flat');
   
    % draw tree
    for i=2:size(tree,1),
        X = [tree(i,1), tree(tree(i,6),1)];
        Y = [tree(i,2), tree(tree(i,6),2)];   
        Z = [tree(i,3), tree(tree(i,6),3)];            
        plot3(Y,X,-Z,'g')
    end
  
    % draw path
    X = path(:,1);
    Y = path(:,2);
    Z = path(:,3);
    plot3(Y,X,-Z,'r','linewidth',2);

    % draw smooth path
    X = smoothedPath(:,1);
    Y = smoothedPath(:,2);
    Z = smoothedPath(:,3);
    plot3(Y,X,-Z,'k','linewidth',2);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% buildingVertFace(x,y,width,height)
%%   define patches for a building located at (x,y)
function [V,F,patchcolors] = buildingVertFace(n,e,width,height)
 
  % vertices of the building
  V = [...
        e+width/2, n+width/2, 0;...
        e+width/2, n-width/2, 0;...
        e-width/2, n-width/2, 0;...
        e-width/2, n+width/2, 0;...
        e+width/2, n+width/2, height;...
        e+width/2, n-width/2, height;...
        e-width/2, n-width/2, height;...
        e-width/2, n+width/2, height;...
        ];    
  % define faces of fuselage
  F = [...
        1, 4, 8, 5;... % North Side
        1, 2, 6, 5;... % East Side
        2, 3, 7, 6;... % South Side
        3, 4, 8, 7;... % West Side
        5, 6, 7, 8;... % Top
        ];   

  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1,1,0];
  mymagenta   = [0, 1, 1];

  patchcolors = [...
    mygreen;... % North
    mygreen;... % East
    mygreen;... % South
    mygreen;... % West
    myyellow;...  % Top
    ];

end

    
