function [ output ] = inHalfSpace( p, r, n )
%returns a boolean value of whether the point, p, is in the half space
%defined by r, and n.
%   r = a point in R3 in the plane, n is the normal vector pointing in the
%   direction of the half space

    if (p - r)'*n >= 0
        output = 1; 
    else
        output = 0;
    end
    
end