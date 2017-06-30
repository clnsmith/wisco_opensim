function [y] = smooth_step(x,low_limit,high_limit)
%Implements a smooth step from y=0 to y=1 over the range x=low_limit to 
%x=high_limit
%==========================================================================
if ( x < low_limit)
    y = 0;
elseif (x > high_limit)
    y=1;
else
    scale_x = x- low_limit/(high_limit-low_limit);
    y = 6*scale_x^5-15*scale_x^4+10*scale_x^3;
end 

