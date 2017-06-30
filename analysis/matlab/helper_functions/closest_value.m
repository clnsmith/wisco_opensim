function [val ind] = closest_value(x,Y)
%==========================================================================
%Finds the closest value to x in array Y
%==========================================================================
[~, ind] = min(abs(Y-x));

val = Y(ind);