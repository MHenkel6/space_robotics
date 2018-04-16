function [ X ] = cartLineEqual( Pstart,Pend,dt, vp )
% Interpolate a linear path in cartesian space based on initial and end position  
%  [x,y,z] and orientation angles [theta,phi,psi]
% Output:
% X = cartesian 3 x n vector of position
%% Setup
Length = norm(Pend-Pstart);
tarray = 0:dt:round(Length/vp/dt)*dt;
X = zeros(size(tarray,2),3);
%% Position
punit = (Pend-Pstart)/norm(Pend-Pstart);
%% Find position and orientation as function of time
X=Pstart+vp*tarray.*punit;
end


