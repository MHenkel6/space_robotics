function [ X,vX,aX,tarray ] = cartesian_circ( Pstart,Pend,Rstart,Rend,Pcenter,Rotaxis,righthanded,t ,dt, vp )
% Interpolate a circular arc path in cartesian space based on initial and end position  
%  [x,y,z] and orientation angles [theta,phi,psi], rotation center and
%  axis, and rotation direction
% Output:
% X = cartesian 6xn vector of position and angles 
% vX = cartesian 6xn vector of position and angle velocities
% aX = cartesian 6xn vector of position and angle accelerations
%% Setup
motion_possible = true;

tarray = 0:dt:t;
X = zeros(size(tarray,2),6);
vX = zeros(size(tarray,2),6);
aX = zeros(size(tarray,2),6);

%path variable s
spsi = zeros(size(tarray,2),1);
vspsi = zeros(size(tarray,2),1);
aspsi = zeros(size(tarray,2),1);

%% Determine rotation plane

xprime = (Pstart-Pcenter)/norm(Pstart-Pcenter);
zprime = Rotaxis/norm(Rotaxis);
yprime = cross(zprime,xprime);
x= [1;0;0];
y = [0;1;0];
z = [0;0;1];
R = [dot(xprime,x),dot(xprime,y),dot(xprime,z);dot(yprime,x),dot(yprime,y),dot(yprime,z);dot(zprime,x),dot(zprime,y),dot(zprime,z)];
%% Determine radius and end angle
radius = norm(Pstart-Pcenter);
psistart = 0;%Angle begins at 0
[psiend,axis] =   anglevec(Pstart-Pcenter,Pend-Pcenter);
if (righthanded == -1) && (int8(dot(axis,zprime))==1) %left handed rotation, positive angle
    psiend =psiend-2*pi;
elseif (righthanded == 1) && (int8(dot(axis,zprime))== -1) %left handed rotation, opposite angle is chosen
    psiend =2*pi-psiend;
elseif (righthanded == -1) && (int8(dot(axis,zprime))== -1)
    psiend = -psiend;
elseif (int8(dot(axis,zprime))== 0)
    psiend = righthanded*psiend;
end
%% Interpolate path variable
    vmaxpsi = vp/radius;
    if vp/radius>2*abs(psiend)/t
        vmaxpsi =2*(psiend)/t;
    end
    alphapsi =vmaxpsi^2/(vmaxpsi*t-(psiend-psistart));
    tb = (vmaxpsi*t-psiend)/vmaxpsi;
    if tb < 0 
        motion_possible = false;
    end

%% Find position and orientation as function of time
if motion_possible
    tbindex = round(tb/dt); % blend times are equal, so any one can be used for index calculation
    tbround = tbindex*dt; % rounded blend time
    %Position, position velocity and acceleration
    spsi(1:tbindex) = 0.5*alphapsi*tarray(1:tbindex).^2;
    vspsi(1:tbindex) = alphapsi*tarray(1:tbindex);
    aspsi(1:tbindex) = alphapsi*ones(tbindex,1);
    %Orientation, orientation velocity and acceleration
    if tbindex~=(size(X,1)-tbindex)
        spsi(tbindex+1:end-tbindex)= alphapsi*tbround*(tarray(tbindex+1:end-tbindex)-tbround/2);
        vspsi(tbindex+1:end-tbindex) = vmaxpsi*ones(size(tarray,2)-2*tbindex,1);
        aspsi(tbindex+1:end-tbindex) = 0;      
    end
    spsi(end-tbindex+1:end) = psiend-0.5*alphapsi*(t-tarray((end-tbindex+1):end)).^2;
    vspsi(end-tbindex+1:end) = alphapsi*(t-tarray((end-tbindex+1):end));
    aspsi(end-tbindex+1:end) = -alphapsi;
%% From path variable, give positions in Cartesian space
    Pprime = [radius * cos(spsi),radius * sin(spsi),zeros(size(spsi,1),1)]; % position in O frame
    for ii = 1:1:size(X,1)
        X(ii,1:3) = Pcenter + R' * Pprime(ii,:)'; % Position in World frame:
    end
end
end
