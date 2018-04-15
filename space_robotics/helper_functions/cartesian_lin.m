function [ X,vX,aX, tarray, motion_possible ] = cartesian_lin( Pstart,Pend,Rstart,Rend,t ,dt, vp, vomega, )
% Interpolate a linear path in cartesian space based on initial and end position  
%  [x,y,z] and orientation angles [theta,phi,psi]
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

sp = zeros(size(tarray,2),1);
vsp = zeros(size(tarray,2),1);
asp = zeros(size(tarray,2),1);
sr = zeros(size(tarray,2),1);
vsr = zeros(size(tarray,2),1);
asr = zeros(size(tarray,2),1);

%% Position
punit = (Pend-Pstart)/norm(Pend-Pstart);
    vmaxp = vp;
    if vp>2*norm(Pend-Pstart)/t
        vmaxp = 2*norm(Pend-Pstart)/t;
    end
    alphap =vmaxp^2/(vmaxp*t-norm(Pend-Pstart));
    tbp = (vmaxp*t-norm(Pend-Pstart))/vmaxp;
%% Orientation
runit = (Rend-Rstart)/norm(Rend-Rstart);
    vmaxr = vomega;
    if vomega>2*norm(Rend-Rstart)/t
        vmaxr = 2*norm(Rend-Rstart)/t;
    end
    alphar =vmaxr^2/(vmaxr*t-norm(Rend-Rstart));
    tbr = (vmaxr*t-norm(Rend-Rstart))/vmaxr;

    if tbr < 0
        motion_possible = false;
    elseif tbp < 0 
        motion_possible = false;
    end
%% Adjust blend times 
if tbr < tbp
    tbr = tbp;
    vmaxr = norm(Rend-Rstart)/(t-tbr);
    alphar =vmaxr^2/(vmaxr*t-norm(Rend-Rstart));
elseif tbp<tbr
    tbp= tbr;
    vmaxp = norm(Pend-Pstart)/(t-tbp);
    alphar =vmaxp^2/(vmaxp*t-norm(Pend-Pstart));
end

%% Find position and orientation as function of time
if motion_possible
    tbindex = round(tbr/dt); % blend times are equal, so any one can be used for index calculation
    tbround = tbindex*dt; % rounded blend time
    %Position, position velocity and acceleration
    sp(1:tbindex) = 0.5*alphap*tarray(1:tbindex).^2;
    vsp(1:tbindex) = alphap*tarray(1:tbindex);
    asp(1:tbindex) = alphap*ones(tbindex,1);
    %Orientation, orientation velocity and acceleration
    sr(1:tbindex) = 0.5*alphar*tarray(1:tbindex).^2;
    vsr(1:tbindex) = alphar*tarray(1:tbindex);
    asr(1:tbindex) = alphar;
    if tbindex~=(size(X,1)-tbindex)
        sp(tbindex+1:end-tbindex)= alphap*tbround*(tarray(tbindex+1:end-tbindex)-tbround/2);
        vsp(tbindex+1:end-tbindex) = vmaxp*ones(size(tarray,2)-2*tbindex,1);
        asp(tbindex+1:end-tbindex) = 0;
        
        sr(tbindex+1:end-tbindex)= alphar*tbround*(tarray(tbindex+1:end-tbindex)-tbround/2);
        vsr(tbindex+1:end-tbindex) = vmaxr*ones(size(tarray,2)-2*tbindex,1);
        asr(tbindex+1:end-tbindex) = 0;
    end
    sp(end-tbindex+1:end) = norm(Pend-Pstart)-0.5*alphap*(t-tarray((end-tbindex+1):end)).^2;
    vsp(end-tbindex+1:end) = alphap*(t-tarray((end-tbindex+1):end));
    asp(end-tbindex+1:end) = -alphap;
    
    sr(end-tbindex+1:end) =norm(Rend-Rstart)-0.5*alphar*(t-tarray((end-tbindex+1):end)).^2;
    vsr(end-tbindex+1:end) = alphar*(t-tarray((end-tbindex+1):end));
    asr(end-tbindex+1:end) = -alphar;

    X(:,1:3) =sp.*punit;
    vX(:,1:3) =vsp.*punit;
    aX(:,1:3) =asp.*punit;

    X(:,4:6) =sr.*runit;
    vX(:,4:6) =vsr.*runit;
    aX(:,4:6) =asr.*runit;
    end
end

