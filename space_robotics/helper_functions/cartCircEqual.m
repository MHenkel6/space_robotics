function [ X ] = cartCircEqual( Pstart,Pend,Pcenter,Rotaxis,righthanded ,dt, vp )
    % Interpolate a circular arc path in cartesian space based on initial and end position  
    %  [x,y,z] and orientation angles [theta,phi,psi], rotation center and
    %  axis, and rotation direction
    % Output:
    % X = cartesian 6xn vector of position
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
    [psiend,axis] =   anglevec(Pstart-Pcenter,Pend-Pcenter);
    if abs(psiend) <1e-10
        psiend =2*pi;
    end
    if (righthanded == -1) && (int8(dot(axis,zprime))==1) %left handed rotation, positive angle
        psiend =psiend-2*pi;
    elseif (righthanded == 1) && (int8(dot(axis,zprime))== -1) %left handed rotation, opposite angle is chosen
        psiend =2*pi-psiend;
    elseif (righthanded == -1) && (int8(dot(axis,zprime))== -1)
        psiend = -psiend;
    elseif (int8(dot(axis,zprime))== 0)
        psiend = righthanded*psiend;
    end
    Length = abs(radius*psiend);
    tarray = 0:dt:round(Length/vp/dt)*dt;
    X = zeros(size(tarray,2),3);
    omega = vp/radius*sign(psiend);
    Pprime = [radius * cos(omega*tarray)',radius * sin(omega*tarray)',zeros(size(tarray,2),1)]; % position in O frame
    for ii = 1:1:size(X,1)
        X(ii,1:3) = Pcenter + R' * Pprime(ii,:)'; % Position in World frame:
    end
    X = X';
end
