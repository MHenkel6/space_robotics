%% Inverse kineamtics for planar robotic arm with two rotational joints
clc;
clear;
%% Joint length parameters
a = zeros(1,6);
d = zeros(1,6);
alpha = [pi/2 0 pi/2 pi/2 pi/2 0];
dTheta = [0 pi/2 0 pi pi 0];

a(1) = 150;
d(1) = 525;
a(2) = 790;
a(3) = 250;
d(4) = 835;
d(6) = 100;

forwardKin = dhRobot(a, d, alpha, zeros(1,6), 'RRRRRR', dTheta);

%% Input Point and Rotataion matrix
P = [400, 400, d(1)]';
z_angle = pi;
y_angle = -pi/2;
x_angle = 0;
R       = rotMat(z_angle, 'z')*rotMat(y_angle, 'y')*rotMat(x_angle, 'x');



%% Inverse kinematics

% go from endpoint to wrist point
W = P - R * [0; 0; d(6)];


% two solutions for first angle, each with four sub-solutions
theta = zeros(6,8);
theta(1, 1:4) = atan2(W(2), W(1));
theta(1, 5:8) = theta(1, 1) - pi;


% planar RR link part of solution
L1 = a(2);
L2 = sqrt(a(3)^2 + d(4)^2);
phi = atan(d(4) / a(3)); % angle between x2-axis and line from 02 to wrist
for ii = 1:2
    if ii==1
        Vx = sqrt(W(1)^2 + W(2)^2) - a(1);
    else
        Vx = sqrt(W(1)^2 + W(2)^2) + a(1);
    end
    ind = (1+4*(ii-1)):(4*ii);
    Vy = W(3) - d(1);

    % check if solution is possible
    distance = sqrt(Vx^2 + Vy^2);   % distance from origin to desired point
    if distance > L1+L2 || (L1 ~= L2 && distance < abs(L1-L2))            % no solution
        theta(2, ind) = NaN;
        theta(3, ind) = NaN;
    elseif distance == L1 + L2      % one solution, outer range
        theta(3, ind) = phi;
        theta(2, ind) = atan2(Vy, Vx) - pi/2;
    elseif distance == abs(L1 - L2) % one solution, inner range
        theta(3, ind) = phi + pi;
        if L1 > L2
            theta(2, ind) = atan2(Vy, Vx)  + pi/2;
        else
            theta(2, ind) = atan2(-Vy, -Vx) - pi/2;
        end
    else                            % two solutions
        C_theta3 = (distance^2 - L1^2 - L2^2) / (2 * L1 * L2);
        % elbow up and elbow down solutions
        theta(3, ind(1:2)) = -acos(C_theta3);
        theta(3, ind(3:4)) = -theta(3, ind(1));
        theta(3, ind) = theta(3, ind) + phi;
        
        % corresponding rotations for first joint
        % beta = angle between lines O1-O2 and O1-O3
        beta(1:2) = asin(L2 * sqrt(1 - C_theta3^2) / distance);
        beta(3:4) = -beta(1);
        if (L1^2 + distance^2) < L2^2 % take into account possibility of obtuse angle with sine rule
            beta = [pi pi -pi -pi] - beta;
        end
        theta(2, ind)  = atan2(Vy, Vx) + beta - pi/2;
    end
    % some adjustments for when first link is pointing backwards
    if ii == 2
        theta(2, ind) = theta(2, ind) + pi;
    end
end


% solution for ZYZ-like wrist rotation
for ii = 1:2:7
    % got orientation of O3 axes
    forwardKin.updateParams(theta(:,ii))
    RWrist = forwardKin.ATotal{3}(1:3,1:3);
    Rt = transp(RWrist) * R;
    % solve for euler angles
    theta(4, ii)   = atan2(Rt(2,3), Rt(1,3));
    theta(4, ii+1) = atan2(-Rt(2,3), -Rt(1,3));
    
    theta(5, ii)   = atan2(sqrt(Rt(1,3)^2 + Rt(2,3)^2), Rt(3,3));
    theta(5, ii+1) = atan2(-sqrt(Rt(1,3)^2 + Rt(2,3)^2), Rt(3,3));
    
    theta(6, ii)   = atan2(Rt(3,2), -Rt(3,1));
    theta(6, ii+1) = atan2(-Rt(3,2), Rt(3,1));
end

% clear out any columns with NaN values, i.e. impossible solutions
theta = theta(:, ~any(isnan(theta), 1));

%% send result to spanviewer
[~, n] = size(theta);
udps = dsp.UDPSender;
for jj = 1:n
    forwardKin.updateParams(theta(:,jj))
    udps(theta(:,jj))
    pause(1)
    disp((forwardKin.ATotal{end}(1:3, 4)-P)')
end


fprintf('Finito!\n%d solutions found.\n', n)