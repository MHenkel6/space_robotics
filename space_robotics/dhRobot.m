classdef dhRobot < handle
    %DHROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        links;      % dhLink classes from base to end effector 
        n;          % number of links
        q;          %
        ATotal;     %
        
        % DH parameters
        a;
        d;
        alpha;
    end
    
    %% Public methods
    
    methods
        %% Constructor
        function self = dhRobot(...
                a, ...
                d, ...
                alpha, ...
                theta, ...
                type, ...
                delta_q)
            
            self.n = length(a);
            self.ATotal = cell(1, self.n);
            self.a = a;
            self.d = d;
            self.alpha = alpha;
            for ii = 1:self.n
                self.links = [self.links, dhLink(a(ii), d(ii), alpha(ii), theta(ii), type(ii), delta_q(ii))];
                self.ATotal{ii} = transMatPost({self.links(1:ii).A_current});
            end
            self.q = zeros(1, self.n);
        end
        
        
        %% Get current position of link N
        function P = getPos(self, n)
            P = self.ATotal{n}(1:3,4);
        end
        
        
        %% Get current rotation matrix of link N
        function C = getRot(self, n)
            C = self.ATotal{n}(1:3,1:3);
        end
        
        
        %% Update DH parameters of all the links
        function updateParams(self, q_new)
            self.q = q_new;
            for ii = 1:self.n
                self.links(ii).update(q_new(ii));
                self.ATotal{ii} = transMatPost({self.links(1:ii).A_current});
            end
        end
        
        %% Forward kinematics (don't save to internal memory)
        function A_out ...   [4x4] homogenous transform for link ii
               = forwardKinematics(self, ...
                 q_new, ...  [1x6] [rad] list of DH parameters
                 index ...   [1-n] desired link index for output
                 )
            A_list = cell(1, index); % initialize cell array of DH matrices
            for ii = 1:index
                A_list{ii} = self.links(ii).calcA(q_new(ii));
            end
            A_out = transMatPost(A_list);
        end
        
        %% Inverse Kinematics calculation
        function theta ... [1x6] [rad] joint parameters to reach desired state
              = inverseKinematics(self, ...
                P, ... [3x1] [mm] position vector
                R, ... [3x3] rotation matrix
                config ... [1-8] desired joint configuration
                )
            A = self.a;
            D = self.d;
            % go from endpoint to wrist point
            W = P - R * [0; 0; D(6)];


            % two solutions for first angle, each with four sub-solutions
            theta = zeros(6,8);
            theta(1, 1:4) = atan2(W(2), W(1));
            theta(1, 5:8) = theta(1, 1) - pi;


            % planar RR link part of solution
            L1 = A(2);
            L2 = sqrt(A(3)^2 + D(4)^2);
            phi = atan(D(4) / A(3)); % angle between x2-axis and line from 02 to wrist
            for ii = 1:2
                if ii==1
                    Vx = sqrt(W(1)^2 + W(2)^2) - A(1);
                else
                    Vx = sqrt(W(1)^2 + W(2)^2) + A(1);
                end
                ind = (1+4*(ii-1)):(4*ii);
                Vy = W(3) - D(1);
                disp(Vy)
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
                AWrist = self.forwardKinematics(theta(:,ii), 3);
                RWrist = AWrist(1:3,1:3);
                Rt = RWrist' * R;
                % solve for euler angles
                theta(4, ii)   = atan2(Rt(2,3), Rt(1,3));
                theta(4, ii+1) = atan2(-Rt(2,3), -Rt(1,3));

                theta(5, ii)   = atan2(sqrt(Rt(1,3)^2 + Rt(2,3)^2), Rt(3,3));
                theta(5, ii+1) = atan2(-sqrt(Rt(1,3)^2 + Rt(2,3)^2), Rt(3,3));

                theta(6, ii)   = atan2(Rt(3,2), -Rt(3,1));
                theta(6, ii+1) = atan2(-Rt(3,2), Rt(3,1));
            end
            
            % select desired configuration
            theta = theta(:, config)';
            disp(self.forwardKinematics(theta, 6))
            if any(isnan(theta))
                errordlg('Desired location not possible')
                theta = [];
            end
        end
    end
end

