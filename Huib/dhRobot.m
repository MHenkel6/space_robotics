classdef dhRobot < handle
    %DHROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        links;      % dhLink classes from base to end effector 
        n;          % number of links
        q;          %
        ATotal;     %
        linkSegments;%
    end
    
    %% Public methods
    
    methods
        % Constructor
        function self = dhRobot(...
                a, ...
                d, ...
                alpha, ...
                theta, ...
                type, ...
                delta_q)
            
            self.n = length(a);
            self.ATotal = cell(1, self.n);
            self.linkSegments = zeros(3, self.n+1);
            for ii = 1:self.n
                self.links = [self.links, dhLink(a(ii), d(ii), alpha(ii), theta(ii), type(ii), delta_q(ii))];
                self.ATotal{ii} = transMatPost({self.links(1:ii).A_current});
                self.linkSegments(:,ii+1) = self.ATotal{ii}(1:3, 4);
            end
        end
        
        % Update DH parameters of all the links
        function updateParams(self, q_new)
            self.q = q_new;
            for ii = 1:self.n
                self.links(ii).update(q_new(ii));
            end
            self.updateJoints();
        end
        
        % Plot all joints 
        function plotJoints(self)
            % plot x-y-z axes at correct position
            plotBase([eye(3), [0;0;0]; [0 0 0 1]], 0.3)
            for ii = 1:self.n
                plotBase(self.ATotal{ii}, 0.2)
            end
            % plot connecting links
            plot3(self.linkSegments(1,:), self.linkSegments(2,:), self.linkSegments(3,:))
        end
    end
    
    %% Private methods
    
    methods (Access = private)
        % Update position & rotation of all joint axes
        function updateJoints(self)
            for ii = 1:self.n
                self.ATotal{ii} = transMatPost({self.links(1:ii).A_current});
                self.linkSegments(:,ii+1) = self.ATotal{ii}(1:3, 4);
            end
        end
    end
end

