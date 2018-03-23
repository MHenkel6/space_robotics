classdef dhLink < handle
    %DHLINK Link for DH-convetion robot kinematics
    
    properties
        A_function;     % parametric transformation matrix of joint
        A_current;      % current transformation state of joint
        q;              % current joint variable state
        d_q;            % joint variable zero-offset
        type;           % rotational (R) or prismatic (P)
    end
    
    methods
        function self = dhLink(...
                a, ...
                d, ...
                alpha, ...
                theta, ...
                type, ...
                delta_q)
            
            self.d_q  = delta_q;
            self.type = type;
            
            switch type
                case 'R' % rotational joint
                    theta  = theta + delta_q;
                    self.q = theta;
                    self.A_function = @(q) ...
                        [cos(q), -sin(q)*cos(alpha),  sin(q)*sin(alpha), a*cos(q);
                         sin(q),  cos(q)*cos(alpha), -cos(q)*sin(alpha), a*sin(q);
                              0,         sin(alpha),         cos(alpha),        d;
                              0,                  0,                  0,        1];
                case 'P' % prismatic joint
                    d      = d + delta_q;
                    self.q = d;
                    self.A_function = @(q) ...
                        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                                  0,             sin(alpha),             cos(alpha),            q;
                                  0,                      0,                      0,            1];
                otherwise
                    error('Error\nType of joint must be ''R'' (rotational) or ''P'' (prismatic)')
            end
            self.A_current = self.A_function(self.q);
        end
        
        function update(self, q_new)
            self.q = q_new + self.d_q;
            self.A_current = self.A_function(self.q);
        end
    end
end

