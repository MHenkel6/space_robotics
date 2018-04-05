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
            % pre-compute trig functions for some speed benefit
            c_alpha = cos(alpha);
            s_alpha = sin(alpha);
            c_theta = cos(theta);
            s_theta = sin(theta);
            
            switch type
                case 'R' % rotational joint
                    theta  = theta + delta_q;
                    self.q = theta;
                    self.A_function = @(s_q, c_q) ...
                        [c_q, -s_q*c_alpha,  s_q*s_alpha, a*c_q;
                         s_q,  c_q*c_alpha, -c_q*s_alpha, a*s_q;
                           0,      s_alpha,      c_alpha,     d;
                           0,            0,            0,     1];
                    self.A_current = self.A_function(sin(self.q), cos(self.q));
                case 'P' % prismatic joint
                    d      = d + delta_q;
                    self.q = d;
                    self.A_function = @(q) ...
                        [c_theta, -s_theta*c_alpha,  s_theta*s_alpha, a*c_theta;
                         s_theta,  c_theta*c_alpha, -c_theta*s_alpha, a*s_theta;
                               0,          s_alpha,          c_alpha,         q;
                               0,                0,                0,         1];
                    self.A_current = self.A_function(self.q);
                otherwise
                    error('Error\nType of joint must be ''R'' (rotational) or ''P'' (prismatic)')
            end
        end
        
        % update internal parameters
        function update(self, q_new)
            self.q = q_new + self.d_q;
            switch self.type
                case 'R'
                    self.A_current = self.A_function(sin(self.q), cos(self.q));
                case 'P'
                    self.A_current = self.A_function(self.q);
            end
        end
        
        % calculate A without updating internals
        function A_out = calcA(self, q_new)
            q_new = q_new + self.d_q;
            switch self.type
                case 'R'
                    A_out = self.A_function(sin(q_new), cos(q_new));
                case 'P'
                    A_out = self.A_function(q_new);
            end
        end
        
    end
end
