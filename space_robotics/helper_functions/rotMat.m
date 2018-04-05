function [ matrix ]...
  = rotMat(...
    angle, ...
    direction)
    switch direction
        case 'x'
            matrix = [          1,          0,           0;
                                0, cos(angle), -sin(angle);
                                0, sin(angle),  cos(angle)];
        case 'y'
            matrix = [ cos(angle),          0,  sin(angle);
                                0,          1,           0;
                      -sin(angle),          0,  cos(angle)];
        case 'z'
            matrix = [ cos(angle), -sin(angle),           0;
                       sin(angle),  cos(angle),           0;
                                0,           0,           1];
        otherwise
    end
end

