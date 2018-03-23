function [ matrix ]...
  = rotMatd(...
    angle, ...
    direction)
    switch direction
        case 'x'
            matrix = [           1,           0,            0;
                                 0, cosd(angle), -sind(angle);
                                 0, sind(angle),  cosd(angle)];
        case 'y'
            matrix = [ cosd(angle),           0,  sind(angle);
                                 0,           1,            0;
                      -sind(angle),           0,  cosd(angle)];
        case 'z'
            matrix = [ cosd(angle), -sind(angle),           0;
                       sind(angle),  cosd(angle),           0;
                                 0,            0,           1];
        otherwise
    end
end

