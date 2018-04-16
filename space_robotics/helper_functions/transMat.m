function [ transMat ] = transMat( rotMat, transLat)
% Create transformation matrix from rotation matrix and translation vector
transMat = [ rotMat, transLat;
            0, 0, 0,        1];
end

