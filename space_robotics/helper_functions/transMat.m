function [ transMat ] = transMat( rotMat, transLat)
transMat = [ rotMat, transLat;
            0, 0, 0,        1];
end

