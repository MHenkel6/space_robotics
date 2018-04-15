function [angle, axis ] = anglevec( vec1,vec2 )
%Determines angle  between two vectors, and the right handed axis 
angle = atanFullCircle(norm(cross(vec1,vec2)),dot(vec1,vec2));
axis = cross(vec1,vec2)/norm(cross(vec1,vec2));
end

