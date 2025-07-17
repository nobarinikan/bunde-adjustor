% In: 3x1 Axis of rotation and 1x1 Angle of rotation
% Out: 3x3 Rotation Matrix R = C'
function R = createRotationMatrix2(theta, a)
    R = eye(3) + ...
        sin(theta)*skew(normalize3DVector(a(:))) + ...
        (1-cos(theta))*(skew(normalize3DVector(a(:))))^2;
end