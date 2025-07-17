% In: 6x1 Pose Vector
% Out: 4x4 SE(3) Group group-non-member?
function result = lift(ksi)
    result = [skew(ksi(4:6)) ksi(1:3);
              zeros(1,3) 0];
end