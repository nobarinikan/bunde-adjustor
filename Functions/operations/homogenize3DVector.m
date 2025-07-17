% In: 3x1 Heteregenous Vector
% Out: 4x1 Homogenous Vector
function homogenousVector = homogenize3DVector(v)
    homogenousVector = [v; 1];
end