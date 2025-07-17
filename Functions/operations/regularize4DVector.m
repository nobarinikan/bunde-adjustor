% In: 4x1 Homogenous Vector
% Out: 3x1 Heterogenous Vector
function heterogeneousVector  = regularize4DVector(v)
    heterogeneousVector  = [v(1)/v(4); v(2)/v(4); v(3)/v(4);];
end