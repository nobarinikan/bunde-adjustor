% p is a homogenous (4x1) point, that is represented w.r.t the Priveleged
% Frame
function result = S_jk(p)
    f_u = Constants.intrinsics(1);
    f_v = Constants.intrinsics(2);
    c_u = Constants.intrinsics(3);
    c_v = Constants.intrinsics(4);
    b = Constants.intrinsics(5);

    x = p(1);
    y = p(2);
    z = p(3);

    % Non-Linear Camera Model Jacobian (3x4), calculated around the given
    % operating point
    result = [f_u/z,     0, -(f_u*x)/z^2, 0;
               0, f_v/z, -(f_v*y)/z^2, 0;
               0,     0, -(b*f_u)/z^2, 0];
end

