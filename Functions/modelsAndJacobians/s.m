% Nonlinear Left Camera-Based Stereo Camera Model
% p is 4x1 Homogenous Landmark
function result = s(p)
    % if ~(p(3)/p(3) == 1 && p(4) == 1)
    %     error('s landmark input problematic')
    % end
    % p_new = p/p(3);
    % result = Constants.P_stereo * p_new;

    f_u = Constants.intrinsics(1);
    f_v = Constants.intrinsics(2);
    c_u = Constants.intrinsics(3);
    c_v = Constants.intrinsics(4);
    b = Constants.intrinsics(5);
    x = p(1);
    y = p(2);
    z = p(3);
    if z < 0 
        disp('Landmark is behind of camera.')
    end

    result = [f_u * x/z + c_u;
              f_v * y/z + c_v;
              f_u * b / z];
end

