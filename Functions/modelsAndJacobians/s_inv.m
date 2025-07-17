% Nonlinear Left Camera-Based Stereo Camera Model
% p is 4x1 Homogenous Landmark
function p = s_inv(input)
    u_l = input(1); % pixel in u, 
    v_l = input(2); % pixel in v
    d = input(3);% disparity
    
    f_u = Constants.intrinsics(1);
    f_v = Constants.intrinsics(2);
    c_u = Constants.intrinsics(3);
    c_v = Constants.intrinsics(4);
    b = Constants.intrinsics(5);

    % Recover depth (z) from disparity
    z = f_u * b / d;
    
    % Recover x and y using the 2D coordinates and the depth (z)
    x = (u_l - c_u) * z / f_u;
    y = (v_l - c_v) * z / f_v;
    
    % Reconstructed Homogenous 3D point
    p = [x; y; z; 1];
end

