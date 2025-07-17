classdef Constants
    properties (Constant)
        % Stereo Camera Intrinsics
        f_u = 969.4750;  % Focal length in image x [px]
        f_v = 969.4750;  % Focal length in image y [px]
        c_x = 643.6190; % Principal point x-coordinate [px]
        c_y = 481.4160; % Principal point y-coordinate [px]
        b = 0.24; % Baseline distance between cameras [m]

        intrinsics = [969.4750, 969.4750, 643.6190, 481.4160, 0.24];
        
        % Projection Matrix for the left-camera-based stereo 
        P_stereo = [969.4750 0 643.6190 0;
                    0 969.4750 481.4160 0;
                    0 0 0 969.4750 * 0.24];
        
    end
end
