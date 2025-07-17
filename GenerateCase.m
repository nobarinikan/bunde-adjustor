function [T, L, PoseVector, totalMeasurements] = GenerateCase(K, M, caseType, sp)
    % K: Number of frames (without T_0)
    % M: Number of landmarks
    % caseType: Select the case for landmark sharing (1 or 2)
    
    PoseVector = cell(K+1, 1);
    totalMeasurements = 0;  % Initialize total number of measurements
    
    % Number of shared points for each case
    num_SP = sp;  % Number of shared landmarks between consecutive frames (Case 1)
    shared_landmarks_first_pose = sp;  % Number of shared landmarks with the first pose (Case 2)

    % Generate transformations between the frames (T_k,G)
    for k = 1:K      
        % Random rotation and translation
        C_delta =SO3(pi/8,[0; -0.3; 1]);%SO3(pi/16,[0; -0.15; 1]);
        t_delta = [2.5;0;0]; % rand(3,1) * 2 - 0;%Translation: random between [-1, 1]
        T_delta = SE3(C_delta, t_delta);  % T_k+1,k
        
        if k == 1
            T{k} = T_delta * SE3();  % SE(3) transformation matrix AFTER T_0
        else
            T{k} = T_delta * T{k-1};  % SE(3) transformation matrix T_k,G
        end
    end

    % Case 1: Consecutive frames share points
    if caseType == 1
        for k = 1:K+1
            landmarkVisibilityVector_for_pose_k = false(M, 1);          
            if k == 1
                % First pose: num_SP true values at the beginning
                landmarkVisibilityVector_for_pose_k(1:num_SP) = true;
                PoseVector{1} = Pose(SE3(), 0, M); 
                PoseVector{1}.setVisibilityVector(landmarkVisibilityVector_for_pose_k);
            elseif k == K+1
                % Last pose: num_SP true values at the end
                landmarkVisibilityVector_for_pose_k(M-num_SP+1:M) = true;
                PoseVector{k} = Pose(T{k-1}, k-1, M);
                PoseVector{k}.setVisibilityVector(landmarkVisibilityVector_for_pose_k);
            else
                % Intermediate poses: 2*num_SP true values
                landmarkVisibilityVector_for_pose_k((k-2)*num_SP + 1 : k*num_SP) = true;
                PoseVector{k} = Pose(T{k-1}, k-1, M);
                PoseVector{k}.setVisibilityVector(landmarkVisibilityVector_for_pose_k);
            end
            
            % Update total number of measurements
            totalMeasurements = totalMeasurements + PoseVector{k}.numMeasurements;
        end

    % Case 2: First pose shares points with every other pose
    elseif caseType == 2
        % Initialize the first pose
        landmarkVisibilityVector_for_first_pose = false(M, 1);
        shared_points_counter = 1;
        for k = 1:K+1
            landmarkVisibilityVector_for_pose_k = false(M, 1);

            if k == 1
                % First pose sees k * number_of_shared_landmarks
                for kk = 2:K+1
                    landmarkVisibilityVector_for_first_pose(shared_points_counter:shared_points_counter + shared_landmarks_first_pose - 1) = true;
                    shared_points_counter = shared_points_counter + shared_landmarks_first_pose;
                end
                PoseVector{1} = Pose(SE3(), 0, M);
                PoseVector{1}.setVisibilityVector(landmarkVisibilityVector_for_first_pose);

            else
                % Each subsequent pose shares shared_landmarks_first_pose points with T_0
                start_index = (k-2) * shared_landmarks_first_pose + 1;
                if k ~=2
                    start_index = start_index-1;
                end
                end_index = (k-1) * shared_landmarks_first_pose;
                landmarkVisibilityVector_for_pose_k(start_index:end_index) = true;
                PoseVector{k} = Pose(T{k-1}, k-1, M);
                PoseVector{k}.setVisibilityVector(landmarkVisibilityVector_for_pose_k);
            end
            
            % Update total number of measurements
            totalMeasurements = totalMeasurements + PoseVector{k}.numMeasurements;
        end
    else
        error('Invalid caseType. Choose either 1 or 2.');
    end

    % Generate landmarks in positive Z-axis (observable region)
    % L_G = generateNonCollinearPoints(M);
    for j = 1:M
        % Random landmarks in front of the camera (positive Z)
        L_G{j} = [randn(1,1) * 2 + 4;  % X in [
                    randn(1,1) * 3 + 6;  % Y in [-2, 2]
                    randn(1,1) * 1 + 8;  % Z in [0.5, 5] (positive)
                    1];  % Homogeneous coordinate

    end

    % Compute landmark locations for each frame 
    for k = 1:K+1
        for j = 1:M
            if k == 1
                L{j,k} = eye(4) * L_G{j};  % Landmark j in 1st frame % Stereo camera measurement of Landmark j in the 1st k
            else
                L{j,k} = T{k-1} * L_G{j};  % Landmark j in frame k
            end
        end
    end
    
end


%%
% % Define random waypoints for x and y (for spline interpolation)
%     numWaypoints = ceil(K / 4);  % Use fewer waypoints than frames for smooth paths
%     waypoints_x = cumsum(randn(numWaypoints, 1));  % Random cumulative movement in x
%     waypoints_y = cumsum(randn(numWaypoints, 1));  % Random cumulative movement in y
% 
%     % Create spline interpolation over the frames
%     frameIndices = linspace(1, K, numWaypoints);
%     x_spline = spline(frameIndices, waypoints_x, 1:K);  % Interpolated x-values
%     y_spline = spline(frameIndices, waypoints_y, 1:K);  % Interpolated y-values
% 
%     % Initialize cell array for poses
%     T = cell(K, 1);
% 
%     % Loop through each frame and generate SE(3) transformations
%     for k = 1:K
%         % Translation with small z-noise
%         t_k = [x_spline(k); y_spline(k); 0.1 * randn()];  
% 
%         % Small random yaw rotation (around z-axis)
%         theta = pi / 16 * randn();  % Small random angle
%         C_k = SO3(theta, [0; 0; 1]);  % Rotation around z-axis
% 
%         % Create SE(3) transformation
%         T{k} = SE3(C_k, t_k);  % SE(3) transformation matrix T_k,k-1
% 
%         % Compose with the previous pose
%         if k > 1
%             T{k} = T{k} * T{k-1};  % Accumulate transformations
%         end
%     end
