% Implementation of a discrete-time state estimator for stereo Visual 
% Odometry with Bundle Adjustment, using Gauss-Newton state estimator. 
% More specifically it implements Chapter 10.1 of Tim Barfoot's "State
% Estimation for Robotics" book 2nd ed.

clc; clear; close all;
% Barfoot p.166 has both line search and Levenberg-Marquardt
%% Main
addpath('Classes\'); addpath('Functions\ConstantsPersistent\');
addpath('Functions\'); addpath('Functions\operations\'); 
addpath('Functions\modelsAndJacobians');

rng('default')

NumberOfSharedPoints = 3; % Number of landmarks shared between the 
% state/measurement Poses/Frames. (Must be at minimum 3!, since we don't 
% have a motion prior or odometry input)

K = 10; % # of Free Poses/Frames
M = K*NumberOfSharedPoints; % # of Landmarks
K_w_zero = K+1; % # of Frames (T_0 included)

stateSize = K*6+M*3;
% Generate True States (Simulation)
[T_true, L_true, Pose_GroundTruth, numOfTotalMeasurements] ...
    = GenerateCase(K,M,1,NumberOfSharedPoints);  

% Algorithm Parameters
N = 1e2; % Max # of Iterations, was 1e6 (Should be set high if using line 
% search)
i = 1;
upper_limit = 60; % Limit for Divergent Solutions (translation error (m))

% Measurement Noise parameters 
% Create noise parameters to create noisy measurements
isNoisy = true; % Sets whether the generated measurements should be noisy
noise_std_ImagePix = 1 * isNoisy; % Pixels are increased by integers
noise_std_Disparity = 0.3 * isNoisy; % 

% Create noisy measurements
createMeasurements;

% Initiliaze x_op and A,b and G
initiliazeEstimator; 

% Initial Damping Factor for Levenberg-Marquardt Regularization
lambda = 1e-3;

% Gauss-Newton optimizer (It's techinally Levenberg-Marquardt with line 
% search)
while (i<=N)
    J_obj = 0; % Objective Function around operating point x_op
    error_index = 1;
    for k = 1:K_w_zero
        visibleLandmarksFromPoseK = ...
            find(Pose_GroundTruth{k}.landmarkVisibilityVector);
        numOfMeasurementsFromPoseK = length(visibleLandmarksFromPoseK);
        for jIDX = 1:numOfMeasurementsFromPoseK
        % The landmark j, from pose k is visible.

            % Pick the kth Pose and jth Landmark Point
            j = visibleLandmarksFromPoseK(jIDX);
            if k == 1 
                x_op_jk{1} = SE3().T;
            else
                x_op_jk{1} = x_op{k-1}.T; % T_op_k 
            end
            x_op_jk{2} = x_op{K+j}; % p_op_j

            if k ~= 1
                % Indices for G1_jk
                row_start_G1 = 3 * (error_index - 1) + 1;
                row_end_G1 = 3*(error_index);
                col_start_G1 = 6*(k-2)+1;
                col_end_G1 = 6*(k-1);
    
                % Assign G1_jk to G1 
                G1(row_start_G1:row_end_G1 , col_start_G1:col_end_G1) ...
                    = G1_jk(x_op_jk); %ones(3,6);%
            end

            % Indices for G1_jk
            row_start_G2 = 3 * (error_index - 1) + 1;
            row_end_G2 = 3*(error_index);
            col_start_G2 = 3*(j-1)+1;
            col_end_G2 = 3*j;

            % Assign G2_jk to G2
            G2(row_start_G2:row_end_G2 , col_start_G2:col_end_G2) ...
                = G2_jk(x_op_jk); %ones(3,3);%         

            % Generate Covariance Matrix R_jk associated with y_jk               
            R_jk{error_index} = eye(3);%chol(diag([noise_std_ImagePix;noise_std_ImagePix;noise_std_Disparity])); % 3x3 (Constant FOR NOW)

            % Calculate error and objective function around operating point
            e_jk = y_noisy_measurement{j,k} - g(x_op_jk);
            e_y(error_index + 2*(error_index-1) : 3*error_index, 1)...
                = e_jk; % Estimation error around operating point                                   
            
            J_obj = J_obj + (1/2)*(e_jk'*inv(R_jk{error_index})*e_jk);

            % Index related with the jk'th landmarks observation.
            error_index = error_index + 1;    
        end
    end

    R = blkdiag(R_jk{:});
    G = [G1 G2];
    A = G'*inv(R)*G;
    b = G'*inv(R)*e_y;
    
    % ---- Levenberg-Marquardt Regularization ----
    A_LM = A + lambda * eye(size(A));  % Damped version of A
    % Solve the system with LM regularization
    x_delta = A_LM \ b; 
    % --------------------------------------------


    % % Solve the linear system (Non Levenberg-Marquardt)
    % x_delta = A\b;
     
    % Check for NaN, Inf, or singularity issues
    if any(isnan(x_delta)) || any(isinf(x_delta)) || rcond(A) < 1e-20
        error('Estimator stopped: Solution contains NaN/Inf or matrix is near singular.');
        break;
    end
    
    % ---- Start Line Search ----
    n_values = 0:1:20;
    % n_values = linspace(1, 0, 20);
    J_obj_min = J_obj; % The original J is J_obj, min_J should be less than it.
    alpha_best = 0; 

    for n = n_values
        % Temporary update with alpha scaling
        % n_used = n-1;
        alpha = (1/2)^(n);
        x_op_temp = updateState(x_op, alpha * x_delta, K, M);
        
        % Recalculate J for this alpha
        J_temp = computeObjectiveFunction(x_op_temp, y_noisy_measurement, R_jk,Pose_GroundTruth);
        
        % Check if this alpha gives a better J
        if J_temp < J_obj_min
            J_obj_min = J_temp;
            alpha_best = alpha;
        end
    end
   
    %---- End Line Search ----

    % % Adaptive(optional) Levenberg-Marquardt Regularization adjust lambda (damping factor)
    % if best_alpha == 1
    %     lambda = lambda / 10;  % Reduce lambda if convergence is good
    % else
    %     lambda = lambda * 10;  % Increase lambda if convergence is slow
    % end

    % Apply the update with the best alpha
    x_op = updateState(x_op, alpha_best * x_delta, K, M);

    % -------------------- History --------------------
    % Objective Function History
    J_obj_hist(i,:) = J_obj_min;
    alpha_hist(i,:) = alpha_best;

    % Compute translation error for the last frame (K)
    translation_true_last = T_true{K}.T(1:3, 4);  % Last true translation
    translation_estimated_last = x_op{K}.T(1:3, 4);  % Last estimated translation (x_op{k} is T_estimated)
    
    % Compute Euclidean distance (translation error)
    translation_error = norm(translation_true_last - translation_estimated_last);
    
    % Store the error at iteration i
    translation_error_hist(i) = translation_error;
    
    % Average errors over the K poses, in the iteration i
    avg_t_error_i = 0;
    avg_C_orientation_error_i = zeros(3);
    avg_eul_orientation_error_i = zeros(3,1);
    for k = 1:K
        avg_t_error_i = avg_t_error_i + x_op{k}.t- Pose_GroundTruth{k+1}.T.t;
        avg_C_orientation_error_i = x_op{k}.SO3_element_of_SE3.C * (Pose_GroundTruth{k+1}.T.SO3_element_of_SE3.C)'; % T_estimated * T_groundtruth'
        avg_eul_orientation_error_i = avg_eul_orientation_error_i + rotm2eul(avg_C_orientation_error_i,'ZYX')';
    end
    avg_translation_error_hist(i,:) = (1/K) * avg_t_error_i;
    avg_eul_orientation_error_hist(i,:) = (1/K) * avg_eul_orientation_error_i;

    % Check Max. # of iterations
    if i == N
        T_estimated = x_op(1:K);
        L_estimated = x_op(K+1:K+M)';
        disp('Max number of iterations is reached')
        break; % FAILURE
    end
    % % Check Divergence Criteria
    if  norm(avg_translation_error_hist(i,:)) > upper_limit 
        T_estimated = x_op(1:K);
        L_estimated = x_op(K+1:K+M);
        fprintf('Failure: Solution diverges');
        break; % FAILURE
    end
     if alpha_best == 0
        T_estimated = x_op(1:K);
        L_estimated = x_op(K+1:K+M);
        fprintf('SUCCESS: Solution converges in iteration i = %i. \n', i);
         break; % SUCCESS
    end
    % % Check Convergence Criteria
    % if alpha_best == 0
    %     T_estimated = x_op(1:K);
    %     L_estimated = x_op(K+1:K+M);
    %     fprintf('Success: Solution converges in iteration i = %i. \n', i);
    %     break; % SUCCESS
    % end
    i=i+1;
end

% % Set RNG to default :)
% rng('default');

%% Plot Translation Error Over Iterations
% Trim unused entries (in case of early convergence)
translation_error_hist = translation_error_hist(1:i);
% 
% % Plot translation error of last pose over iterations
% figure;
% plot(1:i, translation_error_hist, '--');
% xlabel('Iteration');
% ylabel('Translation Error (Euclidean Distance) [m]');
% title('Translation Error [m] (Last Frame) Over Iterations');
% grid on;

% Plot average errors over iterations
figure;
plot(1:i, avg_translation_error_hist(:,1), 'r-', ...  % x in red
     1:i, avg_translation_error_hist(:,2), 'g-', ...  % y in green
     1:i, avg_translation_error_hist(:,3), 'b-');     % z in blue
xlabel('Iteration');
ylabel('Translation Error [m]');
title('Average Translation [m] Error Over Iterations');
legend('x', 'y', 'z');
grid on;

figure;
plot(1:i, avg_eul_orientation_error_hist(:,1), 'r-', ...  % phi in red
     1:i, avg_eul_orientation_error_hist(:,2), 'g-', ...  % theta in green
     1:i, avg_eul_orientation_error_hist(:,3), 'b-');     % psi in blue
xlabel('Iteration');
ylabel('Euler Angles Orientation Error [rad] (ZYX)');
title('Average Orientation Error [rad] Over Iterations');
legend('\phi', '\theta', '\psi');
grid on;

% Plot obj function over iterations
figure;
plot(1:i, J_obj_hist, '-');
xlabel('Iteration');
ylabel('Objective Function Value');
title('Objective Function Over Iterations');
grid on;

% Plot alpha over iterations
figure;
plot(1:i, alpha_hist, '-');
xlabel('Iteration');
ylabel('alpha Function Value');
title('alpha Function Over Iterations');
grid on;




%% Plot The Estimated State vs. Ground Truth 
visualize_estimations(T_estimated, T_true, L_estimated, L_true);