% *** START Initiliaze State Estimate ***
xhat_init = cell(K+M,1); % Plot: CAN CHANGE ROW SIZE TO REFLECT ITERATIONS (to plot maybe?)

% Initialize Pose Estimates
[xhat_init{1:K}] = deal(SE3()); % Default Initialization
% [xhat_init{K+1:K+M}] = deal(ones(4,1)); % Degenerate Landmark Initialization

% Random initilization
% for j = 1:M
%     % Initialize Homogenized Landmark Point Estimates
%     [xhat_init{K+j}] = [0;0;7;0] + [3*randn(3,1); 1]; 
% end
% for k = 1:K
%     % Initialize Homogenized Landmark Point Estimates
%     [xhat_init{k}] = T_true{k}*SE3(SO3(),[1;0;0]); 
% end

% % Correct Initiliazations
% [xhat_init{1:K}] = deal(T_true{1:K}); % Correct Pose Initializatio
% [xhat_init{K+1:K+M}] = deal(L_true{1:M,1}); % Correct Landmark Initialization
%

% Landmark Initilization (Method used is a wrong method, landmark 
% observations shouldn't be averaged. Hence this method serves the same 
% purpose as a random initilization, which is totally fine. All random 
% landmark initilizations are fine, as long as they are not collinear, then 
% the estimator will break.)
for j = 1:M  % Loop over landmarks
    sum_landmark = zeros(4, 1);  % To accumulate transformed measurements
    num_observations = 0;  % Count how many observations we use

    for k = 1:K_w_zero  % Loop over all poses including the first one (T_0)
        if Pose_GroundTruth{k}.isLandmarkVisible(j)
            % Get the noisy measurement for landmark j from pose k
            y_jk = y_noisy_measurement{j, k};  % 4x1 noisy measurement

            % Use the pose initialization (identity if k=1) to transform measurement
            if k == 1
                T_kG = SE3();  % Identity for the first pose
                T_Gk = T_kG;
            else
                T_kG = xhat_init{k-1};  % Pose initialization for pose k
                T_Gk = pieceWiseInverse(T_kG);
            end

            % Transform the measurement to the ground frame
            y_jk_in_ground_frame = T_Gk * s_inv(y_jk);  % Inverse transformation

            % Accumulate the transformed measurement
            sum_landmark = sum_landmark + y_jk_in_ground_frame;
            num_observations = num_observations + 1;
        end
    end

    % Average the accumulated measurements to initialize the landmark
    if num_observations > 0
        xhat_init{K + j} = sum_landmark / num_observations;  % Averaged initialization
    else
        error('Non-observed Landmark')
        % xhat_init{K + j} = [0; 0; 5; 1];  % Fallback if no observations (unlikely)
    end
end
clear y_jk T_kG y_jk_in_ground_frame

% Set initial operating points as estimates
x_op = xhat_init;

% *** END Initiliaze State Estimate ***

% Allocate size to Linear System Elements
A = zeros(stateSize);
b = zeros(stateSize,1);

% Everything below here must change size with # of total measurements
G = zeros(3*numOfTotalMeasurements, stateSize); % K=4 M=2 G--> (32,24)
R_jk = cell(numOfTotalMeasurements,1);
e_y = zeros(3*numOfTotalMeasurements,1);

G1 = zeros(3*numOfTotalMeasurements, 6*(K_w_zero-1)); 
G2 = zeros(3*numOfTotalMeasurements, 3*M); 

clear landmarkIndex landmarksVisibleFromFirstPose