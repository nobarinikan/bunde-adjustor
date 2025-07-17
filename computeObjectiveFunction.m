function J_obj = computeObjectiveFunction(x_op, y_noisy_measurement, R_jk,Pose_GroundTruth)
    J_obj = 0;
    error_index = 1;
    K =  length(Pose_GroundTruth)-1;
    K_w_zero = length(Pose_GroundTruth);
    
    for k = 1:K_w_zero
        visibleLandmarksFromPoseK = find(Pose_GroundTruth{k}.landmarkVisibilityVector);
        for jIDX = 1:length(visibleLandmarksFromPoseK)
            j = visibleLandmarksFromPoseK(jIDX);
            
            % Select the appropriate pose transformation
            if k == 1
                x_op_jk{1} = SE3().T;
            else
                x_op_jk{1} = x_op{k-1}.T;
            end
            x_op_jk{2} = x_op{K + j};

            % Compute error and update objective function J
            e_jk = y_noisy_measurement{j, k} - g(x_op_jk);
            p_G = z(x_op_jk);
            if p_G(3) < 0 
                % Normally landmarks with z < 0 should create larger J's,
                % so they should be never selected. This is just an extra
                % step to make sure.
                % disp('Adding inf in computeObjectiveFunction.')
                J_obj = inf;
                return;
            end
            J_obj = J_obj + (1/2) * (e_jk'*inv(R_jk{error_index})*e_jk);
            error_index = error_index + 1;
        end
    end
end
