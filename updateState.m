function x_op_new = updateState(x_op, x_delta, K, M)
    % Create a copy of the state to update
    x_op_new = x_op;

    % Update the transformations for each frame (k)
    for k = 1:K
        % Extract the 6x1 optimal pose pertubration
        poseIndex1 = (k - 1) * 6 + 1;
        poseIndex2 = k * 6;
        eps_k = x_delta(poseIndex1:poseIndex2);

        % Update landmark operating point T_op_k
        T_delta_k = SE3(eps_k);  % SE(3) update
        x_op_new{k} = T_delta_k * x_op{k};
    end

    % Update the landmarks for each landmark (j)
    for j = 1:M
        % Extract the 3x1 optimal landmark pertubration
        landmarkIndex1 = K*6+(j-1)*3 + 1;
        landmarkIndex2 = K*6+j*3;
        zeta_j = x_delta(landmarkIndex1:landmarkIndex2);

        % Update landmark operating point p_op_j
        x_op_new{K + j} = x_op{K + j} + D*zeta_j;
    end
end
