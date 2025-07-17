y_noisy_measurement = cell(M, K_w_zero);

for k = 1:K_w_zero
    for j = 1:M
        % Generate noise for pixel coordinates and disparity
        n_xy = noise_std_ImagePix * randn(2, 1);  % Pixel noise (u, v)
        n_disparity = noise_std_Disparity * randn();  % Disparity noise

        % Stereo camera measurement of Landmark j in frame k, [u_L,v,d]
        L_true_stereoMeasured{j, k} = s(L_true{j, k});

        % Add noise to the measurement
        y_jk = L_true_stereoMeasured{j, k} + [n_xy; n_disparity];

        % Store the noisy measurement
        y_noisy_measurement{j, k} = y_jk;
    end
end
clear y_jk;
