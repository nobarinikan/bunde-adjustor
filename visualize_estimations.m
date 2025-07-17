function visualize_estimations(T_estimated, T_true, L_estimated, L_true)
    % Define colors and markers
    markerSize = 5;
    frameSize = 1; % size of the frame axis
    figure; hold on; grid on; axis equal;

    % Plot the first frame (T_0 = eye(4)) for clarity
    T_0 = eye(4);
    plot_frame(T_0, frameSize, 'true'); % Initial frame

    % Visualize the transformation matrices
    for k = 1:length(T_true)
        % Plot the true transformation frame in blackish colors
        plot_frame(T_true{k}.T, frameSize, 'true'); % True frame
        % Plot the estimated transformation frame with colored axes
        plot_frame(T_estimated{k}.T, frameSize, 'estimated'); % Estimated frame
    end

    % Visualize the landmark points
    for i = 1:length(L_true)
         % Visualize the landmark points
        for i = 1:length(L_true)
            % Plot true landmark as crosses
            plot3(L_true{i}(1), L_true{i}(2), L_true{i}(3), 'kx', 'MarkerSize', markerSize);
            % Plot estimated landmark as points
            plot3(L_estimated{i}(1), L_estimated{i}(2), L_estimated{i}(3), 'ro', 'MarkerSize', markerSize);
        end
    end
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Visualization of Transformations and Landmarks');
    hold off;
end

% Helper function to plot a frame with different styles for true and estimated
function plot_frame(T, frameSize, frameType)
    % Extract the origin
    origin = T(1:3, 4);
    
    % Define axis vectors (columns of rotation matrix)
    x_axis = T(1:3, 1) * frameSize; % X-axis
    y_axis = T(1:3, 2) * frameSize; % Y-axis
    z_axis = T(1:3, 3) * frameSize; % Z-axis

    % Choose colors based on the type of frame (true or estimated)
    if strcmp(frameType, 'true')
        % True frame with shades of gray
        quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 'Color', [0.6, 0.6, 0.6], 'LineWidth', 2); % X-axis in dark gray
        quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 'Color', [0.4, 0.4, 0.4], 'LineWidth', 2); % Y-axis in lighter gray
        quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 'k', 'LineWidth', 2); % Z-axis in black
    else
        % Estimated frame with red, green, blue colors
        quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2); % X-axis in red
        quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2); % Y-axis in green
        quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2); % Z-axis in blue
    end
end


