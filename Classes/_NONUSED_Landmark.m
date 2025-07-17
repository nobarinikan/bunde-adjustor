classdef Landmark
    properties
        Position      % 4x1 homogeneous vector for the position of the landmark
        Visibility    % Logical array where each entry indicates visibility from a pose
    end
    
    methods
        % Constructor to initialize the landmark with position and visibility
        function obj = Landmark(position, visibility)
            if nargin > 0  % If arguments are passed
                obj.Position = position;
                obj.Visibility = visibility;
            end
        end
        
        % Method to set the visibility from a particular pose
        function obj = setVisibility(obj, poseIndex, isVisible)
            obj.Visibility(poseIndex) = isVisible;
        end
        
        % Method to get whether the landmark is visible from a specific pose
        function isVisible = isVisibleFrom(obj, poseIndex)
            isVisible = obj.Visibility(poseIndex);
        end
        
        % Method to transform the landmark's position by a given transformation matrix
        function transformedPosition = transformByPose(obj, T)
            transformedPosition = T * obj.Position;
        end
    end
end
