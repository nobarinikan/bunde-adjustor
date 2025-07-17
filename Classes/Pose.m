classdef Pose < handle
    properties
        T  % 4x4 transformation matrix (SE(3))
        k double {mustBeNonnegative} = 0  % Index of the pose among K poses (excluding T_0)
        landmarkVisibilityVector logical  % Mx1 logical array representing visible landmarks
        numMeasurements double {mustBeNonnegative} = 0  % Number of visible landmarks
    end
    methods
        % Constructor to initialize the pose
        function obj = Pose(T, k, M)
            if nargin > 0  % If arguments are passed
                if nargin == 3
                    obj.setgetM(M);
                    obj.T = T;
                    obj.k = k;
                    obj.landmarkVisibilityVector = false(M, 1);
                    obj.numMeasurements = sum(obj.landmarkVisibilityVector);  % Initialize with the number of measurements
                elseif nargin == 2
                    % obj.T = T;
                    % obj.k = k;                  
                end
            else
                error('Pose Initialization is faulty.')
            end
        end

        % Method to set visibility for a landmark individually
        function obj = setVisibilityIndex(obj, j, val)
            obj.landmarkVisibilityVector(j,1) = val;
            obj.numMeasurements = sum(obj.landmarkVisibilityVector);  % Update number of measurements
        end
        
        % Method to set visibility for the whole vector
        function obj = setVisibilityVector(obj, visibilityVector)
            obj.landmarkVisibilityVector = visibilityVector;
            obj.numMeasurements = sum(obj.landmarkVisibilityVector);  % Update number of measurements
        end     

        % Method to check if a landmark is visible from this pose
        function isVisible = isLandmarkVisible(obj, landmarkIndex)
            isVisible = obj.landmarkVisibilityVector(landmarkIndex);
        end
    end

    methods (Static)
        function out = setgetM(data)
            persistent M  % M is the total number of landmarks
            if nargin
                M = data;
            end
            out = M;
        end
    end
end
