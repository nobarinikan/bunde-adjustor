classdef SO3
    properties
        axis = [0;0;0];
        angle = 0;
        C = eye(3); % Rotation Matrix 3x3
    end
    methods
        % **Constructor:
        function obj = SO3(input1, input2)
            if nargin > 0
                % In: 3x1 Axis of rotation and 1x1 Angle of rotation
                % Out: 3x3 Rotation Matrix C
                createRotationMatrix = @(theta, a) cos(theta)*eye(3) + (1-cos(theta))*(normalize3DVector(a(:))*normalize3DVector(a(:))') + sin(theta)*skew(normalize3DVector(a(:)));
                if nargin == 2
                    % Case 1: Input is angle (theta) and axis (a)
                    theta = input1;
                    a = input2;
                    obj.angle = theta;
                    obj.axis = normalize3DVector(a(:));                
                    obj.C = createRotationMatrix(obj.angle,obj.axis);
                    
                elseif nargin == 1
                    % Case 2: Input is a 3x3 rotation matrix
                    if isnumeric(input1) && all(size(input1) == [3, 3])
                        obj.C = input1;
                        % Extract axis and angle from the rotation matrix
                        % [obj.angle, obj.axis] = rotationMatrixToAxisAngle(input1);
                    elseif isnumeric(input1) && (size(input1,1) == 3)
                        obj.angle = minRepToAxisAngle(input1).angle;
                        obj.axis = minRepToAxisAngle(input1).unitAxis;
                        obj.C = createRotationMatrix(obj.angle,obj.axis);
                    end        
                else
                    error('Input must be either an angle and axis rep., or a 3x3 rotation matrix');
                end
                % Check if determinant of C is close to 1 (for proper rotations)
                if abs(det(obj.C) - 1) > 1e-12
                    error('Determinant of the rotation matrix is not equal to 1. Not a valid rotation matrix.');
                end
        
                % Check if C is orthogonal: C * C' should be close to identity matrix
                if norm(obj.C * obj.C' - eye(3), 'fro') > 1e-12
                    error('The rotation matrix is not orthogonal. Not a valid rotation matrix.');
                end
            else
                % disp('Creating SO3 object with identity properties!!!!')
            end 
        end

        % **Overload mtimes (multiplication)
        function result = mtimes(obj, LHS)
            % Case 1: SO3 * SO3
            if isa(LHS, 'SO3')
                % Combine the SO3 rotations and translations
                C_new = obj.C * LHS.C;
                result = SO3(C_new);

            % Case 2: SE3 * 3x1 vector (applying transformation to a point)
            elseif isnumeric(LHS) && size(LHS,1) == 3
                result = obj.C * LHS;              
            else
                error('Unsupported multiplication: SE3 can only be multiplied with SE3 or 4x1 points');
            end
        end
    end
end

% % Extract angle and axis from a 3x3 rotation matrix
% function [angle, axis] = rotationMatrixToAxisAngle(C)
%     angle = acos((trace(C) - 1) / 2);  % Compute the angle from the trace of the matrix
%     axis = 1 / (2 * sin(angle)) * [C(3,2) - C(2,3); C(1,3) - C(3,1); C(2,1) - C(1,2)];  % Extract the rotation axis
%     axis = normalize3DVector(axis);  % Ensure the axis is normalized
% end
