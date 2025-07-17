classdef SE3 
    properties
        T = eye(4); % The transformation matrix 4x4
        SO3_element_of_SE3 = SO3(); % Member of SO3
        t = zeros(3,1); % Translation vector (3x1)
    end

    methods
        % **Constructor:
        function obj = SE3(input1, input2)
            if nargin > 0
                if nargin == 2
                    % Case 1: Input is an SO3 object and a translation vector                  
                    if isa(input1, 'SO3')  % Check if input is an SO3 object
                        obj.SO3_element_of_SE3 = input1;  % Assign the SO3 object
                        obj.t = input2;  % Set the translation vector
                    else
                        error('The first input must be an SO3 object');
                    end
                    obj.T = [obj.SO3_element_of_SE3.C, obj.t; 0 0 0 1];  % SE3 matrix
    
                elseif nargin == 1
                    % Case 2: Input is a 6x1 ksi vectori
                    if isnumeric(input1) && size(input1,1) == 6
                        ksi = input1;
                        % In: 6x1 Lie Algebra member of se(3)
                        % Out: 4x4 Lie Group member SE(3) 
                        createPieceWiseSE3Matrix = @(ksi) ...
                            [SO3(ksi(4:6)).C, leftJacobian(ksi(4:6)) * ksi(1:3);
                             0 0 0 1];   
                        obj.T = createPieceWiseSE3Matrix(ksi);  % Compute the SE3 matrix
                        % Extract SO3 element and translation vector from the matrix
                        obj.SO3_element_of_SE3 = SO3(obj.T(1:3, 1:3));  % Extract the SO3 rotation
                        obj.t = obj.T(1:3, 4);  % Extract the translation vector
                    end
                else
                    error('Input must be either an SO3 object and a translation vector, or a 6x1 ksi vector');
                end
            else
                % % Default constructor: identity rotation and zero translation
                % disp('!!!!Hey, Creating SE3 object with identity properties!!!!')
            end
        end

        % **Overload mtimes (multiplication)
        function result = mtimes(obj, LHS)
            % Case 1: SE3 * SE3
            if isa(LHS, 'SE3')
                % Combine the SO3 rotations and translations
                T_new = obj.T * LHS.T;
                C_new = SO3(T_new(1:3, 1:3));  % Extract the SO3 rotation
                t_new = T_new(1:3, 4);  % Extract the translation vector
                result = SE3(C_new, t_new);

            % Case 2: SE3 * 4x1 homogeneous point (applying transformation to a point)
            elseif isnumeric(LHS) && size(LHS,1) == 4
                result = obj.T * LHS;              
            else
                error('Unsupported multiplication: SE3 can only be multiplied with SE3 or 4x1 points');
            end
        end

        function invTransform = pieceWiseInverse(obj)
            C_new = SO3((obj.SO3_element_of_SE3.C)');
            t_new = -1*(C_new * obj.t);
            invTransform = SE3(C_new, t_new);
        end

    end
end
