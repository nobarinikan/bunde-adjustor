% In: 3x1 Axis of rotation and 1x1 Angle of rotation
% Out: 3x3 Left Jacobian for Axis-angle Representation
function J = leftJacobian(input1, input2)
tolerance = 1e-12;

    leftJacobian = @(theta, a)...
        (sin(theta)/theta) * eye(3) + (1 - (sin(theta)/theta)) * normalize3DVector(a(:)) * normalize3DVector(a(:))' + (1 - cos(theta))/theta * skew(normalize3DVector(a(:)));
    if nargin == 2
        % Case 1: Angle (1x1) + Axis (3x1)
        if input1 < tolerance % input1 == 0 && input2 == zeros(3,1)
            % If the angle is small, then utilize series expansion. 
            % J = eye(3); Works if input1 & input2 = zeros(3,1)
            J = jacSeries(input1*input2);
        else          
            input2_normalized = normalize3DVector(input2);
            J = leftJacobian(input1,input2_normalized);
        end
    elseif nargin == 1 && size(input1,1) == 3
        % Case 2: Min-rep (3x1) phi 
        phi_angle = norm(input1);
        if phi_angle < tolerance % input1 == zeros(3,1)
            % If the angle is small, then utilize series expansion. 
            % J = eye(3); Works if input1 = zeros(3,1)
            J = jacSeries(input1);
        else          
            theta_extracted = minRepToAxisAngle(input1).angle;
            a_extracted = minRepToAxisAngle(input1).unitAxis;
            J = leftJacobian(theta_extracted,a_extracted);
        end
    else
        error('leftJacobian parameters are wrong!')
    end

end
% leftJacobian = @(theta, a)...
%     (sin(theta)/theta) * eye(3) + (1 - (sin(theta)/theta)) * a(:) * a(:)' + (1 - cos(theta))/theta * skew(a(:));
