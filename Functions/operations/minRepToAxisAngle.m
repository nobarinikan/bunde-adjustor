% % In: 3x1 phi = a, min rep. of representation
% % Out: 3x1 norm(a)*unit vector
function axisAngleStruct = minRepToAxisAngle(phi)
    axisAngleStruct = struct('angle', norm(phi), 'unitAxis', normalize3DVector(phi));
end
% minRepToAxisAngle = @(phi)...
%     struct('angle', norm(phi), 'unitAxis', [phi(1)/norm(phi); phi(2)/norm(phi); phi(3)/norm(phi)]);


