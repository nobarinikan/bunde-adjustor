% In: 3x1 Vector
% Out: 3x1 Normalized Vector
function v_normalized = normalize3DVector(v)
    if norm(v) == 0
        % disp('Norm of vector is 0!!!!');
        v_normalized = v;
    else
        v_normalized = [v(1)/norm(v);
                        v(2)/norm(v);
                        v(3)/norm(v);];
    end
end
% normalize3DVector = @(v)...
%     [v(1)/norm(v);
%      v(2)/norm(v);
%      v(3)/norm(v);];
