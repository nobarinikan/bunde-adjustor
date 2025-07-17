% x_jk is a Cell Array --> x_jk = {T(4,4), p(4,1)}
function result = z(x_jk)
    % Carry jth Landmark Point from kth Frame to Priveleged Frame G
    result = x_jk{1}*x_jk{2}; % 4x1 landmark p represented w.r.t Frame G
end

