% x_jk is a Cell Array --> x_jk = {T(4,4), p(4,1)}
function result = g(x_jk)
    % The full Nonlinear Observation model
    result = s(z(x_jk));
end

