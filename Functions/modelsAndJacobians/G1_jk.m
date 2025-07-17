function result = G1_jk(x_op_jk)
    % 1st part of G (3x6)
    p_G = z(x_op_jk); % The landmark represented w.r.t to the Priveleged frame
    result = S_jk(p_G)*circleDot(p_G);
end

