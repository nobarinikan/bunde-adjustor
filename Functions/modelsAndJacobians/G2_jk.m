function result = G2_jk(x_op_jk)
    % 2nd part of G (3x3)
    p_G = z(x_op_jk); % The landmark represented w.r.t to the Priveleged frame
    result = S_jk(p_G)*x_op_jk{1}*D;
end

