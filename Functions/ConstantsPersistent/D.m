% 4x3 Landmark Dilation Matrix
function result = D
    persistent D_value;
    if isempty(D_value)
        D_value = eye(4,3);
    end
    result = D_value;
end