% In: 4x1 Homogenous Vector
% Out: 6x4 weird thing
function result = circleCir(p)
    result = [zeros(3) p(1:3);
             -skew(p(1:3)) zeros(3,1)];
end