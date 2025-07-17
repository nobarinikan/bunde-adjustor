% In: 4x1 Homogenous Vector
% Out: 4x6 weird thing
function result = circleDot(p)
    result = [p(4)*eye(3) -skew(p(1:3));
              zeros(1,3) zeros(1,3)]; 
end