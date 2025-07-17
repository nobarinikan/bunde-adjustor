% IN: 3x1 or 6x1 local lie algebra vector
% OUT: 3x3 or 6x6 left J.
function J = jacSeries(vec)
validateattributes(vec,{'double'},{'ncols',1})
N = 10;

if size(vec,1) == 3 
    
    J = eye(3);
    pxn = eye(3);
    phiMatrix = skew(vec);
    for n = 1:N
        pxn = pxn*phiMatrix/(n + 1);    
        J = J + pxn;
    end
    
elseif size(vec,1) == 6    
    
    J = eye(6);
    pxn = eye(6);
    phiMatrix = curlyhat(vec);
    for n = 1:N
        pxn = pxn*phiMatrix /(n + 1);    
        J = J + pxn;
    end    
    
else   
    warning('Wrong size for input size in jacSeries');   
end


end

