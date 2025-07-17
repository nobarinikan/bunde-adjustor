% In: 3x1 Vector
% Out: 3x3 Skew-symmetrix Matrix
function S = skew(x)
    S=[0 -x(3) x(2);
       x(3) 0 -x(1);
      -x(2) x(1) 0];
end
% skew = @(x)...
%     [0 -x(3) x(2);
%      x(3) 0 -x(1);
%     -x(2) x(1) 0];