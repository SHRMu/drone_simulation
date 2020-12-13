function [x,group] = checkGroup(x, R, group)
% R = 2*dangerR
% less than R -> add group
%
    if ~isempty(x)
        xi = x(1:2,:);
        [~, N] = size(x);
        for di = 1:N-1
            dj = di +1;
            dis = norm(xi(:,di)-xi(:,dj));
            if dis < R
                x(6,di) = 1;
                x(6,dj) = 1;
                group = [5 5 R; 5 5 R];
            end
        end
    end
end