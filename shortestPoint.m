function [tx,ty] = shortestPoint(x, obs)
    [N,~] = size(obs);
    C = eye(2);
    d = [x(1);x(2)];
    A = [];
    b = [];
    for i = 1:N-1
        for j = i+1:N
            if (obs(j,1)-obs(i,1))==0 || (obs(j,2)-obs(i,2))==0
                continue;
            end
            % [(y1-y2)/(x2-x1) 1]
            A = [A;(obs(i,2)-obs(j,2))/(obs(j,1)-obs(i,1)) 1];
%             A = [A;(obs(i,2)-obs(j,2))/(obs(j,1)-obs(i,1)) 1;(obs(i,2)-obs(j,2))/(obs(j,1)-obs(i,1)) 1];
            % y2-x2*(y2-y1)/(x2-x1)
            b = [b;obs(j,2)-obs(j,1)*((obs(j,2)-obs(i,2))/(obs(j,1)-obs(i,1)))];
%             val_b = obs(j,2)-obs(j,1)*((obs(j,2)-obs(i,2))/(obs(j,1)-obs(i,1)));
%             b = [b; val_b; (-1)*val_b];
        end
    end
    lb = min(obs);
    ub = max(obs);
    [t] = lsqlin(C,d,A,b,A,b,lb,ub);
    tx = t(1);
    ty = t(2);
    tx = roundn(tx,-2);
    ty = roundn(ty,-2);
end
