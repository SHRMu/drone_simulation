function ob = escapeObs(x, obs)
%     slopes = [];
%     points = [];    
    obs = sortrows(obs);
    M = containers.Map('KeyType','double', 'ValueType','any');
    for oi = 1:length(obs)-1
        for oj = oi+1:length(obs)
            tmp = [obs(oi,:);obs(oj,:)];
            if oj ~= length(obs) && isCollinear(obs(oj+1,:),tmp) == 1
                break
            end
            slope = (obs(oj,2)-obs(oi,2))/(obs(oj,1)-obs(oi,1));
            if ~isKey(M,slope)
                [tx,ty] = shortestPoint(x, [obs(oi,:); obs(oj,:)]);
                ob = [tx,ty];
                TF = isCollinear(ob,obs);
                if TF == 1
                    M(slope) = [tx,ty];
                end
            end
        end
    end
    slopes = cell2mat(keys(M));
    ob = M(slopes(1));
    dis = norm(ob(1,:)-x(1:2)');
    for i = 2:length(slopes)
        ob_tmp = M(slopes(i));
        tmp = norm(ob_tmp(1,:)-x(1:2)');
        if tmp < dis
            ob = ob_tmp;
        end
    end
end