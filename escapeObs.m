function [ob_on,ob_off] = escapeObs(x, obs) 
    obs = sortMatrix(obs);
    M = containers.Map('KeyType','double', 'ValueType','any');
    oi = 1;
    while oi < length(obs)
        for oj = oi+1:length(obs)
            tmp = [obs(oi,:);obs(oj,:)];
            if oj ~= length(obs) && checkCollinear(obs(oj+1,:),tmp) == 1
                continue;
            end
            slope = (obs(oj,2)-obs(oi,2))/(obs(oj,1)-obs(oi,1));
            if ~isKey(M,slope)
                [tx,ty] = shortestPoint(x, [obs(oi,:); obs(oj,:)]);
                ob_on = [tx,ty];
                TF = checkCollinear(ob_on,obs);
                if TF == 1
                    M(slope) = [tx,ty];
                end
            end
            oi = oj;
            break;
        end
    end
    
    slopes = cell2mat(keys(M));
    ob_on = M(slopes(1));
    dis = norm(ob_on(1,:)-x(1:2)');
    % add effect obs
    ob_off = [];
    for i = 2:length(slopes)
        ob_tmp = M(slopes(i));
        tmp = norm(ob_tmp(1,:)-x(1:2)');
        if tmp < dis
            ob_off = [ob_off;ob_on];
            ob_on = ob_tmp;
        else
            ob_off = [ob_off;ob_tmp];
        end
    end
end