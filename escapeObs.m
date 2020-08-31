function ob = escapeObs(x, obs)
%     slopes = [];
%     points = [];    
    obs = sortMatrix(obs);
    M = containers.Map('KeyType','double', 'ValueType','any');
%     start = 1;
%     for oi = 1:length(obs)-1
    oi = 1;
    while oi < length(obs)
        for oj = oi+1:length(obs)
            tmp = [obs(oi,:);obs(oj,:)];
            if oj ~= length(obs) && checkCollinear(obs(oj+1,:),tmp) == 1
                continue;
%             elseif oj ~= length(obs) && oi ~= 1 && checkCollinear(obs(oi-1,:),tmp) == 1
%                 break;
            end
            slope = (obs(oj,2)-obs(oi,2))/(obs(oj,1)-obs(oi,1));
            if ~isKey(M,slope)
                if oj ~= length(obs)
                    [tx,ty] = shortestPoint(x, [obs(oi,:); obs(oj,:)]);
                else
                    [tx,ty] = shortestPoint(x, [obs(oi,:); obs(oj,:)]);
                end
                ob = [tx,ty];
                TF = checkCollinear(ob,obs);
                if TF == 1
                    M(slope) = [tx,ty];
                end
            end
            oi = oj;
            break;
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