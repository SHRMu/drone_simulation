function ret = sortMatrix(obs)
    obs = sortrows(obs);
    ret = [];
    for oi = 1:length(obs)
        if oi ~= 1 && obs(oi,1) == obs(oi-1,1)
            break;
        else
            val = obs(oi,1);
            F = obs(find(obs(:,1)==val),:);
            F = sortrows(F,-2);
            ret = [ret;F];
        end
    end
end
