function ret = sortMatrix(x, obs)
    obs = sortrows(obs);
    ret = [];
    for oi = 1:length(obs)
        if oi ~= 1 && obs(oi,1) == obs(oi-1,1)
            continue;
        else
            val = obs(oi,1);
            F = obs(find(obs(:,1)==val),:);
            if F(1,1) < x(1)
                F = sortrows(F,2);
            else
                F = sortrows(F,-2);
            end
            ret = [ret;F];
        end
    end
end
