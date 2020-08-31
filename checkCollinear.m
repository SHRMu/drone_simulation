function TF = checkCollinear(ob,obs)
    TF = 0;
    obs = sortrows(obs);
    for oi = 1:length(obs)-1
        for oj = oi+1:length(obs)
            A = [obs(oi,:) 1; obs(oj,:) 1; ob(1,:) 1];
            if rank(A)== 2
                TF = 1;
                break;
            end
        end
    end
end