function TF = isCollinear(ob,obs)
    TF = 0;
    for oi = 1:length(obs)
        for oj = oi+1:length(obs)
            A = [obs(oi,:) 1; obs(oj,:) 1; ob(1,:) 1];
            if rank(A)== 2
                TF = 1;
                break;
            end
        end
    end
end