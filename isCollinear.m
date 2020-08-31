function TF = isCollinear(obs)
    TF = 1;
    obs = sortrows(obs);
    for oi = 1:length(obs)-2
        A = [obs(oi,:) 1; obs(oi+1,:) 1; obs(oi+2,:) 1];
        if rank(A)== 3
            TF = 0;
            break;
        end
    end
end