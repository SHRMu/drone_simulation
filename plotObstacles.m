function plotObstacles(obs,R)
    for oi=1:length(obs(:,1))-1
        % group obstacles when dis between them less then 2*dangerR
        for oj=oi+1:length(obs(:,1))
            dis=norm(obs(oj,:)-obs(oi,:));
            if dis <= R 
                ob = [obs(oi,:);obs(oj,:)];
                if oj~=length(obs(:,1)) && checkCollinear(obs(oj+1,:),ob) == 1
                    line([obs(oi,1),obs(oj,1)],[obs(oi,2),obs(oj,2)],'LineWidth',2);hold on;
                elseif oi ~= 1 && checkCollinear(obs(oi-1,:),ob) == 1
                    line([obs(oi,1),obs(oj,1)],[obs(oi,2),obs(oj,2)],'LineWidth',2);hold on;
                else
                    plot(obs(oi,1),obs(oi,2),'*');hold on;
                end
                break;
            else
                plot(obs(oi,1),obs(oi,2),'*');hold on;
                break;
            end
        end    
    end
end
