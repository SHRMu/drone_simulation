function plotObstacles(obs,R)
    if ~isempty(obs)  
        for oi=1:length(obs(:,1))
            if oi == length(obs(:,1))
                dis=norm(obs(oi-1,:)-obs(oi,:));
                if dis <= R
                    line([obs(oi-1,1),obs(oi,1)],[obs(oi-1,2),obs(oi,2)],'LineWidth',2);hold on;
                else
                    plot(obs(oi,1),obs(oi,2),'d');hold on;
                end   
            else
                % group obstacles when dis between them less then 2*dangerR
                oj=oi+1;
                dis=norm(obs(oj,:)-obs(oi,:));
                if dis <= R 
                    ob = [obs(oi,:);obs(oj,:)];
                    % connect to next point
                    if oj~=length(obs(:,1)) && checkCollinear(obs(oj+1,:),ob) == 1
                        line([obs(oi,1),obs(oj,1)],[obs(oi,2),obs(oj,2)],'LineWidth',2);hold on;
                    elseif oi ~= 1 && checkCollinear(obs(oi-1,:),ob) == 1
                        line([obs(oi,1),obs(oj,1)],[obs(oi,2),obs(oj,2)],'LineWidth',2);hold on;
                    else
                        plot(obs(oi,1),obs(oi,2),'d');hold on;
                    end
                    continue;
                else
                    if oi > 2
                        ob = [obs(oi-2,:);obs(oi-1,:)];
                        if checkCollinear(obs(oi,:),ob) == 1
                            line([obs(oi-1,1),obs(oi,1)],[obs(oi-1,2),obs(oi,2)],'LineWidth',2);hold on;
                        else
                            plot(obs(oi,1),obs(oi,2),'d');hold on;
                        end
                    else
                        plot(obs(oi,1),obs(oi,2),'d');hold on;
                        continue;
                    end
                end  
            end
        end
    end
end
