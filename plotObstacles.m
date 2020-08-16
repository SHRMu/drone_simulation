function plotObstacles(obs,R)
    for oi=1:length(obs(:,1))
        flag = 0;
        % group obstacles when dis between them less then 2*dangerR
        for oj=oi+1:length(obs(:,1))
            dis=norm(obs(oj,:)-obs(oi,:));
            if dis <= 2*R
                flag = 1;
                line([obs(oi,1),obs(oj,1)],[obs(oi,2),obs(oj,2)],'LineWidth',1);hold on;
            end
        end
        if flag == 0
            plot(obs(oi,1),obs(oi,2),'d');hold on;
        end      
    end
end
