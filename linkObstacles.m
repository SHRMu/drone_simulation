function res_obs = linkObstacles(obs,R) % R:danger region R
    if ~isempty(obs)
%         plot(obs(:,1),obs(:,2),'d');hold on;
        obs = sortrows(obs);
        [M,~] = size(obs);
        link_mat = zeros(M,M);
        for oi = 1:M-1
            neighbour_flag = 0;
            for oj = oi+1:M
                
                dis=norm(obs(oj,:)-obs(oi,:));
                if dis <= R
                    neighbour_flag = 1;
                    link_mat(oi,oj) = 1;
                    link_mat(oj,oi) = 1;
%                     line([obs(oi,1),obs(oj,1)],[obs(oi,2),obs(oj,2)],'LineWidth',2);hold on;
                end
                
            end
            
            if neighbour_flag == 0
%                 plot(obs(oi,1),obs(oi,2),'d');hold on;
            end
            
        end
        res_obs = [];
        di = diag(link_mat);
        items = find(di==0);
        while ~isempty(items)
            oi = items(1);
            array = link_mat(oi,:);
            index = find(array == 1);
            next_id = index(find(index ~= oi));
            if array(oi) == 0
                res_obs = [res_obs; obs(oi,:)];
                link_mat(oi,oi) = 1;
            end
            while ~isempty(next_id)       
                for i = 1:length(next_id)
                    id = next_id(i);
                    res_obs = [res_obs;obs(id,:)];
                    link_mat(oi,id) = 0;
                    link_mat(id,oi) = 0;
                    link_mat(id,id) = 1;
                    oi = id;
                end
                array = link_mat(oi,:);
                index = find(array == 1);
                next_id = index(find(index ~= oi));
            end
            di = diag(link_mat);
            items = find(di==0);
        end
    end
end