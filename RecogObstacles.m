% obstacle recognition
% obstacles within the sensor zone
% obstalces can be recontructed
function obs_new = RecogObstacles(x,zoneParam,obs)
    zoneObs = [];
%     otherObs = [];
    % filter obstacles within sensor zone
    for oi = 1:length(obs(:,1))
        % distance from obs_oi to x
        di=norm(obs(oi,:)-x(1:2)');
        if di <= zoneParam(2) % within sensor zone
            zoneObs = [zoneObs;obs(oi,1) obs(oi,2)];
%         else
%             otherObs = [otherObs;obs(oi,1) obs(oi,2)];
        end
    end
    obs_new = [];
    while ~isempty(zoneObs) 
        groupObs = [];
%         ob = [zoneObs(1,:)];
        [minVal,row] = matchest(zoneObs,[x(1),x(2)]); 
        ob = zoneObs(row,:);
        groupObs = [groupObs;ob]; 
        zoneObs(row,:) = [];
        if isempty(zoneObs)
            obs_new=[obs_new;groupObs];
            break;
        else
            [minVal,row] = matchest(zoneObs,ob);
            while ~isempty(zoneObs) && minVal <= zoneParam(1)
               groupObs = [groupObs;zoneObs(row,:)];
               zoneObs(row,:) = [];
               [minVal,row] = groupMatchest(zoneObs,groupObs);
            end
            [N,~] = size(groupObs);
            if N >= 2
                obs_new=[obs_new;ReconstructObs(x,groupObs)];
            else
                obs_new = [obs_new;groupObs];
            end
        end
    end
end

function [obs] = ReconstructObs(x,obs)
    TF = isCollinear(obs);
    if TF == 1
        [tx,ty] = shortestPoint(x, obs);
        ob = [tx ty];
    else
        ob = escapeObs(x,obs);
    end
    
%     [in,on] = inpolygon(x(1),x(2),obs(:,1),obs(:,2));
%     if in == 1 || on == 1
%         [tx,ty] = escapeObs(x,obs);
%     end
    
    [~,row]= matchest(obs,ob);
    obs = [obs(row,:);ob]; 
end

function [minValue,row] = matchest(zoneObs,ob)
%     [N,M]=size(zoneObs);
%     Distance=zeros([1,N]);
    Distance=sqrt(ob.^2*ones(size(zoneObs'))+ones(size(ob))*(zoneObs').^2-2*ob*zoneObs');
    [minValue,row]=min(Distance);
end

function [minValue,row] = groupMatchest(zoneObs,groupObs)
    [minValue,row] = matchest(zoneObs,groupObs(1,:));
    for oi = 2:length(groupObs)
        [tmp_value,tmp_row] = matchest(zoneObs,groupObs(oi,:));
        if tmp_value < minValue
            minValue = tmp_value;
            row = tmp_row;
        end
    end
end
