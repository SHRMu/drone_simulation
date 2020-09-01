% obstacle recognition
% obstacles within the sensor zone
% obstalces can be recontructed
function [obs_on,obs_off] = RecogObstacles(x,zoneParam,obs)
    zoneObs = [];
%     otherObs = [];
    % filter obstacles within sensor zone
    for oi = 1:length(obs(:,1))
        % distance from obs_oi to x
        di=norm(obs(oi,:)-x(1:2)');
        if di <= zoneParam(2) % within sensor zone
            zoneObs = [zoneObs;obs(oi,1) obs(oi,2)];
        end
    end
    obs_on = [];
    obs_off = [];
    while ~isempty(zoneObs) 
        groupObs = [];
        [minVal,row] = matchest(zoneObs,[x(1),x(2)]); 
        ob = zoneObs(row,:);
        groupObs = [groupObs;ob]; 
        zoneObs(row,:) = [];
        if isempty(zoneObs)
            obs_on=[obs_on;groupObs];
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
                [ob, obs_off] = ReconstructObs(x,groupObs);
                obs_on=[obs_on; ob];
            else
                obs_on = [obs_on;groupObs];
            end
        end
    end
end

function [obs,obs_off] = ReconstructObs(x,obs)
    TF = isCollinear(obs);
    if TF == 1
        [tx,ty] = shortestPoint(x, obs);
        ob = [tx ty];
        obs_off = [];
    else
        [ob,obs_off] = escapeObs(x,obs);
    end
    
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
