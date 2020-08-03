% obstacle recognition
% obstacles within the sensor zone
% obstalces can be recontructed
function obs_new = RecogObstacles(x,zoneParam,obs)
    zoneObs = [];
    otherObs = [];
    % filter obstacles within sensor zone
    for oi = 1:length(obs(:,1))
        % distance from obs_oi to x
        di=norm(obs(oi,:)-x(1:2)');
        if di <= zoneParam(2) % within sensor zone
            zoneObs = [zoneObs;obs(oi,1) obs(oi,2)];
        else
            otherObs = [otherObs;obs(oi,1) obs(oi,2)];
        end
    end
    obs_new = [];
    while ~isempty(zoneObs) 
        groupObs = [];
        ob = [zoneObs(1,:)];
        groupObs = [groupObs;ob]; 
        zoneObs(1,:) = [];
        if isempty(zoneObs)
            obs_new=[obs_new;groupObs];
            break;
        else
            [minVal,row] = matchest(zoneObs,ob);
            while ~isempty(zoneObs) && minVal <= zoneParam(1)
               groupObs = [groupObs;zoneObs(row,:)];
               zoneObs(row,:) = [];
            end
            obs_new=[obs_new;ReconstructObs(x,groupObs)];
        end
    end
end

function [obs] = ReconstructObs(x,obs)
    [tx,ty] = shortestPoint(x, obs);
%     obs = mean(obs);
    ob = [tx ty];
    [~,row]= matchest(obs,ob);
    obs = [obs(row,:);ob]; 
end

function [minValue,row] = matchest(zoneObs,ob)
%     [N,M]=size(zoneObs);
%     Distance=zeros([1,N]);
    Distance=sqrt(ob.^2*ones(size(zoneObs'))+ones(size(ob))*(zoneObs').^2-2*ob*zoneObs');
    [minValue,row]=min(Distance);
end
