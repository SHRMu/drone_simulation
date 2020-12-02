% obstacle recognition
% obstacles within the sensor zone
% obstalces can be recontructed
function [obs_on,obs_off] = recogObstacles(x,goal,zoneParams,obs)
    zoneObs = [];
%     otherObs = [];
    % filter obstacles within sensor zone
    for oi = 1:length(obs(:,1))
        % distance from obs_oi to x
        di=norm(obs(oi,:)-x(1:2)');
        if di <= zoneParams(3) % within sensor zone
            zoneObs = [zoneObs;obs(oi,1) obs(oi,2)];
        end
    end
    obs_on = [];
    obs_off = [];
    while ~isempty(zoneObs) 
        groupObs = [];
        [~,row] = matchest(zoneObs,[x(1),x(2)]); 
        ob = zoneObs(row,:);
        groupObs = [groupObs;ob]; 
        zoneObs(row,:) = [];
        if isempty(zoneObs)
            obs_on=[obs_on;groupObs];
            break;
        else
            [minVal,row] = matchest(zoneObs,ob);
            while ~isempty(zoneObs) && minVal <= zoneParams(2)
               groupObs = [groupObs;zoneObs(row,:)];
               zoneObs(row,:) = [];
               [minVal,row] = groupMatchest(zoneObs,groupObs);
            end
            [N,~] = size(groupObs);
            if N >= 2
                [ob, obs_off] = reconstructObs(x,groupObs);
                obs_on=[obs_on; ob];
            else
                obs_on = [obs_on;groupObs];
            end
        end
    end
    obs_on = unique(obs_on,'rows','stable');
    [N,~] = size(obs_on);
    if N >= 2
        obs_mat = zeros(length(obs_on),2);
        theta_t = angleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
        for oi = 1:length(obs_on)
            obs_mat(oi,1) = norm(obs(oi,:)-x(1:2)');
            theta_o = angleConversion(toDegree(atan2(obs_on(oi,2)-x(2),obs_on(oi,1)-x(1)))-180);
            obs_mat(oi,2) = abs(angleConversion(theta_o-theta_t));
        end
        if obs_mat(1,2) == obs_mat(2,2)
            if obs_mat(1,1) <= obs_mat(2,1)
                obs_off = [obs_off;obs_on(2,:)];
                obs_on(2,:) = [];
            else
                obs_off = [obs_off;obs_on(1,:)];
                obs_on(1,:) = [];
            end
        end
    end
    obs_off = unique(obs_off,'rows','stable');
end

function [obs_on,obs_off] = reconstructObs(x,obs)
    TF = isCollinear(obs);
    if TF == 1
        [tx,ty] = shortestPoint(x, obs);
        ob = [tx ty];
        obs_off = [];
    else
        % concave scenario
        [ob,obs_off] = escapeObs(x,obs);
    end
    
%     [~,row]= matchest(obs,ob);
%     obs_on = [obs(row,:);ob]; 
    obs_on = ob;
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
