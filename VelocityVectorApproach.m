function [u,traj] = VelocityVectorApproach(x,model,goal,zoneParam,obs,r)
    obs_new = RecogObstacles(x,goal,zoneParam,obs,r);
    [vt,ot] = CalcVelocity(x,model,goal,zoneParam,obs_new,r);
    [x,traj]= GenerateTrajectory(x,model,vt,ot,0.1);
    u = [vt,ot]';
end

function obs_new = RecogObstacles(x,goal,zoneParam,obs,r)
    zoneObs = [];
    otherObs = [];
    % filter obstacles within sensor zone
    for oi = 1:length(obs(:,1))
        di=norm(obs(oi,:)-x(1:2)');
        if di <= zoneParam(2)
            zoneObs = [zoneObs;obs(oi,1) obs(oi,2)];
        else
            otherObs = [otherObs;obs(oi,1) obs(oi,2)];
        end
    end
    obs_new = [];
    while ~isempty(zoneObs) 
        tmp = [];
        ob = [zoneObs(1,:)];
        tmp = [tmp;ob]; % tmp group
        zoneObs(1,:) = [];
        if isempty(zoneObs)
            obs_new=[obs_new;tmp];
            break;
        else
            [minVal,row] = matchest(zoneObs,ob);
            while ~isempty(zoneObs) && minVal <= zoneParam(1)
               tmp = [tmp;zoneObs(row,:)];
               zoneObs(row,:) = [];
            end
            obs_new=[obs_new;ReconstructObs(x,tmp)];
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


