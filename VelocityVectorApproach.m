function [u,traj] = VelocityVectorApproach(x,model,goal,zoneParam,obs,r)
    obs = RecogObstacles(x,goal,zoneParam,obs,r);
    [vt,ot] = CalcVelocity(x,model,goal,zoneParam,obs,r);
    [x,traj]= GenerateTrajectory(x,model,vt,ot,0.1);
    u = [vt,ot]';
end

function [vt,ot] = CalcVelocity(x,model,goal,zoneParam,obs,r)
    vt_mat = [];
    for io=1:length(obs(:,1))
        di=norm(obs(io,:)-x(1:2)');
        if di < r
            alpha = 0;
            beta = 1;
            gamma = 0;
        elseif di > r && di <= zoneParam(1)
            alpha = 0;
            beta = 0.5*(1+cos(pi*(di-r)/(zoneParam(1)-r)));
            gamma = 0.5*(1-cos(pi*(di-r)/(zoneParam(1)-r)));
        elseif di > zoneParam(1) && di <= zoneParam(2)
            alpha = 0.5*(1-cos(pi*(di-zoneParam(1))/(zoneParam(2)-zoneParam(1))));
            beta = 0;
            gamma = 0.5*(1+cos(pi*(di-zoneParam(1))/(zoneParam(2)-zoneParam(1))));
        elseif di > zoneParam(2)
            alpha = 1;
            beta = 0;
            gamma = 0;
        end
        
        V_t = alpha*model(1);
        theta_t = angleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
        V_o = beta*model(1);
        theta_o = angleConversion(toDegree(atan2(obs(io,2)-x(2),obs(io,1)-x(1)))-180);
        V_g = gamma*model(1);
        theta_g = angleConversion(toDegree(atan2(obs(io,2)-x(2),obs(io,1)-x(1)))-90);
        if abs(theta_t - theta_g) > 90
            theta_g = angleConversion(theta_g + 180);
        end
        
        vtx = V_t*cos(toRadian(theta_t))+V_o*cos(toRadian(theta_o))+V_g*cos(toRadian(theta_g));
        vty = V_t*sin(toRadian(theta_t))+V_o*sin(toRadian(theta_o))+V_g*sin(toRadian(theta_g));
        vt_mat = [vt_mat;vtx vty];
             
    end
    vt_mat = mean(vt_mat);
    vtx = vt_mat(1);
    vty = vt_mat(2);
    theta = atan2(vty,vtx);
    vt = vtx/cos(theta);
    if theta ~= x(3)
        ot = (theta-x(3))/0.1;
    else
        ot = 0;
    end 
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
    obs_new = [otherObs];
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
            obs = ReconstructObs(x,obs);
            obs_new=[obs_new;obs];
        end
    end
end

function [tmp] = ReconstructObs(x,obs)
    [minVal,row] = matchest(obs,[x(1,1),x(2,1)]);
    tmp = [obs(row,:)];
    obs = mean(obs);
    tmp = [tmp;obs];
end

function [x,y] = shortestPoint(x, obs)
    
end

function [minValue,row] = matchest(zoneObs,ob)
%     [N,M]=size(zoneObs);
%     Distance=zeros([1,N]);
    Distance=sqrt(ob.^2*ones(size(zoneObs'))+ones(size(ob))*(zoneObs').^2-2*ob*zoneObs');
    [minValue,row]=min(Distance);
end


