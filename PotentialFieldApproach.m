function [ut,obs,ob] = PotentialFieldApproach(x,model,goal,zoneParams,obs,drones,dR,evalParams) 
    % static obs
    if ~isempty(obs)
        [obs_on,obs_off] = recogObstacles(x,goal,zoneParams,obs); 
        ob = [];
        if ~isempty(obs_on)
            ob = obs_on(1,:);
        end
%         obs = [obs;ob];
%         obs = unique(obs,'rows','stable');
    else
        obs_on = [];
        obs_off = [];
    end
    % dynamic drones communication
    [obs_dr] = droneConnection(x, goal, zoneParams, drones);
    
    Vt = CalcPotentialVelocity(x,model,goal,zoneParams,obs_on,obs_dr,dR); % Vt is ideal field velocity
    
    obs_all = [obs_on;obs_dr];
    Vr = [Vt(1) Vt(1) Vt(2) Vt(2) Vt(3) Vt(3)];
%     Vr = CalcDynamicWindow(x,Vt,model);
    if isempty(obs_all) || isempty(obs_dr)
        ut=[Vr(1) Vr(3) Vr(5)]';
    else
        [evalDB,trajDB] = Evaluation(x,Vt,model,goal,Vr,obs_dr,drones,zoneParams,evalParams(4));
        if isempty(evalDB)
            global dt;
            if abs(x(4)) < model(2)*dt
                vtx = 0;
            else
                vtx = x(4) - x(4)/abs(x(4))*model(2)*dt;
            end
            if abs(x(5)) < model(2)*dt
                vty = 0;
            else
                vty = x(5) - x(5)/abs(x(5))*model(2)*dt;
            end
            ut=[vtx;vty;0];return;
        else
            ut=[Vr(1); Vr(3); Vr(5)];
        end
    end
end

function Vt = CalcPotentialVelocity(x,model,goal,zoneParams,obs_on,obs_dr,dR)
    % no obs within sensor zone
    if isempty(obs_on) && isempty(obs_dr)
        theta_t = angleConversion(rad2deg(atan2(goal(1,2)-x(2),goal(1,1)-x(1)))); % theta2target
        yaw = angleConversion(rad2deg(x(3)));
        delta_t = angleConversion(theta_t - yaw);
        Tvtx = model(1)*cos(deg2rad(delta_t));
        Tvty = model(1)*sin(deg2rad(delta_t));
    else
        [Tvtx,Tvty] = calcFunc(x,model,goal,zoneParams,obs_on,obs_dr,dR);
    end
    Twt = CalcAngular(x,Tvtx,Tvty);
    Vt = [Tvtx,Tvty,Twt]';
end

function [Tvtx,Tvty] = calcFunc(x,model,goal,zoneParams,obs_on,obs_dr,dR)
    vt_mat = [];
    obs.pos = [obs_on;obs_dr];
    if ~isempty(obs.pos)
        obs.dis = [];
        obs.weight = [];
        for io=1:length(obs.pos(:,1))
    %         for io=1:1
            dis=norm(obs.pos(io,:)-x(1:2)');
            vt = vtVector(x,model,goal,obs.pos(io,:),dis,zoneParams,dR);
            if any(vt) == 1
                obs.dis = [obs.dis;1/dis];
                vt_mat = [vt_mat;vt];
            end
        end
        if ~isempty(obs.dis)
            sum_dis = sum(obs.dis);
            obs.weight = obs.dis/sum_dis;
        end
    else
        V_t = model(1);
        theta_t = angleConversion(rad2deg(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
        vtx = V_t*cos(toRadian(theta_t));
        vty = V_t*sin(toRadian(theta_t));
        vt_mat = [vt_mat;vtx vty]; 
    end
    
    [M,~] = size(vt_mat);
    if M > 1
        vt_mat = sum(vt_mat.*obs.weight);
    end
    
    if M  == 0
        vt_mat = [0 0];
    end
    
    Tvtx = vt_mat(1);
    Tvty = vt_mat(2);
    
end

function [vt] = vtVector(x,model,goal,obs,dis,zoneParams,dR)

    theta_t = angleConversion(rad2deg(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
    theta_o = angleConversion(rad2deg(atan2(obs(1,2)-x(2),obs(1,1)-x(1)))-180);

    yaw = angleConversion(rad2deg(x(3)));

    if dis < dR
        alpha = 0;
        beta = 1;
    elseif dis > dR && dis <= zoneParams(2)
        alpha = 0;
        beta = 1;
    elseif dis > zoneParams(2) && dis <= zoneParams(3)
        if abs(angleConversion(theta_t - theta_o)) < 90
            alpha = 1;
            beta = 0;
        else
            alpha = 0.5*(1-cos(pi*(dis-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
            beta = 0.5*(1+cos(pi*(dis-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
        end
    else
        alpha = 1;
        beta = 0;
    end

    V_t = alpha*model(1);
    V_o = beta*model(1);
    
    delta_t = angleConversion(theta_t - yaw);
    delta_o = angleConversion(theta_o - yaw);

    vtx = V_t*cos(deg2rad(delta_t))+V_o*cos(deg2rad(delta_o));
    vty = V_t*sin(deg2rad(delta_t))+V_o*sin(deg2rad(delta_o));

    vt = [vtx vty];
end