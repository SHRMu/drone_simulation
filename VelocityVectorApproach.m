function [ut,obs,ob] = VelocityVectorApproach(x,model,goal,zoneParams,obs,drones,dR,evalParams) 
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
    
    Vt = CalcFieldVelocity(x,model,goal,zoneParams,obs_on,obs_dr,dR); % Vt is ideal field velocity
    obs_all = [obs_on;obs_dr];
    Vr = [Vt(1) Vt(1) Vt(2) Vt(2) Vt(3) Vt(3)]; % ideal 
%     Vr = CalcDynamicWindow(x,Vt,model);
    if isempty(obs_all) || isempty(obs_dr)
        ut=[Vr(1) Vr(3) Vr(5)]';
    else
        [evalDB,trajDB] = Evaluation(x,Vt,model,goal,Vr,obs_dr,drones,zoneParams,evalParams(4));
        if isempty(evalDB)
            ut=[Vr(1); Vr(3); Vr(5)];
%              ut = breakdown(model,x(4),x(5),x(6));
%              return;
        else
            ut=[Vr(1); Vr(3); Vr(5)];
        end
    end
    
end






