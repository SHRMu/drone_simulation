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
        ob = [];
        obs_on = [];
        obs_off = [];
    end
    % dynamic drones communication
    [obs_dr,dronesInSensor] = droneConnection(x, goal, zoneParams, drones);
    
    Vt = CalcFieldVelocity(x,model,goal,zoneParams,obs_on,obs_dr,dR); % Vt is ideal field velocity
    obs_all = [obs_on;obs_dr];
    
    %%%%%%%%%%%%%%%%%%%%%% ideal case %%%%%%%%%%%%%%%%%%%%%%
%     ut = [Vt(1); Vt(2); Vt(3)]; % ideal 

    %%%%%%%%%%%%%%%%%%%%%% actual  %%%%%%%%%%%%%%%%%%%%%%%
    
    Vr = CalcDynamicWindow(x,Vt,model);

    [evalDB] = Evaluation(x,Vt,model,Vr,obs_all,dronesInSensor,zoneParams);
    
    if isempty(evalDB)
        if abs(Vr(1)-0.0001)<=0.0001 && abs(x(4)-0.0001)<=0.0001
            ut = [0; 0; Vt(3)]; return; %hover
        else
            ut = breakdown(model,x(4),x(5),x(6)); return; % brake because of obstacle
        end
    else
        evalDB = NormalizeEval(evalDB);
        feval=[];
        for id=1:length(evalDB(:,1))
            feval=[feval;evalParams(1:2)*evalDB(id,4:5)'];
        end
        evalDB=[evalDB feval];

        [maxv,ind]=max(feval);%
        ut=evalDB(ind,1:2)';% 
        wt = CalcAngular(x,ut(1),ut(2));
        ut = [ut(1); ut(2); wt];
    end
%      
%     if isempty(obs_all) && abs(ut(3)-Vt(3)) > 0.0001
%         if abs(Vr(1)-0.0001)<=0.0001 && abs(x(4)-0.0001)<=0.0001
%             ut = [0; 0; Vt(3)]; return; %hover
%         else
%             ut = breakdown(model,x(4),x(5),x(6)); return; % brake because of obstacle
%         end
%     end
        
end

function EvalDB=NormalizeEval(EvalDB)
    if sum(EvalDB(:,4))~=0
        EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
    end
    if sum(EvalDB(:,5))~=0
        EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
    end
end






