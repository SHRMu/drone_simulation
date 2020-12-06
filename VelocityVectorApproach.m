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
    [obs_dr] = droneConnection(x, zoneParams, drones);
    
    [Vt] = calculateVelocity(x,model,goal,zoneParams,obs_on,obs_dr,dR);
    obs_all = [obs_on;obs_dr];
    if isempty(obs_all)
        Vr = CalcDynamicWindow(x,Vt,model);
        ut=[Vr(1) Vr(3) Vr(5)]';
    else
        Vr = CalcDynamicWindow(x,Vt,model);
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
        %
%         evalDB=NormalizeEval(evalDB);
%         %
%         feval=[];
%         for id=1:length(evalDB(:,1))
%             feval=[feval;evalParams(1:3)*evalDB(id,4:6)']; %evalParam=[0.05,0.2,0.1,0.05];
%         end
%         evalDB=[evalDB feval];
% 
%         [maxv,ind]=max(feval);%
%         ut=evalDB(ind,1:3)';% 
    end
    
end

function Vr=CalcDynamicWindow(x,Vt,model)
    % 
    global dt;
    yaw = rad2deg(x(3));
    Wt = calcAngular(Vt(1),Vt(2),yaw);
    % current max and min range of velocity
    Vc=[x(4)-model(2)*dt x(4)+model(2)*dt x(5)-model(2)*dt x(5)+model(2)*dt x(6)-model(4)*dt x(6)+model(4)*dt];
    % Maximum and minimum range of velocity
    Vs=[0 model(1) -model(1) model(1) -model(3) model(3)]; %[0 Vm -Wm Wm ]
    Vc=[max(Vc(1),Vs(1)) min(Vc(2),Vs(2)) max(Vc(3),Vs(3)) min(Vc(4),Vs(4)) max(Vc(5),Vs(5)) min(Vc(6),Vs(6))];

    if Vt(1) < Vc(1)
        Vr(1)=Vc(1);
        Vr(2)=Vc(1);
    elseif Vt(1) > Vc(2)
        Vr(1)=Vc(2);
        Vr(2)=Vc(2);
    else
        Vr(1)=Vt(1);
        Vr(2)=Vt(1);
    end

    if Vt(2) < Vc(3)
        Vr(3)=Vc(3);
        Vr(4)=Vc(3);
    elseif Vt(2) > Vc(4)
        Vr(3)=Vc(4);
        Vr(4)=Vc(4);
    else
        Vr(3)=Vt(2);
        Vr(4)=Vt(2);
    end

    
    
    angular = calcAngular(Vr(1),Vr(3),yaw);
    if angular >  Vc(6)
        if Vr(1) < Vc(2)
            Vr(1) = Vr(1) + 0.001;
        end
        if Vr(3) > Vc(3)
            Vr(3) = Vr(3) - 0.001;
        end
        angular = calcAngular(Vr(1),Vr(3),yaw);
        while angular > Vc(6)
            if Vr(1) >= Vc(2) && Vr(3) <= Vc(3)
                break;
            end
            if Vr(1) < Vc(2)
                Vr(1) = Vr(1) + 0.001;
            end
            if Vr(3) > Vc(3)
                Vr(3) = Vr(3) - 0.001;
            end
            angular = calcAngular(Vr(1),Vr(3),yaw);
        end
        Vr(5) = angular;
    elseif angular < Vc(5)
        if Vr(1) > Vc(1)
            Vr(1) = Vr(1) - 0.001;
        end
        if Vr(3) < Vc(4)
            Vr(3) = Vr(3) + 0.001;
        end
        angular = calcAngular(Vr(1),Vr(3),yaw);
        while angular < Vc(5) 
            if Vr(1) <= Vc(1) && Vr(3) >= Vc(4)
                break;
            end
            if Vr(1) > Vc(1)
                Vr(1) = Vr(1) - 0.001;
            end
            if Vr(3) < Vc(4)
                Vr(3) = Vr(3) + 0.001;
            end
            angular = calcAngular(Vr(1),Vr(3),yaw);
        end
        Vr(5) = angular;
    else
        Vr(5) = angular;
    end
    Vr(6) = Vr(5);
    
end

function angular = calcAngular(Vx,Vy,yaw)
    global dt;
    A = [cos(deg2rad(yaw)) -sin(deg2rad(yaw));
         sin(deg2rad(yaw))  cos(deg2rad(yaw))];
    delta = A*[Vx;Vy];
    delta_x = delta(1,1);
    delta_y = delta(2,1);
    delta_th = angleConversion(rad2deg(atan2(delta_y,delta_x))-yaw);
    angular=deg2rad(min(delta_th,360-delta_th))/dt;
end

function EvalDB=NormalizeEval(EvalDB)
    if sum(EvalDB(:,4))~=0
        EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
    end
    if sum(EvalDB(:,5))~=0
        EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
    end
    if sum(EvalDB(:,6))~=0
        EvalDB(:,6)=EvalDB(:,6)/sum(EvalDB(:,6));
    end
end



