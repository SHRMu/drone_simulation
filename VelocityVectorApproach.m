function [ut,obs,ob] = VelocityVectorApproach(x,model,goal,zoneParams,obs,drones,dR,evalParams) 
%     obs = [obs;drones];
    % static obs
    if ~isempty(obs)
        [obs_on,obs_off] = recogObstacles(x,goal,zoneParams,obs); 
        ob = [];
        if ~isempty(obs_on)
            ob = obs_on(1,:);
        end
        obs = unique(obs,'rows','stable');
    else
        obs_on = [];
        obs_off = [];
    end
    % dynamic drones communication
    [Ovir] = droneConnection(x, zoneParams, drones);
    
%     if ~isempty(Ovir)
% %         obs_on = Ovir(1,:);
%         obs_on = Ovir;
%     end
    
    [Vt] = calcVelocity(x,model,goal,zoneParams,obs_on,obs_off,Ovir,drones,dR,evalParams(4));
    if isempty(obs_on)
        Vr = CalcDynamicWindow(x,Vt,model);
        ut=[Vr(1) Vr(3) Vr(5)]';
    else
        Vr = CalcDynamicWindow(x,Vt,model);
        if ~isempty(drones)
            [evalDB,trajDB] = Evaluation(x,Vt,model,goal,Vr,Ovir,drones,zoneParams,evalParams(4));
            if isempty(evalDB)
    %             disp('no path to goal!!');
                ut=[0;0;0];return;
            end
            %
            evalDB=NormalizeEval(evalDB);
            %
            feval=[];
            for id=1:length(evalDB(:,1))
                feval=[feval;evalParams(1:3)*evalDB(id,4:6)']; %evalParam=[0.05,0.2,0.1,0.05];
            end
            evalDB=[evalDB feval];

            [maxv,ind]=max(feval);%
            ut=evalDB(ind,1:3)';% 
        else
            ut=[Vr(1) Vr(3) Vr(5)]';
        end
        
    end
    
end

function Vr=CalcDynamicWindow(x,Vt,model)
    % 
    global dt;
    % current max and min range of velocity
    Vc=[x(4)-model(2)*dt x(4)+model(2)*dt x(5)-model(2)*dt x(5)+model(2)*dt x(6)-model(4)*dt x(6)+model(4)*dt];
    % Maximum and minimum range of velocity
%     Vs=[0 model(1) 0 model(1) -model(2) model(2)]; %[0 Vm -Wm Wm ]

    % Dynamic window calculated based on current velocity and acceleration limit
%     Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt]; %[]
%     Vd=[u(1)-model(3)*dt u(1)+model(3)*dt u(2)-model(3)*dt u(2)+model(3)*dt u(3)-model(4)*dt u(3)+model(4)*dt];
%     Vt=[Vt(1) Vt(1) Vt(2) Vt(2) Vt(3) Vt(3)];

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
    
    yaw = rad2deg(x(3));
    delta_y = Vr(1)*sin(deg2rad(yaw)) + Vr(3)*cos(deg2rad(yaw));
    delta_x = Vr(1)*cos(deg2rad(yaw)) - Vr(3)*sin(deg2rad(yaw));
    th = atan2(delta_y,delta_x);
    Vr(5) = (th-x(3))/0.05;
    Vr(6)=Vr(5);
    
%     if Vr(5) < Vc(5)
%         Vr(5)=Vc(5);
%         Vr(6)=Vc(5);
%     elseif Vr(5) > Vc(6)
%         Vr(5)=Vc(6);
%         Vr(6)=Vc(6);
%     else
%         Vr(6)=Vr(5);
%     end
    
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




