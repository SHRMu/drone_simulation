function [Vt] = CalcFieldVelocity(x,model,goal,zoneParams,obs_on,obs_dr,dR)
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
    Twt = angularConversion(Twt);
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
        gamma = 0;
    elseif dis > dR && dis <= zoneParams(2)
        alpha = 0;
        beta = 0.5*(1+cos(pi*(dis-dR)/(zoneParams(2)-dR)));
        gamma = 0.5*(1-cos(pi*(dis-dR)/(zoneParams(2)-dR)));
    elseif dis > zoneParams(2) && dis <= zoneParams(3)
        % original 
%         alpha = 0.5*(1-cos(pi*(dis-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
%         beta = 0;
%         gamma = 0.5*(1+cos(pi*(dis-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
        % improved
        if abs(angleConversion(theta_t - theta_o)) < 90
            alpha = 1;
            beta = 0;
            gamma = 0;
        else
            alpha = 0.5*(1-cos(pi*(dis-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
            beta = 0;
            gamma = 0.5*(1+cos(pi*(dis-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
        end
    else
        alpha = 1;
        beta = 0;
        gamma = 0;
    end

%     Vm = model(1)*dis/zoneParams(3);
    Vm = model(1);
    
    V_t = alpha*Vm;
    V_o = beta*Vm;
    V_g = gamma*Vm;

    %Fixed counterclockwise
    theta_g = angleConversion(theta_o + 90);
    
%     Flexible direction
%     if abs(angleConversion(theta_t-theta_g)) > 90
%         theta_g = angleConversion(theta_o-90);
%     end

    delta_t = angleConversion(theta_t - yaw);
    delta_o = angleConversion(theta_o - yaw);
    delta_g = angleConversion(theta_g - yaw);

    vtx = V_t*cos(deg2rad(delta_t))+V_o*cos(deg2rad(delta_o))+V_g*cos(deg2rad(delta_g));
    vty = V_t*sin(deg2rad(delta_t))+V_o*sin(deg2rad(delta_o))+V_g*sin(deg2rad(delta_g));

    vt = [vtx vty];

end


