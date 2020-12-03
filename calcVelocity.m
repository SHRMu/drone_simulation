function [Vt] = calcVelocity(x,model,goal,zoneParams,obs_on,obs_off,Ovir,drones,R,T)

    % no obs within sensor zone
    if isempty(obs_on) && isempty(Ovir)
%         Tvt = model(1);
        theta_t = angleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1)))); % theta2target
        yaw = angleConversion(toDegree(x(3)));
%         delta_t = angleConversion(yaw - theta_t);
        delta_t = angleConversion(theta_t - yaw);
        Tvtx = model(1)*cos(deg2rad(delta_t));
        Tvty = model(1)*sin(deg2rad(delta_t));
%         Twt = (deg2rad(theta_t)-x(3))/T;   
    else
        [Tvtx,Tvty] = calc(x,model,goal,zoneParams,obs_on,obs_off,Ovir,R,T);
    end
    
%     tmp = angleConversion(rad2deg(Twt));
%     if tmp*Twt < 0
%         Twt = deg2rad(tmp-180);
%     else
%         Twt = deg2rad(tmp);
%     end
    Vt = [Tvtx,Tvty]';
    
end

function [Tvtx,Tvty] = calc(x,model,goal,zoneParams,obs_on,obs_off,Ovir,r,T)
    vt_mat = [];
    obs_on = [obs_on;Ovir];
    if ~isempty(obs_on)
        for io=1:length(obs_on(:,1))
    %         for io=1:1
            di=norm(obs_on(io,:)-x(1:2)');

            theta_t = angleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
            theta_o = angleConversion(toDegree(atan2(obs_on(io,2)-x(2),obs_on(io,1)-x(1)))-180);

            yaw = angleConversion(toDegree(x(3)));


            if di < r
                alpha = 0;
                beta = 1;
                gamma = 0;
            elseif di > r && di <= zoneParams(2)
                alpha = 0;
                beta = 0.5*(1+cos(pi*(di-r)/(zoneParams(2)-r)));
                gamma = 0.5*(1-cos(pi*(di-r)/(zoneParams(2)-r)));
            elseif di > zoneParams(2) && di <= zoneParams(3)
                if abs(angleConversion(theta_t - theta_o)) < 90
                    alpha = 1;
                    beta = 0;
                    gamma = 0;
%                 elseif abs(angleConversion(theta_t - yaw)) > 90
%                     alpha = 0;
%                     beta = 0;
%                     gamma = 1;
                else
                    alpha = 0.5*(1-cos(pi*(di-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
                    beta = 0;
                    gamma = 0.5*(1+cos(pi*(di-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
                end
            else
                alpha = 1;
                beta = 0;
                gamma = 0;
            end

            V_t = alpha*model(1);
            V_o = beta*model(1);
            V_g = gamma*model(1);
            
%             if abs(angleConversion(theta_o - 90)-theta_t) < 90
%                 theta_g = angleConversion(theta_o - 90);
%             else
%                 theta_g = angleConversion(theta_o + 90);
%             end
            theta_g = angleConversion(theta_o + 90);

%             if abs(angleConversion(theta_g - yaw)) > 90
%                 theta_g = angleConversion(theta_g + 180);
%             end

            delta_t = angleConversion(theta_t - yaw);
            delta_o = angleConversion(theta_o - yaw);
            delta_g = angleConversion(theta_g - yaw);
            
            vtx = V_t*cos(deg2rad(delta_t))+V_o*cos(deg2rad(delta_o))+V_g*cos(deg2rad(delta_g));
            vty = V_t*sin(deg2rad(delta_t))+V_o*sin(deg2rad(delta_o))+V_g*sin(deg2rad(delta_g));

%             vtx = V_t*cos(toRadian(theta_t))+V_o*cos(toRadian(theta_o))+V_g*cos(toRadian(theta_g));
%             vty = V_t*sin(toRadian(theta_t))+V_o*sin(toRadian(theta_o))+V_g*sin(toRadian(theta_g));
            
            vt_mat = [vt_mat;vtx vty];
        end
    else
        V_t = model(1);
        theta_t = angleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
        vtx = V_t*cos(toRadian(theta_t));
        vty = V_t*sin(toRadian(theta_t));
        vt_mat = [vt_mat;vtx vty];   
    end
    
%     if ~isempty(drones)
%         [M,~] = size(drones);
%         obs_on = [];
%         for i = 1:M
%             Vangle = angleConversion(toDegree(atan2(vty,vtx)));
%             Dangle = angleConversion(toDegree(atan2(drones(i,2)-x(2),drones(i,1)-x(1))));
%             theta = angleConversion(Vangle-Dangle);
%             if abs(theta) < 90
% %                 obs_on = [obs_on;drones(i,:)];
%                 vt_mat = [0 0];
%             end
%         end
%     end
    
    [M,~] = size(vt_mat);
    if M > 1
        vt_mat = mean(vt_mat);
    end
    
    Tvtx = vt_mat(1);
    Tvty = vt_mat(2);
%     Tvt = sqrt(vt_mat(1)^2 + vt_mat(2)^2);
%     delta_y = Tvtx*sin(deg2rad(yaw)) + Tvty*cos(deg2rad(yaw));
%     delta_x = Tvtx*cos(deg2rad(yaw)) - Tvty*sin(deg2rad(yaw));
%     th = atan2(delta_y,delta_x);
%     Twt = (th-x(3))/T;
end


