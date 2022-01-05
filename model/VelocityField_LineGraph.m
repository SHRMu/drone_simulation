function VelocityField_LineGraph

    droneR=0.1; % drone r
    dangerR=0.5; % danger zone R
    w=1.5; % sensor zone wR

    k = -1;
    b = 1;
    
    obs_1 = [0 1];
    obs_2 = [1 0];
    obs = 0.5*obs_1 + 0.5*obs_2;
    
    goal = [2 2];
    
    [X,Y] = meshgrid(-1:0.1:3);
    
    dim = size(X);
    dis2o = zeros(dim);
    dis2t = zeros(dim);
    
    angle_t = zeros(dim);
    angle_o = zeros(dim);
    angle_g = zeros(dim);

    for xi = 1:dim
        for yi = 1:dim
            dis2t(xi,yi) = norm([X(xi,yi) Y(xi,yi)] - goal(1,:));
            % case 1 base on obs_1 and obs_2
            dis2o1 = norm([X(xi,yi) Y(xi,yi)] - obs_1(1,:));
            dis2o2 = norm([X(xi,yi) Y(xi,yi)] - obs_2(1,:));
            if dis2o1 <= w*dangerR && Y(xi,yi) >= X(xi,yi)+1
                vec2t = goal(1,:) - [X(xi,yi) Y(xi,yi)];
                angle_t(xi,yi) = rad2deg(atan2(vec2t(1,2), vec2t(1,1)));
                
                dis2o(xi,yi) = dis2o1;
                vec2o =  [X(xi,yi) Y(xi,yi)] - obs_1(1,:); % direction from obs to pos
                angle_o(xi,yi) = rad2deg(atan2(vec2o(1,2), vec2o(1,1)));
                theta_g = angleConversion(angle_o(xi,yi) -90);
                if abs(theta_g - angle_t(xi,yi)) <= 90
                    angle_g(xi,yi) = theta_g;
                else
                    angle_g(xi,yi) = angleConversion(theta_g + 180);
                end
            elseif dis2o2 <= w*dangerR && Y(xi,yi) <= X(xi,yi)-1
                vec2t = goal(1,:) - [X(xi,yi) Y(xi,yi)];
                angle_t(xi,yi) = rad2deg(atan2(vec2t(1,2), vec2t(1,1)));
                
                dis2o(xi,yi) = dis2o2;
                vec2o =  [X(xi,yi) Y(xi,yi)] - obs_2(1,:); % direction from obs to pos
                angle_o(xi,yi) = rad2deg(atan2(vec2o(1,2), vec2o(1,1)));
%                 theta_g = angleConversion(angle_o(xi,yi) -90);
%                 if abs(theta_g - angle_t(xi,yi)) <= 90
%                     angle_g(xi,yi) = theta_g;
%                 else
%                     angle_g(xi,yi) = angleConversion(theta_g + 180);
%                 end
            else
                % dis2o case 2 base on obs_on
                y1= Y(xi,yi);
                x1= X(xi,yi);
                x2 = (k*y1+x1-k*b)/(1+k*k);
                y2 = k*x2+b;
                if x2>= 0 && x2 <= 1
                    obs_on = [x2 y2];
                elseif x2 <0
                    obs_on = obs_2;
                elseif x2>1
                    obs_on = obs_1;
                end
                dis2o(xi,yi) = norm([X(xi,yi) Y(xi,yi)] - obs_on(1,:));
                vec2t = goal(1,:) - [X(xi,yi) Y(xi,yi)];
                angle_t(xi,yi) = rad2deg(atan2(vec2t(1,2), vec2t(1,1)));
            
                if dis2o(xi,yi) <= w*dangerR
                    vec2o =  [X(xi,yi) Y(xi,yi)] - obs_on(1,:); % direction from obs to pos
                    angle_o(xi,yi) = rad2deg(atan2(vec2o(1,2), vec2o(1,1)));
%                     theta_g = angleConversion(angle_o(xi,yi) -90);
    %                 if abs(theta_g - angle_t(xi,yi)) <= 90
    %                     angle_g(xi,yi) = theta_g;
    %                 else
    %                     angle_g(xi,yi) = angleConversion(theta_g + 180);
    %                 end
                end
            end
        end
    end
    
    alpha = 0*(dis2o<=droneR)+0*(dis2o>droneR&dis2o<=dangerR)+(1-cos(pi*(dis2o-dangerR)/(w*dangerR-dangerR)))/2.*(dis2o>dangerR&dis2o<=w*dangerR)+1*(dis2o>w*dangerR);
    beta = 1*(dis2o<=droneR)+(1+cos(pi*(dis2o-droneR)/(dangerR-droneR)))/2.*(dis2o>droneR&dis2o<=dangerR)+0*(dis2o>dangerR&dis2o<=w*dangerR)+0*(dis2o>w*dangerR);
    gamma = 0*(dis2o<=droneR)+(1-cos(pi*(dis2o-droneR)/(dangerR-droneR)))/2.*(dis2o>droneR&dis2o<=dangerR)+(1+cos(pi*(dis2o-dangerR)/(w*dangerR-dangerR)))/2.*(dis2o>dangerR&dis2o<=w*dangerR)+0*(dis2o>w*dangerR);
    
    for xi=1:dim
        for yi=1:dim
            % if the obstacle and target in the opposite sides
            if abs(angle_t(xi,yi) - angle_o(xi,yi)) < 90
                alpha(xi,yi) = 1;
                gamma(xi,yi) = 0;
            end
        end
    end
    
%     a_scale = 100;
%     attr = a_scale.*sqrt((X-goal(1)).^2+(Y-goal(2)).^2); % attr_potential
%     normA = attr - min(attr(:));
%     attr = normA ./ max(normA(:)); %
%     r_scale = 100;
%     repu = r_scale./dis2o;
%     repu(repu>500) = 500; 
%     normA = repu - min(repu(:));
%     repu = normA ./ max(normA(:)); %
%     g_scale = 100;
%     guide = g_scale*gamma;
%     
%     total = attr + repu + guide;
%     normA = total - min(total(:));
%     Z = normA ./ max(normA(:)); % 
%     figure;
%     surf(X,Y,Z);
    
    dx = alpha.*cosd(angle_t) + beta.*sind(90-angle_o) + gamma.*cosd(angle_g);
    dy = alpha.*sind(angle_t) + beta.*cosd(90-angle_o) + gamma.*sind(angle_g);

    theta = rad2deg(atan2(dy,dx));
    normA = dis2t - min(dis2t(:)); % normlization
    normA = normA ./ max(normA(:));
    u = 10.*cosd(theta).*normA;
    v = 10.*sind(theta).*normA;

    figure
    quiver(X,Y,u,v);
    hold on;
    plot(obs_1,obs_2,'LineWidth',5);
%     plot(obs(1,1), obs(1,2),'d');
    plot(goal(1,1), goal(1,2),'*');
    xlabel('X(m)');
    ylabel('Y(m)');
end


%limite input theta within [-180,180)
function theta = angleConversion(theta)
    theta = theta + 180;
    while theta >= 360 || theta < 0
        multiple = floor(theta/360);
        theta = theta - (multiple)*360;
    end
    theta = theta - 180;
end