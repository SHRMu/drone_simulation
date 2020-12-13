function VelocityField_PointGraph

    droneR=0.1; % drone r
    dangerR=0.5; % danger zone R
    w=2.5; % sensor zone wR

    obs = [0 0]; 
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
            dis2o(xi,yi) = norm([X(xi,yi) Y(xi,yi)] - obs(1,:));
            vec2t = goal(1,:) - [X(xi,yi) Y(xi,yi)];
            angle_t(xi,yi) = rad2deg(atan2(vec2t(1,2), vec2t(1,1)));
            
            if dis2o(xi,yi) <= w*dangerR
                vec2o =  [X(xi,yi) Y(xi,yi)] - obs(1,:); % direction from obs to pos
                angle_o(xi,yi) = rad2deg(atan2(vec2o(1,2), vec2o(1,1)));
                theta_g = angleConversion(angle_o(xi,yi) -90);
                if abs(theta_g - angle_t(xi,yi)) <= 90
                    angle_g(xi,yi) = theta_g;
                else
                    angle_g(xi,yi) = angleConversion(theta_g + 180);
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
    plot(obs(1,1), obs(1,2),'d');
    plot(goal(1,1), goal(1,2),'*');
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