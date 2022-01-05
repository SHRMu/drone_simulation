function PotentialField_LineGraph

    R = 0.5; % obstacle_effect_r

    obs_1 = [0 1];
    obs_2 = [1 0];
    
    goal = [2 2];
    
    xv = [0.5;1.5;0.5;-0.5];
    yv = [-0.5;1;1.5;0.5];
    

    [X,Y] = meshgrid(-1:0.1:3);

    a_scale = 100;
    attr = a_scale.*sqrt((X-goal(1)).^2+(Y-goal(2)).^2); % attr_potential
    
    r_scale = 100;
%     rad2deg(atan2(norm(cross(u,v)),dot(u,v)));
%     dis2o = abs(Y-1+X)./norm(obs_2-obs_1);
    [M,N] = size(X);
    dis2o = zeros(M,N);
    for row = 1:M
        for col = 1:N
            tmp = abs(Y(row,col)-1+X(row,col))/norm(obs_2-obs_1);
            if tmp <= R
                in = inpolygon(X(row,col),Y(row,col),xv,yv);
                if in > 0
                    dis2o(row,col) = tmp;
                else
                    dis2o(row,col) = 1;
                end
            else
                dis2o(row,col) = 1;
            end
        end
    end
%     dis2o(dis2o<1) = 1;
    repu = r_scale./dis2o;
    repu(repu>500) = 500;
    total = attr + repu;
%   
    [u,v] = gradient(total);

    figure
    quiver(X,Y,-u,-v);
    hold on;
    plot(obs_1(1,1), obs_1(1,2),'d');
    plot(obs_2(1,1), obs_2(1,2),'d');
    
    plot(goal(1,1), goal(1,2),'*');
    normA = total - min(total(:));
    Z = normA ./ max(normA(:)); % 
    figure;
    surf(X,Y,Z);
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Potential value');
end