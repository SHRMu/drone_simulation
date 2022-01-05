function PotentialField_PointGraph

    R = 0.5; % obstacle_effect_r

    obs = [0 0]; 
    goal = [2 2];
    
    [X,Y] = meshgrid(-1:0.2:3);

    a_scale = 100;
    attr = a_scale.*sqrt((X-goal(1)).^2+(Y-goal(2)).^2); % attr_potential
    
    r_scale = 100;
    di2o = sqrt((X-obs(1)).^2+(Y-obs(2)).^2);
    di2o(di2o>R) = 1;
    repu = r_scale./di2o;
    repu(repu>500) = 500;
    total = attr + repu;
%   
    [u,v] = gradient(total);

    figure
    quiver(X,Y,-u,-v);
    hold on;
    plot(obs(1,1), obs(1,2),'d');
    plot(goal(1,1), goal(1,2),'*');
    xlabel('X(m)');
    ylabel('Y(m)');
    normA = total - min(total(:));
    Z = normA ./ max(normA(:)); % *
    figure;
    surfc(X,Y,Z);
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Potential value');
end
