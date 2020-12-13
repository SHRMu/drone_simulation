function PotentialFieldGraph % without any obstacle
%     [X,Y] = meshgrid(1:0.1:12);
%     % target 
%     xp = 1;
%     yp = 1;
% 
%     Z = 0.1*((X-xp).^2 + (Y-yp).^2);
%     C = X.*Y;
%     surfc(X,Y,Z,C)
%     colorbar
    map = imread('obs_points.png');
    map = rgb2gray(map);
%     img1 = 255-(map-imerode(map,ones(3)));  %???
%     figure;
    imshow(map);
    MapSize = size(map);
    [X,Y] = meshgrid(1:MapSize(1),1:MapSize(2));
    a_scale = 1;
    dest = [200 200];
    attr_potential = a_scale*sqrt((X-dest(1)).^2+(Y-dest(2)).^2);
    rho = (bwdist(map)/100)+1;
    influence = 1.5;
    r_scale = 1000;
    rep_potential = r_scale*((1./rho-1./influence).^2);
    rep_potential(rho>influence) = 0;
    potential_field = attr_potential + rep_potential;
    [u,v] = gradient(potential_field);
    quiver(X,Y,u,v);
end
