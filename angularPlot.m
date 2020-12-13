function angularPlot
    
    vx = 0:0.1:1;
    vy = -1:0.1:1;
    
    dt = 0.1;
    
    [Vx,Vy]=meshgrid(vx,vy);
    [M,N] = size(Vx);
    Wt = zeros(M,N);
    
    A = [cosd(0) -sind(0);
         sind(0)  cosd(0)];
    
    for row = 1:M
        delta = A*[Vx(row,:);Vy(row,:)];
        delta_x = delta(1,:);
        delta_y = delta(2,:);
        for col = 1:N
            delta_th = rad2deg(atan2(delta_y(1,col),delta_x(1,col)))-0;
%             delta_th = angleConversion(rad2deg(atan2(delta_y(1,col),delta_x(1,col)))-0);
            Wt(row,col) = deg2rad(delta_th)/dt;
        end
    end
    
    figure;
    grid on;
    
    surf(Vx,Vy,Wt);
    xlabel("Vx(m/s)");
    ylabel("Vy(m/s)");
    zlabel('Angular(rad/s)');
    
end