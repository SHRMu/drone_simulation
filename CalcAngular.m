function angular = CalcAngular(x,Vx,Vy)
    global dt;

    yaw = rad2deg(x(3));
    A = [cos(deg2rad(yaw)) -sin(deg2rad(yaw));
         sin(deg2rad(yaw))  cos(deg2rad(yaw))];
    delta = A*[Vx;Vy];
    delta_x = delta(1,1)*dt;
    delta_y = delta(2,1)*dt;
    delta_th = angleConversion(rad2deg(atan2(delta_y,delta_x))-yaw);
    angular = deg2rad(delta_th)/dt;
    
end

