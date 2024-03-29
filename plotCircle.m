function plotCircle(a,b,zoneParam)
    theta=-2*pi:0.01:2*pi;
    danger_x=zoneParam(2)*sin(theta)+a;
    danger_y=zoneParam(2)*cos(theta)+b;
    sensor_x=zoneParam(3)*sin(theta)+a;
    sensor_y=zoneParam(3)*cos(theta)+b;
    % plot danger circle
    plot(danger_x,danger_y,'r-');
    % plot sensor circle
    plot(sensor_x,sensor_y,'g:'); 
    axis equal;
end