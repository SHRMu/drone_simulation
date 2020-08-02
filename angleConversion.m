% theta limited to (-180,180]
function r_theta = angleConversion(theta)
    while theta <= -180 || theta > 180
       multiple = floor(theta/180);
       theta = theta - (multiple)*180;
    end
    r_theta = theta;
end