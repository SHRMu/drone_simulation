%limite input theta within (-180,180]
function theta = AngleConversion(theta)
    while theta <= -180 || theta > 180
       multiple = floor(theta/180);
       theta = theta - (multiple)*180;
    end
end