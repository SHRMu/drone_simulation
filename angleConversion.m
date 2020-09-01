%limite input theta within [-180,180)
function theta = angleConversion(theta)
    theta = theta + 180;
    while theta >= 360 || theta < 0
        multiple = floor(theta/360);
        theta = theta - (multiple)*360;
    end
    theta = theta - 180;
end