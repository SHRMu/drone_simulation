function res = angularConversion(angular)
    if angular > 2*pi
        angular = angular - 2*pi;
        while angular > 2*pi
             angular = angular - 2*pi;
        end
    elseif angular < -2*pi
        angular = angular + 2*pi;
        while angular < -2*pi
             angular = angular + 2*pi;
        end
    end
    res = angular;
end