function [vt,wt] = calcVelocity(x,IDs,model,goal,zoneParam,drones,R,T)
 
    vt_mat = calc(x,dis,model,goal,zoneParam,obs_on,obs_off,drones,R,T);
    
    % last T param
    Otheta = x(3);
    Ovtx = cos(Otheta)*x(4);
    Ovty = sin(Otheta)*x(4);
    Owt = x(5); 
    
    % now T
    vtx = vt_mat(1);
    vty = vt_mat(2);
    
    if abs(vtx-Ovtx)/T > model(2)
        vtx = Ovtx + (vtx-Ovtx)/abs(vtx-Ovtx)*model(2);
    end
    if abs(vty-Ovty)/T > model(2)
        vty = Ovty + (vty-Ovty)/abs(vty-Ovty)*model(2);
    end
    
    theta = atan2(vty,vtx);
    vt = vtx/cos(theta);
    
    if theta ~= x(3)
        wt = (theta-x(3))/T;
        if abs(wt-Owt)/T > model(4)
            wt = Owt + (wt-Owt)/abs(wt-Owt)*model(4);
    	end 
    else
        wt = 0;
    end 
end

function vt_mat = calc(x,IDs,model,goal,zoneParam,drones,r,T)
    vt_mat = [];
    
    
    
end