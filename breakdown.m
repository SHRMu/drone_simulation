function ut = breakdown(model,vtx,vty,wt)
    global dt;
    if abs(vtx) < model(2)*dt
        Vtx = 0;
    else
        Vtx = vtx - vtx/abs(vtx)*model(2)*dt;
    end
    if abs(vty) < model(2)*dt
        Vty = 0;
    else
        Vty = vty - vty/abs(vty)*model(2)*dt;
    end
%     if abs(wt) < model(4)*dt
%         Wt = 0;
%     else
%         Wt = wt - wt/abs(wt)*model(4)*dt;
%     end
    ut=[Vtx;Vty;0];
end