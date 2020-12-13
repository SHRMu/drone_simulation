function Vr=CalcDynamicWindow(x,Vt,model)
    global dt;
    if abs(x(5)-0.0406) < 0.001
        tem = 0;
    end 
    % current max and min range of velocity
    Vc=[x(4)-model(2)*dt x(4)+model(2)*dt x(5)-model(2)*dt x(5)+model(2)*dt x(6)-model(4)*dt x(6)+model(4)*dt];
    % Maximum and minimum range of velocity
    Vs=[0 model(1) -model(1) model(1) -model(3) model(3)]; %[0 Vmx -Vmy Vmy -Wm Wm ]
    Vc=[max(Vc(1),Vs(1)) min(Vc(2),Vs(2)) max(Vc(3),Vs(3)) min(Vc(4),Vs(4)) max(Vc(5),Vs(5)) min(Vc(6),Vs(6))];
    
    Wt = Vt(3);
    
    if abs(Wt-0.001) <= 0.001
        if Vt(1) < Vc(1)
            Vr = [Vc(1) Vc(1) 0 0 0 0]; return;
        elseif Vt(1)> Vc(2)
            Vr = [Vc(2) Vc(2) 0 0 0 0]; return;
        else
            Vr = [Vt(1) Vt(1) 0 0 0 0]; return;
        end
    end
    
    angularMat = [CalcAngular(x,Vc(1),Vc(3)) CalcAngular(x,Vc(2),Vc(3));
                    CalcAngular(x,Vc(1),Vc(4)) CalcAngular(x,Vc(2),Vc(4))];

    
    if Wt > 0 || Wt < 0
        [Wmin,Imin] = min(angularMat(1,:));
        [Wmax,Imax] = max(angularMat(2,:));
         Vc = [Vc(1) Vc(2) Vc(3) Vc(4) max(Vc(5),Wmin) min(Vc(6),Wmax)];
         if Wmax < Vc(5) || Wmin > Vc(6) || Vc(5) > Vc(6)
             ut = breakdown(model,x(4),x(5),x(6));
             Vr = [ut(1) ut(1) ut(2) ut(2) ut(3) ut(3)];
             return;
         end
    end

    if Wt <= Vc(5)
        if Vc(5) < 0 
            n = -1;
            wt = CalcAngular(x,1,n);
            while wt < Vc(5) && n < 0
                n = n+0.01;
                wt = CalcAngular(x,1,n);
            end
            Vmin = max(Vc(1),Vc(4)/n);
            Vmax = min(Vc(2),Vc(3)/n);
            
        else
             n = 0.01;
             wt = CalcAngular(x,1,n);
             while wt < Vc(5)
                n = n+0.01;
                wt = CalcAngular(x,1,n);
             end
             Vmin = max(Vc(1),Vc(3)/n);
             Vmax = min(Vc(2),Vc(4)/n);
        end
        if Vmin > Vmax
             ut = breakdown(model,x(4),x(5),x(6));
             Vr = [ut(1) ut(1) ut(2) ut(2) ut(3) ut(3)];
             return;
        end
        Vc(1) = Vmin;
        Vc(2) = Vmax;
        if Vt(1) < Vc(1)
            Vr(1)=Vc(1);
            Vr(3)=n*Vr(1);
        elseif Vt(1) > Vc(2)
            Vr(1)=Vc(2);
            Vr(3)=n*Vr(1);
        else
            Vr(1)=Vt(1);
            Vr(3)=n*Vr(1);
        end
        Vr(2)=Vr(1);
        Vr(4)=Vr(3);

        angular = CalcAngular(x,Vr(1),Vr(3));
        if angular > 2*pi || angular < -2*pi
            angular = angularConversion(angular);
        end
        Vr(5) = angular;
        Vr(6) = Vr(5);
    elseif Wt > Vc(6)
%         Vr = [Vt(1) Vt(1) Vt(2) Vt(2) Vt(3) Vt(3)];
%         Vr = [Vc(Imax) Vc(Imax) Vc(Imax+2) Vc(Imax+2) Vc(6) Vc(6)];
%         if Vr(3)*Vr(5) < 0
%             Vr = [Vc(2) Vc(2) Vc(Imax+2) Vc(Imax+2) Vc(6) Vc(6)];
%         end
        if Vc(6) > 0
            n = 1;
            wt = CalcAngular(x,1,n); 
            while wt > Vc(6) && n > 0
                n = n-0.01;
                wt = CalcAngular(x,1,n); 
            end
            Vmin = max(Vc(1),Vc(3)/n);
            Vmax = min(Vc(2),Vc(4)/n);
        else
            n = -0.01;
            wt = CalcAngular(x,1,n); 
            while wt > Vc(6)
                n = n-0.01;
                wt = CalcAngular(x,1,n); 
            end
            Vmin = max(Vc(1),Vc(4)/n);
            Vmax = min(Vc(2),Vc(3)/n);    
        end
        if Vmin > Vmax
             ut = breakdown(model,x(4),x(5),x(6));
             Vr = [ut(1) ut(1) ut(2) ut(2) ut(3) ut(3)];
             return;
        end
        Vc(1) = Vmin;
        Vc(2) = Vmax;
        if Vt(1) < Vc(1)
            Vr(1)=Vc(1);
            Vr(3)=n*Vr(1);
        elseif Vt(1) > Vc(2)
            Vr(1)=Vc(2);
            Vr(3)=n*Vr(1);
        else
            Vr(1)=Vt(1);
            Vr(3)=n*Vr(1);
        end
        Vr(2)=Vr(1);
        Vr(4)=Vr(3);

        angular = CalcAngular(x,Vr(1),Vr(3));
        if angular > 2*pi || angular < -2*pi
            angular = angularConversion(angular);
        end
        Vr(5) = angular;
        Vr(6) = Vr(5);
    else
        if Vt(1) > Vc(1) && Vt(1) < Vc(2) && Vt(2) > Vc(3) && Vt(2) < Vc(4)
            Vr(1) = Vt(1);
            Vr(3) = Vt(2);
        else
            if Wt > 0 && abs(Vc(4)-0.0) <= 0.001
                if Vt(1) < Vc(1)
                    Vr(1)=Vc(1);
                    Vr(3)=0;
                elseif Vt(1) > Vc(2)
                    Vr(1)=Vc(2);
                    Vr(3)=0;
                else
                    Vr(1)=Vt(1);
                    Vr(3)=0;
                end
            else
                n = Vt(2)/Vt(1);
                if n > 0
                    Vmin = max(Vc(1),Vc(3)/n);
                    Vmax = min(Vc(2),Vc(4)/n);
                else
                    Vmin = max(Vc(1),Vc(4)/n);
                    Vmax = min(Vc(2),Vc(3)/n);
                end
                if Vmin > Vmax
                     ut = breakdown(model,x(4),x(5),x(6));
                     Vr = [ut(1) ut(1) ut(2) ut(2) ut(3) ut(3)];
                     return;
                end
                Vc(1) = Vmin;
                Vc(2) = Vmax;
                if Vt(1) < Vc(1)
                    Vr(1)=Vc(1);
                    Vr(3)=n*Vr(1);
                elseif Vt(1) > Vc(2)
                    Vr(1)=Vc(2);
                    Vr(3)=n*Vr(1);
                else
                    Vr(1)=Vt(1);
                    Vr(3)=n*Vr(1);
                end
            end
        end
         
        Vr(2)=Vr(1);
        Vr(4)=Vr(3);

        angular = CalcAngular(x,Vr(1),Vr(3));
        if angular > 2*pi || angular < -2*pi
            angular = angularConversion(angular);
        end
        Vr(5) = angular;
        Vr(6) = Vr(5);
    end
end