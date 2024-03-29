function [] = DynamicWindowApproachSample()
 
    close all;
    clear all;
    clc;
    disp('Dynamic Window Approach sample program start!!')
    
    quad_init_x=0;%init pos
    quad_init_y=0;
    goal=[10,10];% target [x(m),y(m)]
    x=[quad_init_x quad_init_y atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0]';%init status[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]

    % obstacles pos [x(m) y(m)]
    
% %     obstacle = [
% %                 2 4;
% %                 2.5 3.5;
% %                 3 3;
% %                 3.5 2.5;
% %                 4 2;
% %                 ];
% %             
%     obstacle = [
%                1 2;
%                1.5 2;
%                2 2;
%                2.5 2;
%                3 2;
%                6 4;
%                6 4.5;
%                6 5;
%                6 5.5;
%                6 6;
%                6 6.5;
%                8 8;
%                ];

    obstacle = [
                 4 6;
                 4.5 6;
                 5 6;
                 5.3 5.7;
                 5.5 5.5;
                 5.7 5.3;
                 6 5;
                 6 4.5;
                 6 4;
                 6 3.5;
                 ];

%     obstacle = [ 
%                   2 2;
%                   4 6;
%                   4.5 6;
%                   5 6;
%                   5.5 6;
%                   6 6;
%                   6 5.5;
%                   6 5;
%                   6 4.5;
%                   6 4;
%                   6 3.5;
%                   6 3;
%                   ];

           
%     obstacle=[0 2;
%               4 2;
%               4 4;
%               5 4;
%               5 5;
%               5 6;
%               5 9
%               8 8
%               8 9
%               7 9];

    obstacleR=0.5;
    global dt; dt=0.5;% time[s]

    % max_v[m/s],max_w[rad/s],acc_v[m/ss],acc_w[rad/ss],
    % resolution_v[m/s],Resolution_w[rad/s]]
    Kinematic=[1.0,deg2rad(90.0),1.0,deg2rad(90.0),0.1,deg2rad(1)];
    % [heading,dist,velocity,predictDT]
    evalParam=[0.05,0.5,0.1,1.0];
%     % zone params
%     dangerR = 1.0; % paranmeter R
%     omega = 2.0;   % parameter omega
%     zoneParam = [dangerR,omega*dangerR];
    
    area=[-1 12 -1 12];% sumulation area [xmin xmax ymin ymax]

    result.x=[];
    
%     writerObj=VideoWriter('simulation.avi');   
%     open(writerObj); 
    obs = linkObstacles(obstacle,0.5); 
    tic;
%     movcount=0;
    % Main loop
    
    for i=1:5000
        % DWA
        [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
        x=f(x,u);

        result.x=[result.x; x'];

        % 
        if norm(x(1:2)-goal')< obstacleR
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.5;% 
        % 
        quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
        plot(result.x(:,1),result.x(:,2),'-b');hold on;
        plot(goal(1),goal(2),'*r');hold on;
%         plot(obstacle(:,1),obstacle(:,2),'d');hold on;
        
%         [M,~] = size(obs);
%         if M >= 2
            plotObstacles(obs,0.5);hold on; 
%         else
%             plot(obs(:,1),obs(:,2),'d');hold on;
%         end
       	
        % plot danger and sensor zone
%         for io=1:length(obstacle(:,1))
%             plotCircle(obstacle(io,1),obstacle(io,2),zoneParam);hold on;
%         end
        % traj
        if ~isempty(traj)
            for it=1:length(traj(:,1))/5
                ind=1+(it-1)*5;
                plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
            end
        end
        axis(area);
        grid on;
        xlabel("X(m)");
        ylabel("Y(m)");
        drawnow;
%         movcount=movcount+1;
%         mov(movcount) = getframe(gcf);% 
%         frame = getframe;              
%         writeVideo(writerObj,frame); 
    end
%     close(writerObj); 
    toc
%     movie2avi(mov,'simulation.avi');
end
 
function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
    % Dynamic Window [vmin,vmax,wmin,wmax]
    Vr=CalcDynamicWindow(x,model);
    % 
    [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);
    if isempty(evalDB)
        disp('no path to goal!!');
        u=[0;0];return;
    end
    %
    evalDB=NormalizeEval(evalDB);
    %
    feval=[];
    for id=1:length(evalDB(:,1))
        feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
    end
    evalDB=[evalDB feval];

    [maxv,ind]=max(feval);%
    u=evalDB(ind,1:2)';% 
end

function Vr=CalcDynamicWindow(x,model)
    % 
    global dt;
    % Maximum and minimum range of velocity
    Vs=[0 model(1) -model(2) model(2)];

    % Dynamic window calculated based on current velocity and acceleration limit
    Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

    % Dynamic Window
    Vtmp=[Vs;Vd];
    Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];

end

function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
    % 
    evalDB=[];
    trajDB=[];
    for vt=Vr(1):model(5):Vr(2)
        for ot=Vr(3):model(6):Vr(4)
            % 
            [xt,traj]=GenerateTrajectory(x,model,vt,ot,evalParam(4));  %evalParam(4),??????;
            heading=CalcHeadingEval(xt,goal);
            dist=CalcDistEval(xt,ob,R);
            vel=abs(vt);
            stopDist=CalcBreakingDist(vel,model);
            if dist>stopDist % 
                evalDB=[evalDB;[vt ot heading dist vel]];
                trajDB=[trajDB;traj];
            end
        end
    end
end

function EvalDB=NormalizeEval(EvalDB)
    if sum(EvalDB(:,3))~=0
        EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
    end
    if sum(EvalDB(:,4))~=0
        EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
    end
    if sum(EvalDB(:,5))~=0
        EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
    end
end


function stopDist=CalcBreakingDist(vel,model)
    global dt;
    stopDist=0;
    while vel>0
        stopDist=stopDist+vel*dt;% ???????
        vel=vel-model(3)*dt;% 
    end
end 
function dist=CalcDistEval(x,ob,R)
    % ?????????
    dist=100;
    for io=1:length(ob(:,1))
        disttmp=norm(ob(io,:)-x(1:2)')-R;
        if dist>disttmp
            dist=disttmp;
        end
    end

    if dist>=2*R
        dist=2*R;
    end
end
 
function heading=CalcHeadingEval(x,goal)
    % heading???????

    theta=rad2deg(x(3));% ?????
    goalTheta=rad2deg(atan2(goal(2)-x(2),goal(1)-x(1)));% ??????

    if goalTheta>theta
        targetTheta=goalTheta-theta;% [deg]
    else
        targetTheta=theta-goalTheta;% [deg]
    end

    heading=180-targetTheta;
end

function x = f(x, u)
    % Motion Model
    % [xi,yi,yaw,v,w,dis]
    % u = [vt; wt]; current velocity, angular_velocity
    global dt;

    F = [1 0 0 0 0 
         0 1 0 0 0 
         0 0 1 0 0 
         0 0 0 0 0 
         0 0 0 0 0];

    B = [dt*cos(x(3)) 0
        dt*sin(x(3)) 0
        0 dt
        1 0
        0 1];

    x= F*x+B*u;
end

function [x,traj]=GenerateTrajectory(x,model,vt,wt,evaldt)
    global dt;
    time=0;
    u=[vt;wt];% 
    traj=x;% 
    while time<=evaldt
        time=time+dt;% 
        x=f(x,u);% 
        traj=[traj x];
    end
end
 
