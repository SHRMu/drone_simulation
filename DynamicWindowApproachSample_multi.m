% -------------------------------------------------------------------------
%
% File : DynamicWindowApproachSample.m
%
% Discription : Mobile Robot Motion Planning with Dynamic Window Approach
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------
 
function [] = DynamicWindowApproachSample_multi()
 
    close all;
    clear all;
    clc;
    disp('Dynamic Window Approach sample program start!!')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num=2;%drone num
    scale = 3*num;
    radius = 0.5*scale;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% circle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    centre = radius*ones(1,2);
    theta = 360/num;
%     quad_init_x=scale*rand(num,1);%init pos
%     quad_init_y=scale*rand(num,1);
%     goal = 0.8*scale*rand(num,2);
    quad_init_x = zeros(num,1);
    quad_init_y = zeros(num,1);
    goal = zeros(num,2);
    
    for i = 1:num
        quad_init_x(i,1) = centre(1,1) + cos(deg2rad(i*theta))*radius;
        quad_init_y(i,1) = centre(1,2) + sin(deg2rad(i*theta))*radius;
        goal(i,1:2) = [centre(1,1) + cos(deg2rad(i*theta+180))*radius centre(1,1) + sin(deg2rad(i*theta+180))*radius];
    end

    x=[quad_init_x(1,1) quad_init_y(1,1) atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0;
       quad_init_x(2,1) quad_init_y(2,1) atan2((goal(2,2)-quad_init_y(2,1)),(goal(2,1)-quad_init_x(2,1))) 0 0;
%        quad_init_x(3,1) quad_init_y(3,1) atan2((goal(3,2)-quad_init_y(3,1)),(goal(3,1)-quad_init_x(3,1))) 0 0;
%        quad_init_x(4,1) quad_init_y(4,1) atan2((goal(4,2)-quad_init_y(4,1)),(goal(4,1)-quad_init_x(4,1))) 0 0;
        ]';%init status[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]

    % obstacles pos [x(m) y(m)]

    obstacleR=0.5;
    global dt; dt=0.5;% time[s]

    % max_v[m/s],max_w[rad/s],acc_v[m/ss],acc_w[rad/ss],
    % resolution_v[m/s],Resolution_w[rad/s]]
    Kinematic=[1,toRadian(90.0),1,toRadian(40.0),0.01,toRadian(1)];
    % [heading,dist,velocity,predictDT]
    evalParam=[0.05,0.2,0.1,0.5];
    % zone params
    dangerR = 1; % paranmeter R
    omega = 2.0;   % parameter omega
    zoneParam = [dangerR,omega*dangerR];
    
    area=[-1 12 -1 12];% sumulation area [xmin xmax ymin ymax]

    result.x1=[];
    result.x2=[];
    result.x3=[];
    result.x4=[];
    
    tic;
    % movcount=0;
    % Main loop
    
    for i=1:5000
        obstacle1 = [x(1,2) x(2,2)];
        obstacle2 = [x(1,1) x(2,1)];
%         obstacle1 = [x(1,2) x(2,2);x(1,3) x(2,3);x(1,4) x(2,4)];
%         obstacle2 = [x(1,1) x(2,1);x(1,3) x(2,3);x(1,4) x(2,4)];
%         obstacle3 = [x(1,1) x(2,1);x(1,2) x(2,2);x(1,4) x(2,4)];
%         obstacle4 = [x(1,1) x(2,1);x(1,2) x(2,2);x(1,3) x(2,3)];
        % DWA
        [u1,traj1]=DynamicWindowApproach(x(:,1),Kinematic,goal(1,:),evalParam,obstacle1,obstacleR);
        x(:,1)=f(x(:,1),u1);
        result.x1=[result.x1; x(:,1)'];
        
        [u2,traj2]=DynamicWindowApproach(x(:,2),Kinematic,goal(2,:),evalParam,obstacle2,obstacleR);
        x(:,2)=f(x(:,2),u2);
        result.x2=[result.x2; x(:,2)'];
        
%         [u3,traj3]=DynamicWindowApproach(x(:,3),Kinematic,goal(3,:),evalParam,obstacle3,obstacleR);
%         x(:,3)=f(x(:,3),u3);
%         result.x3=[result.x3; x(:,3)'];
%         
%         [u4,traj4]=DynamicWindowApproach(x(:,4),Kinematic,goal(4,:),evalParam,obstacle4,obstacleR);
%         x(:,4)=f(x(:,4),u4);
%         result.x4=[result.x4; x(:,4)'];

        % 
        if norm(x(1:2)-goal(1,:)')< obstacleR && norm(x(2:2)-goal(2,:)')< obstacleR
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.5;% 
        % 
        quiver(x(1,1),x(2,1),ArrowLength*cos(x(3,1)),ArrowLength*sin(x(3,1)),'ok');hold on;
        plot(result.x1(:,1),result.x1(:,2),'-b');hold on;
        plot(goal(1,1),goal(1,2),'*r');hold on;
%         plot(obstacle1(:,1),obstacle1(:,2),'d');hold on;
        
        quiver(x(1,2),x(2,2),ArrowLength*cos(x(3,2)),ArrowLength*sin(x(3,2)),'ok');hold on;
        plot(result.x2(:,1),result.x2(:,2),'-b');hold on;
        plot(goal(2,1),goal(2,2),'*r');hold on;
%         plot(obstacle2(:,1),obstacle2(:,2),'d');hold on;
        
%         quiver(x(1,3),x(2,3),ArrowLength*cos(x(3,3)),ArrowLength*sin(x(3,3)),'ok');hold on;
%         plot(result.x3(:,1),result.x3(:,2),'-b');hold on;
%         plot(goal(3,1),goal(3,2),'*r');hold on;
% %         plot(obstacle3(:,1),obstacle3(:,2),'d');hold on;
%         
%         quiver(x(1,4),x(2,4),ArrowLength*cos(x(3,4)),ArrowLength*sin(x(3,4)),'ok');hold on;
%         plot(result.x4(:,1),result.x4(:,2),'-b');hold on;
%         plot(goal(4,1),goal(4,2),'*r');hold on;
%         plot(obstacle4(:,1),obstacle4(:,2),'d');hold on;
        % plot danger and sensor zone
%         for io=1:length(obstacle(:,1))
%             plotCircle(obstacle(io,1),obstacle(io,2),zoneParam);hold on;
%         end
%         traj
        if ~isempty(traj1)
            for it=1:length(traj1(:,1))/5
                ind=1+(it-1)*5;
                plot(traj1(ind,:),traj1(ind+1,:),'-g');hold on;
            end
        end
        if ~isempty(traj2)
            for it=1:length(traj2(:,1))/5
                ind=1+(it-1)*5;
                plot(traj2(ind,:),traj2(ind+1,:),'-m');hold on;
            end
        end
%         if ~isempty(traj3)
%             for it=1:length(traj3(:,1))/5
%                 ind=1+(it-1)*5;
%                 plot(traj3(ind,:),traj3(ind+1,:),'-g');hold on;
%             end
%         end
%         if ~isempty(traj4)
%             for it=1:length(traj4(:,1))/5
%                 ind=1+(it-1)*5;
%                 plot(traj4(ind,:),traj4(ind+1,:),'-m');hold on;
%             end
%         end
       
        axis(area);
        grid on;
        drawnow;
        %movcount=movcount+1;
        %mov(movcount) = getframe(gcf);% 
    end
    toc
    %movie2avi(mov,'movie.avi');
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
    Vs=[0 model(1) -model(2) model(2)]; %[0 Vm -Wm Wm ]

    % Dynamic window calculated based on current velocity and acceleration limit
    Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt]; %[]

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
            % ????????
            heading=CalcHeadingEval(xt,goal);
            dist=CalcDistEval(xt,ob,R);
            vel=abs(vt);
            % ???????
            stopDist=CalcBreakingDist(vel,model);
            if dist>stopDist % 
                evalDB=[evalDB;[vt ot heading dist vel]];
                trajDB=[trajDB;traj];
            end
        end
    end
end

function EvalDB=NormalizeEval(EvalDB)
    % ???????
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
    % ?????????????,???????????????????????
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
        disttmp=norm(ob(io,:)-x(1:2)')-R;%???????????????????
        if dist>disttmp% ?????????
            dist=disttmp;
        end
    end

    % ??????????????????????????????????????
    if dist>=2*R
        dist=2*R;
    end
end
 
function heading=CalcHeadingEval(x,goal)
    % heading???????

    theta=toDegree(x(3));% ?????
    goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% ??????

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
 
