function velocityParam
    r=0.1; % drone r
    R=0.5; % danger zone R
    w=2; % sensor zone wR
    step = 0.01;
    space = 2.5;
    % x=0:0.01:10;
    % y=x.*(x>=0&x<4)+2*(x>=4&x<6)+(5-x/2).*(x>=6&x<8)+1*(x>=8);
    di=r:step:space;
    alpha=0*(di<=r)+0*(di>r&di<=R)+(1-cos(pi*(di-R)/(w*R-R)))/2.*(di>R&di<=w*R)+1*(di>w*R);
    beta = 1*(di<=r)+(1+cos(pi*(di-r)/(R-r)))/2.*(di>r&di<=R)+0*(di>R&di<=w*R)+0*(di>w*R);
    gamma = 0*(di<=r)+(1-cos(pi*(di-r)/(R-r)))/2.*(di>r&di<=R)+(1+cos(pi*(di-R)/(w*R-R)))/2.*(di>R&di<=w*R)+0*(di>w*R);
    plot(di,alpha,'r','linewidth',1.5);
    hold on;  
    plot(di,beta,'b','linewidth',1.5);
    hold on;
    plot(di,gamma,'g','linewidth',1.5);
    hold on;
    legend('Vt','Vo','Vg');
    xlabel(' 0 < di < 2.5m')
    grid on
end