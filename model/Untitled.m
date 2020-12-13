
clear all;
close all;
clc;

img = imread('test.png');
img = rgb2gray(img);
imshow(img);
[h,w] = size(img);

img2gray = 255-(img-imerode(img,ones(3)));  % border test
figure;
imshow(img2gray);

p = ginput(); % select start and target point
hold on;
plot(p(:,1),p(:,2),'r.')

attr = zeros(h*w,1);                    % attractive_potential
repu = zeros(h*w,1);                    % repulive_potential

a_scaler = 2;
r_scaler = 5;

ind=zeros(h*w,2);
indobs=zeros(sum(sum((255-img2gray)>0)),2);
num = 1;
for i=1:h
    for j=1:w
        ind((i-1)*w+j,:) = [j i];
        if img2gray(i,j) ==0   
            indobs(num,:)=[j i];
            num = num + 1;
        end
        d = norm([j,i]-p(2,:));             % distance to target
        attr((i-1)*w+j) = a_scaler*d;       
    end
end

for i=1:length(indobs)
    t = ind - repmat(indobs(i,:),length(ind),1);    
    repu = repu + r_scaler./sqrt(t(:,1).^2+t(:,2).^2);       
end

repu(repu>500) = 500;
total = attr + repu;   

imgre = reshape(total,[h,w])';

[X,Y] = meshgrid(1:h,1:w);
[u,v] = gradient(imgre);
quiver(X,Y,u,v);

nei=[ -1 -1; -1 0; -1 1;
       0 -1; 0 0;  0 1;
       1 -1; 1 0;  1 1];
path=floor(p(1,:));
pre = [0 0];
while norm(path(end,:)-p(2,:))> 2    
    pc = path(end,:);
    im = imgre(pc(end,2)-1:pc(end,2)+1,pc(end,1)-1:pc(end,1)+1);
    [~,ind] = min(reshape(im,9,1));         
    
    pre = path(end,:);   
    path = [path;path(end,:)+nei(ind,:)];
    
    if  norm(path(end,:)-pre) == 0 || ...           
        path(end,1)==1 || path(end,2) ==1 || ...
        path(end,1)==w || path(end,2) ==h
        break;
    end
end

hold on;
plot(path(:,1),path(:,2));
figure;
mesh(imgre);