clc
clear
close all

pointdata=importdata('30_seg.xyz');  %导入数据1

% A=pointdata(1:10:end,:)*1e3;                     %单位换算
A=pointdata*1e3;
%A=pointdata;

[IX,IY]=size(A);

x=A(:,1);                            %各坐标数据
y=A(:,2);
z=A(:,3);

x_mean=(max(x)+min(x))/2;
y_mean=(max(y)+min(y))/2;
x=x-x_mean;
y=y-y_mean;
% z=z+250;
% A(:,3)=z-2.69;

ptCloud_A = pointCloud([x(:),y(:),z(:)]);

figure (1)
pcshow(ptCloud_A);

xlabel('X (mm)');% x轴名称
ylabel('Y (mm)');
zlabel('Z (mm)');
view(0,90)
colormap(jet)

%--------------------------------------------------------------------------
%点云处理，绕Z轴旋转
% gama=-atan(pointdata_nor(1,1)/pointdata_nor(1,2));
% gama_alpha=gama*180/pi;
% gama=2.6*pi/180; 
gama=(2)*pi/180;         


R_z=[cos(gama),-sin(gama),0;
     sin(gama), cos(gama),0;
             0,         0,1];

for point_num=1:IX
    B(point_num,:)=(R_z*A(point_num,:)')';
end
x=B(:,1);                                %各坐标数据
y=B(:,2);
z=B(:,3);

ptCloud_B = pointCloud(B);

figure (2)
pcshow(ptCloud_B);

xlabel('X (mm)');% x轴名称
ylabel('Y (mm)');
zlabel('Z (mm)');
colormap(jet)
view(0,90)
% figure (2)
% [X,Y,Z]=griddata(x,y,z,linspace(min(x),max(x),1000)',linspace(min(y),max(y),1000)); %构造坐标点
% mesh(X,Y,Z);           %三维曲面
% axis equal
% shading interp    % interpolate colors across lines and faces
% colormap(jet)
% xlabel('X (mm)');% x轴名称
% ylabel('Y (mm)');
% zlabel('Z (mm)');

%--------------------------------------------------------------------------
%点云处理，绕X轴旋转
alpha=-1.8*pi/180;
alpha_angle=alpha*180/pi;

R_x=[ 1,          0,           0;
      0, cos(alpha), -sin(alpha);
      0, sin(alpha),  cos(alpha);];

for point_num=1:IX
    C(point_num,:)=(R_x*B(point_num,:)')';
end
x=C(:,1);                                %各坐标数据
y=C(:,2);
z=C(:,3);

ptCloud_C = pointCloud(C);

figure (3)
pcshow(ptCloud_C);

xlabel('X (mm)');% x轴名称
ylabel('Y (mm)');
zlabel('Z (mm)');
colormap(jet)
view(0,90)

% figure (3)
% [X,Y,Z]=griddata(x,y,z,linspace(min(x),max(x),1000)',linspace(min(y),max(y),1000)); %构造坐标点
% mesh(X,Y,Z);           %三维曲面
% axis equal
% shading interp    % interpolate colors across lines and faces
% colormap(jet)
% xlabel('X (mm)');% x轴名称
% ylabel('Y (mm)');
% zlabel('Z (mm)');
%
%--------------------------------------------------------------------------
% 点云处理，绕Y轴旋转
beta=3.0*pi/180;

R_y=[  cos(beta),   0,   sin(beta);
               0,   1,           0;
      -sin(beta),   0,   cos(beta);];
  
for point_num=1:IX
    D(point_num,:)=(R_y*C(point_num,:)')';
end

x=D(:,1);                                %各坐标数据
y=D(:,2);
z=D(:,3);

ptCloud_D = pointCloud(D);

figure (4)
pcshow(ptCloud_D);

xlabel('X (mm)');% x轴名称
ylabel('Y (mm)');
zlabel('Z (mm)');
colormap(jet)
view(0,90)

save('ptCloud.txt','ptCloud_D')

% figure (4)
% [X,Y,Z]=griddata(x,y,z,linspace(min(x),max(x),1000)',linspace(min(y),max(y),1000)); %构造坐标点
% mesh(X,Y,Z);           %三维曲面
% axis equal
% shading interp    % interpolate colors across lines and faces
% colormap(jet)
% xlabel('X (mm)');% x轴名称
% ylabel('Y (mm)');
% zlabel('Z (mm)');


%---------------
%去掉底面
AA=[x,y,z];
id = AA(:,3)<-394  ;
AA(id,:) = [];

x=AA(:,1);
y=AA(:,2);
z=AA(:,3);
%--------------------------------------------------------------------------
%点云绘图

x_mean=(max(x)+min(x))/2;
y_mean=(max(y)+min(y))/2;
x=x-x_mean;
y=y-y_mean;
z=z+394;

figure (5)                               
plot3(x,y,z,'.','MarkerSize',0.5);
xlabel('X (mm)')
ylabel('Y (mm)')
zlabel('Z (mm)')
grid on
hold on;
view(90,0)

D_top=11;
% D=-226.75;               %截面高度
D=16.9;                  %截面高度

%求截平面之上的所有点，并存储在slice_x,slice_y,slice_z
j=1;
for i=1:size(z)
     if z(i)>D %&& z(i)<(D+0.1)
         slice_x(j,1)=x(i,1);     
         slice_y(j,1)=y(i,1);
         slice_z(j,1)=z(i,1);
         j=j+1;        
     end
end


%截面上的点
slice_plan_z=D*ones(size(slice_x));
plot3(slice_x,slice_y,slice_plan_z,'.','MarkerSize',0.5);

%-----------------------------------------------------
%截面上的点数据压缩
Pp=[slice_x,slice_y];
% Pp=[slice_y,slice_x];


% point_simplify=Pp(1:5:end,:);
% figure (7) 
% plot(point_simplify(:,1),point_simplify(:,2), '.')
% xlabel('X (mm)');% x轴名称
% ylabel('Y (mm)');
% axis equal

figure (8) 
plot(Pp(:,1),Pp(:,2), '.')
xlabel('X (mm)');% x轴名称
ylabel('Y (mm)');
axis equal

[xc,yc,seg,movelist]= DBSCAN_cluster(Pp);
% z_slices=[D-1,D];

figure
% plot3(x,y,z,'.','MarkerSize',0.5);
% xlabel('X (mm)')
% ylabel('Y (mm)')
% zlabel('Z (mm)')
% grid on
ptCloud_E = pointCloud([x(:),y(:),z(:)]);

pcshow(ptCloud_E)

hold on;
% plot_slices(movelist,z_slices, 0.02)

% plot3(xc, yc, D*ones(size(xc,2)),'r', seg(:,1), seg(:,2), D*ones(size(seg,1)), 'r');
plot3( seg(:,1), seg(:,2), D*ones(size(seg,1)), 'r');
xlabel('X (mm)')
ylabel('Y (mm)')
zlabel('Z (mm)')
title(['Height z = ','{',num2str(D),'}'])
% save('point_simplify_1.mat','point_simplify')
% save('ptCloud.mat','ptCloud')
% save('Pp.mat','Pp')
%------------------------------------------------------

% 
% 
% %求截平面D处的轮廓点
% k = boundary(slice_x,slice_y);
% 
% %对应的截面数据
% num=size(slice_x(k));
% plan_z=D*ones(num);
% 
% %绘制截面点云
% plot3(slice_x(k),slice_y(k),plan_z,'LineWidth',2);
% 
% point_data_f=[slice_x,slice_y];
% S = alphaShape(point_data_f);
% % S = alphaShape with properties:
% %              Points: [733×2 double]
% %               Alpha: 0.035248
% %       HoleThreshold: 0
% %     RegionThreshold: 0
%    
% figure (6)
% plot(1:size(slice_z),slice_z) %观察截面点与截面D的重合程度
% 
% 
% 
% plot(S) 
% 
% x_space=2;    %扫描点间隔
% y_space=2;
% %======================================================方向没优化之前的
% 
% %构造坐标点
% [xq,yq] = meshgrid(min(slice_x):x_space:max(slice_x), min(slice_y):y_space:max(slice_y));  
% %散乱点插值函数 
% [am_x,am_y,am_z]=griddata(slice_x,slice_y,D*ones(size(slice_x)),xq,yq);                  
% 
% m=1;
%  for i_ind=1:size(am_x,1)
%      if mod(i_ind,2)==0  
%         for j_ind=1:size(am_x,2)
%             if abs(am_z(i_ind,j_ind))>0               %确保数据不是非空
%                 x_path(m,1)=am_x(i_ind,j_ind);
%                 y_path(m,1)=am_y(i_ind,j_ind);
%                 m=m+1;
%             end
%         end
%      elseif mod(i_ind,2)~=0 
%          for j_ind=size(am_x,2):-1:1
%            if abs(am_z(i_ind,j_ind))>0
%                 x_path(m,1)=am_x(i_ind,j_ind);
%                 y_path(m,1)=am_y(i_ind,j_ind);
%                 m=m+1;
%            end
%         end
%      end
%  end
% 
% figure (7)
% plot(x_path,y_path,'LineWidth',1);
% title(['Slice z = ',num2str(D),' mm'])
% xlabel('x(mm)')
% ylabel('y(mm)')
% % xlim([-70,-20]);
% % ylim([-140,-80]);
% axis equal
% %==============================================================
% %优化后，之前的slice需要转换方向。
% 
% edgepoint_x=slice_x(k);
% edgepoint_y=slice_y(k);
% 
% 
% for i=1:num(1,1)-1
%    p_x_1=edgepoint_x(i,1);
%    p_y_1=edgepoint_y(i,1);
%    p_x_2=edgepoint_x(i+1,1);
%    p_y_2=edgepoint_y(i+1,1);
%    
%    %任意点到任意直线的距离 
%    dis=abs((p_x_1-edgepoint_x).*(p_y_2-edgepoint_y)-(p_x_2-edgepoint_x).*(p_y_1-edgepoint_y))/sqrt((p_x_1-p_x_2).^2+(p_y_1-p_y_2).^2);
%    %任意线段距离最短的点
%    dis_max(i)=max(dis);
%   end
% 
% 
% % 优化后的扫描方向绘图
%  [dis_t,I]=min(dis_max);
% 
%  
% oP_x_1=edgepoint_x(I,1);
% oP_y_1=edgepoint_y(I,1);
% oP_x_2=edgepoint_x(I+1,1);
% oP_y_2=edgepoint_y(I+1,1);
% 
% %最优方向画图
% figure (4)
% line([oP_x_1,oP_x_2],[oP_y_1,oP_y_2],'LineWidth',5)
% axis equal
% 
% %最优的扫描方向和水平方向的夹角
% dir_opt=[oP_x_1-oP_x_2,oP_y_1-oP_y_2]';
% dir_opt_unit=dir_opt/norm(dir_opt);
% dir_x=[1,0]';
% angle=pi-acos(dot(dir_opt,dir_x)/(norm(dir_opt)*norm(dir_x)));
% angle_deg=angle*180/pi;
% 
% %图像旋转
% % R_rot=[cos(angle),-sin(angle);sin(angle),cos(angle)];
% edgepoint_x_rot=edgepoint_x.*cos(angle)-edgepoint_y.*sin(angle);
% edgepoint_y_rot=edgepoint_x.*sin(angle)+edgepoint_y.*cos(angle);
% 
% 
% % 旋转前和旋转后的截面轮廓
% figure (5)
% plot(edgepoint_x,edgepoint_y,'LineWidth',1);
% hold on
% plot(edgepoint_x_rot,edgepoint_y_rot,'LineWidth',3);
% axis equal
% hold off
% 
% slice_x_rot=slice_x.*cos(angle)-slice_y.*sin(angle);
% slice_y_rot=slice_x.*sin(angle)+slice_y.*cos(angle);
% 
% [xq_rot,yq_rot] = meshgrid(min(slice_x_rot):x_space:max(slice_x_rot), min(slice_y_rot):y_space:max(slice_y_rot));
% [am_x,am_y,am_z]=griddata(slice_x_rot,slice_y_rot,D*ones(size(slice_x_rot)),xq_rot,yq_rot); %构造坐标点
% 
% m=1;
%  for i_ind=1:size(am_x,1)
%      if mod(i_ind,2)==0  
%         for j_ind=1:size(am_x,2)
%             if abs(am_z(i_ind,j_ind))>0
%                 x_path(m,1)=am_x(i_ind,j_ind);
%                 y_path(m,1)=am_y(i_ind,j_ind);
%                 m=m+1;
%             end
%         end
%      elseif mod(i_ind,2)~=0 
%          for j_ind=size(am_x,2):-1:1
%            if abs(am_z(i_ind,j_ind))>0
%                 x_path(m,1)=am_x(i_ind,j_ind);
%                 y_path(m,1)=am_y(i_ind,j_ind);
%                 m=m+1;
%            end
%         end
%      end
%  end
% 
% x_path_invrot=x_path.*cos(-angle)-y_path.*sin(-angle);
% y_path_invrot=x_path.*sin(-angle)+y_path.*cos(-angle); 
% 
% figure (6)
% plot(x_path_invrot,y_path_invrot,'LineWidth',1);
% title(['Slice z = ',num2str(D),' mm'])
% xlabel('x (mm)')
% ylabel('y (mm)')
% xlim([-70,-20]);
% ylim([-140,-80]);
% axis equal

