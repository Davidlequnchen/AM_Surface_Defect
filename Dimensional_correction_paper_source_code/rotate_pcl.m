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

A(:,1) = x;                            %各坐标数据
A(:,2) = y;

% z=z+250;
% A(:,3)=z-2.69;

ptCloud_A = pointCloud([x(:),y(:),z(:)]);

figure (1)
pcshow(ptCloud_A);

xlabel('X (mm)');% x轴名称
ylabel('Y (mm)');
zlabel('Z (mm)');
colormap(jet)
view(0,90)


%--------------------------------------------------------------------------
%点云处理，绕Z轴旋转
% gama=-atan(pointdata_nor(1,1)/pointdata_nor(1,2));
% gama_alpha=gama*180/pi;
% gama=2.6*pi/180; 
gama=(2.6)*pi/180;         


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
alpha=-1.5*pi/180;
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
%view(0,90)

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
beta=-2.0*pi/180;

R_y=[  cos(beta),   0,   -sin(beta);
               0,   1,           0;
      sin(beta),   0,   cos(beta);];
  
for point_num=1:IX
    D(point_num,:)=(R_y*C(point_num,:)')';
end

x=D(:,1);                                %各坐标数据
y=D(:,2);
z=D(:,3);

z=z-min(z);
x_mean=(max(x)+min(x))/2;
y_mean=(max(y)+min(y))/2;
x=x-x_mean;
y=y-y_mean;

D(:,1) = x;                            %各坐标数据
D(:,2) = y;
D(:,3) = z;

ptCloud_D = pointCloud(D);

figure (4)
pcshow(ptCloud_D);

xlabel('X (mm)');% x轴名称
ylabel('Y (mm)');
zlabel('Z (mm)');
colormap(jet)
%view(0,90)

%--------------------------------------------------------------------------
save('ptCloud.mat','ptCloud_D')
pcwrite(ptCloud_D,'30_seg_rot.pcd')