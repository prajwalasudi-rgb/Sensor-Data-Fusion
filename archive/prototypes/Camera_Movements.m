surf(peaks)
axis vis3d off
for x = -200:5:200
    campos([x,5,10])
    set(gca,'Projection','perspective')
    camva(35)
    drawnow
end

%This program is to simulate different discontinuity and its struct light
%image
clear;clc;close all;
X=[0:0.4:200];
Y=[0:0.4:100];
[x,y]=meshgrid(X,Y);
M=length(X);
N=length(Y);
z=zeros(N,M);
%z(find(X<60))=0;
Mid1=fix(M/4);
Mid2=fix(M/2);
for k=Mid1:Mid2
    z(:,k)=z(:,k)+0.1*(k-Mid1);
end
for k=Mid2+1:M
    z(:,k)=z(:,k)+0.1*(Mid2-Mid1);
end

figure(1);
mesh(x,y,z)
f0=0.02;
I1=cos(2*pi*f0*x);
I2=cos(2*pi*f0*x+z);

figure(2);
subplot(2,1,1);
imshow(I1);
subplot(2,1,2);
imshow(I2);
figure(3);
colormap(Gray);
mesh(x,y,z,I2);
view([0 90]);
set(gca, 'CameraPosition', [100 5000 2000]);
