Ts=0.1; %define the sample time

A=[1 0 0 Ts 0 0; 0 1 0 0 Ts 0; 0 0 1 0 0 Ts; 0 0 0 1 0 0 ; 0 0 0 0 1 0; 0 0 0 0 0 1]; % the state matrix

C=[1 0 0 0 0 0 ; 0 1 0 0 0 0 ; 0 0 1 0 0 0]; % the output matrix

E=[0 0 0 1 0 0 ; 0 0 0 0 1 0 ; 0 0 0 0 0 1]; % the measurement mapping matrix for velocity parameters

B=[0.5*Ts^2 0 0;0 0.5*Ts^2 0;0 0 0.5*Ts^2;Ts 0 0;0 Ts 0; 0 0 Ts]; % the input matrix
B_new=[1 0 0;0 1 0;0 0 1;1 0 0;0 1 0; 0 0 1]; % the input matrix

x0=[0;0;0;0;0;0]; % the initial conditions
xnew=[10,10,10,10,10,10];
sys =ss(A,B,eye(6),[],Ts); % a system to generate true data

t=0:Ts:40; % the time interval

%assuming that the uncertanities in the accelerations are equal, we define

%them as follow:

segmaux=10; %standard deviation ax

segmauy=10; %standard deviation ay

segmaualpha=10; %standard deviation angular acceleration


%define the input(accelerations):

ux=[zeros(1,30) 2*ones(1,20) 0.5*ones(1,20) 0.5*ones(1,length(t)-70)]+normrnd(0,segmaux,1,length(t));

uy=[zeros(1,10) 5*ones(1,60) -0.5*ones(1,length(t)-70)]+normrnd(0,segmauy,1,length(t));

ualpha=[zeros(1,30) 0.5*ones(1,20) 0.05*ones(1,20) -0.05*ones(1,length(t)-70)]+normrnd(0,segmaualpha,1,length(t));

u=[ux;uy;ualpha];

%generating the true data:

Xtrue=lsim(sys,u,t,x0);

xtrue=Xtrue(:,1);

ytrue=Xtrue(:,2);

ztrue= ones(1,400)*5;

thtrue=Xtrue(:,3);

vxtrue=Xtrue(:,4);

vytrue=Xtrue(:,5);

wtrue=Xtrue(:,6);

%defining V:

measurmentsV=[1 0 0; 0 1 0; 0 0 1];
measurmentsVel=[0.1 0 0; 0 0.1 0; 0 0 0.1];
%generating measurment data by adding noise to the true data:

xm=xtrue+normrnd(0,30,length(xtrue),1);
xm2=xtrue+normrnd(0,10,length(xtrue),1);

ym=ytrue+normrnd(0,30,length(ytrue),1);
ym2=ytrue+normrnd(0,10,length(ytrue),1);

thm=thtrue+normrnd(0,30,length(ytrue),1);
thm2=thtrue+normrnd(0,10,length(ytrue),1);
      
vxm=vxtrue+normrnd(0,1,length(ytrue),1);
vym=vytrue+normrnd(0,1,length(ytrue),1);
vwm=wtrue+normrnd(0,1,length(ytrue),1);
%initializing the matricies for the for loop (this will make the matlab run

%the for loop faster.

Xest=zeros(6,length(t));
Xest(:,1)=x0;
Xcm=zeros(3,length(t));
Cov=zeros(6,6,400);
KG=zeros(6,3,400);
Xcm(:,1)=([xm(1); ym(1); thm(1)]*0.8)+([xm2(1); ym2(1); thm2(1)]*0.9)/1.7;
%defining R and Q

R=measurmentsV*C*C';
R_vel=measurmentsVel*C*C';
Q=[segmaux^2 0 0 ; 0 segmauy^2 0 ;0 0 segmaualpha^2];
%Q=[100 0 0 ; 0 100 0 ;0 0 100];
%Initializing P

P=B*Q*B';
%P=eye(6)*10;
disp(P);

for(i=2:1:length(t))

P=A*P*A'+B*Q*B'; %predicting P
%P=A*P*A'+B_new*Q*B_new';
Xest(:,i)=A*Xest(:,i-1)+B*u(:,i-1); %Predicitng the state
K=P*C'/(C*P*C'+R); %calculating the Kalman gains
Cov(:,:,i)=P;%covariance collection
KG(:,:,i)=K ;%Kalman gain collection

if(i<=100)
    Xest(:,i)=Xest(:,i)+K*([xm(i); ym(i); thm(i)]-C*Xest(:,i));%Correcting: estimating the state
    Xcm(:,i)=[xm(i); ym(i); thm(i)]; %combined measurement for plotting purpose
    P=(eye(6)-K*C)*P;%Correcting: estimating P
    
elseif(i>100 && i<=300)
    Xest(:,i)=Xest(:,i)+K*(([xm(i); ym(i); thm(i)])-C*Xest(:,i));
    Xcm(:,i)=[xm(i); ym(i); thm(i)];
    P=(eye(6)-K*C)*P;%Correcting: estimating P
    
else 
    K=P*E'/(E*P*E'+R_vel);
    Xest(:,i)= Xest(:,i)+ K*([vxm(i); vym(i); vwm(i)]-E*Xest(:,i));
    P=(eye(6)-K*E)*P;%Correcting: estimating P

end

 
end

%root mean square error calculation
est_x= Xest(1,:); 
est_y= Xest(2,:); 
est_th= Xest(3,:); 

r_x = sqrt( sum( (est_x(:)-xtrue(:)).^2) / length(t));
r_y = sqrt( sum( (est_y(:)-ytrue(:)).^2) / length(t));
r_th = sqrt( sum( (est_th(:)-thtrue(:)).^2) / length(t));
disp(r_x);
disp(r_y);
disp(r_th);
%subplot(311)

%plot(t,Xest(2,:),'r',t,vtrue,'b')

%xlabel('time [sec]');

%ylabel('velocity [m/s]');
%title('Velocity');

%legend('estimated velocity','true velocity')

t=0:1:400;

figure(1);
plot(t,Xest(1,:),'r',t,Xcm(1,:),'y',t,xtrue,'b');
xlabel('time[s]');
ylabel('displacementx[cm]');
title('displacementx ');
legend('estimated displacementx','measured displacementx','true displacementx');


%subplot(312)
figure(2);
plot(t,Xest(2,:),'r',t,Xcm(2,:),'y',t,ytrue,'b')
xlabel('time[s]');
ylabel('displacementy[cm] ');
title('displacementy');
legend('estimated displacementy','measured displacementy','true displacementy');



%subplot(313)
figure(3);
plot(t,Xest(3,:),'r',t,Xcm(3,:),'y',t,thtrue,'b')
xlabel('time[s] ');
ylabel('angle[degree]');
title('angle theta');
legend('estimated angle theta','measured angle theta','true angle theta');

%t=0:0.1:40;

%figure

%hold on


%simple animation:

for i=1:1:length(t)
figure(4);
axis([min(xtrue)-50 max(xtrue)+50 min(ytrue)-50 max(ytrue)+50]);

%viscircles([xtrue(i) ytrue(i)],20,'color','b')

%viscircles([Xest(1,i) Xest(2,i)],20,'color','r')

plot3(xtrue(i),ytrue(i),10,'-.r*');

plot3(Xest(1,i),Xest(2,i),10,'-.y*');
 
hold on
pause(0.001)

end

figure(5);
hold all
%plot(t(1:100),Xcm(1,(1:100)),'r');
plot(t(1:300),xm(1:300),'r');
plot(t(101:300),xm2(101:300),'g');
plot(t(101:300),Xcm(1,(101:300)),'b');
plot(t(1:300),xtrue((1:300),1),'c');
xlabel('time [sec]');
ylabel('displacementx [cm]');
title('displacementx');
legend('measurement from 2D','measurement from 3D','combined measurement from 2D and 3D','ground truth data');
hold off

figure(6);
hold all
%plot(t(1:100),Xcm(1,(1:100)),'r');
plot(t(1:300),ym(1:300),'r');
plot(t(101:300),ym2(101:300),'g');
plot(t(101:300),Xcm(2,(101:300)),'b');
plot(t(1:300),ytrue((1:300),1),'c');
xlabel('time [sec]');
ylabel('displacementy [cm]');
title('displacementy');
legend('measurement from 2D','measurement from 3D','combined measurement from 2D and 3D','ground truth data');
hold off

figure(7);
hold all
%plot(t(1:100),Xcm(1,(1:100)),'r');
plot(t(1:300),thm(1:300),'r');
plot(t(101:300),thm2(101:300),'g');
plot(t(101:300),Xcm(3,(101:300)),'b');
plot(t(1:300),thtrue((1:300),1),'c');
xlabel('time [sec]');
ylabel('angle [degree]');
title('angle');
legend('measurement from 2D','measurement from 3D','combined measurement from 2D and 3D','ground truth data');
hold off

figure(8);
hold all
%plot(t(1:100),Xcm(1,(1:100)),'r');
plot(t(300:400),vxm(300:400),'r');
plot(t(1:400),vxtrue((1:400),1),'c');
xlabel('time [sec]');
ylabel('deltax [cm]');
title('deltax');
legend('measurement from 3D','ground truth data');
hold off

figure(9);
hold all
%plot(t(1:100),Xcm(1,(1:100)),'r');
plot(t(300:400),vym(300:400),'r');
plot(t(1:400),vytrue((1:400),1),'c');
xlabel('time [sec]');
ylabel('deltay [cm]');
title('deltay');
legend('measurement from 3D','ground truth data');
hold off

figure(10);
hold all
%plot(t(1:100),Xcm(1,(1:100)),'r');
plot(t(300:400),vwm(300:400),'r');
plot(t(1:400),wtrue((1:400),1),'c');
xlabel('time [sec]');
ylabel('deltatheta [degrees]');
title('deltatheta');
legend('measurement from 3D','ground truth data');
hold off

cov_x(:) = Cov(1,1,:);
cov_y(:)= Cov(2,2,:);
cov_th(:)= Cov(3,3,:);
figure(11);
%plot(t(1:100),Xcm(1,(1:100)),'r');
hold all;
plot(t,cov_x,'r');
xlabel('time [sec]');
ylabel('variance_x ');
title('variance in x');
hold off

figure(12);
%plot(t(1:100),Xcm(1,(1:100)),'r');
hold all;
plot(t,cov_y,'r');
xlabel('time [sec]');
ylabel('variance_y');
title('variance in y');
hold off

figure(13);
%plot(t(1:100),Xcm(1,(1:100)),'r');
hold all;
plot(t,cov_th,'r');
xlabel('time [sec]');
ylabel('variance_th ');
title('variance in theta');
hold off

KG_x(:) = KG(1,1,:);
KG_y(:)= KG(2,2,:);
KG_th(:)= KG(3,3,:);

figure(14);
hold all;
plot(t,KG_x,'r');
xlabel('time [sec]');
ylabel('KalmanGain_x ');
title('KalmanGain_x');
hold off

figure(15);
hold all;
plot(t,KG_y,'r');
xlabel('time [sec]');
ylabel('KalmanGain_y ');
title('KalmanGain_y');
hold off

figure(16);
hold all;
plot(t,KG_th,'r');
xlabel('time [sec]');
ylabel('KalmanGain_th ');
title('KalmanGain_th');
hold off

saveas(figure(1),[pwd '/Results/figure1.png']);
saveas(figure(2),[pwd '/Results/figure2.png']);
saveas(figure(3),[pwd '/Results/figure3.png']);
saveas(figure(4),[pwd '/Results/figure4.png']);
saveas(figure(5),[pwd '/Results/figure5.png']);
saveas(figure(6),[pwd '/Results/figure6.png']);
saveas(figure(7),[pwd '/Results/figure7.png']);
saveas(figure(8),[pwd '/Results/figure8.png']);
saveas(figure(9),[pwd '/Results/figure9.png']);
saveas(figure(10),[pwd '/Results/figure10.png']);
saveas(figure(11),[pwd '/Results/figure11.png']);
saveas(figure(12),[pwd '/Results/figure12.png']);
saveas(figure(13),[pwd '/Results/figure13.png']);
saveas(figure(14),[pwd '/Results/figure14.png']);
saveas(figure(15),[pwd '/Results/figure15.png']);
saveas(figure(16),[pwd '/Results/figure16.png']);