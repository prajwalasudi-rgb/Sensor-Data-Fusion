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
measurmentsVel=[1 0 0; 0 1 0; 0 0 1];
%generating measurment data by adding noise to the true data:

k=0;
fileID = fopen('RMS_v.txt','w');
for(l=1:5:25)
j=0.1*l;
xm=xtrue+normrnd(0,10,length(xtrue),1);
xm2=xtrue+normrnd(0,10,length(xtrue),1);

ym=ytrue+normrnd(0,10,length(ytrue),1);
ym2=ytrue+normrnd(0,10,length(ytrue),1);

thm=thtrue+normrnd(0,1,length(ytrue),1);
thm2=thtrue+normrnd(0,1,length(ytrue),1);
      

vxm=vxtrue+normrnd(0,j,length(ytrue),1);
vym=vytrue+normrnd(0,j,length(ytrue),1);
vwm=wtrue+normrnd(0,j,length(ytrue),1);
%initializing the matricies for the for loop (this will make the matlab run

%the for loop faster.

Xest=zeros(6,length(t));
Xest(:,1)=xnew;
Xcm=zeros(3,length(t));
Cov=zeros(6,6,400);
KG=zeros(6,3,400);
Xcm(:,1)=([xm(1); ym(1); thm(1)]*0.8)+([xm2(1); ym2(1); thm2(1)]*0.9)/1.7;
%defining R and Q

R=measurmentsV*C*C';
R_vel=measurmentsVel*C*C';
%Q=[segmaux^2 0 0 ; 0 segmauy^2 0 ;0 0 segmaualpha^2];
Q=[100 0 0; 0 100 0 ;0 0 100];
%Initializing P

%P=B*R*B';
P=eye(6)*100;
%disp(P);

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
    Xest(:,i)=Xest(:,i)+K*(((([xm(i); ym(i); thm(i)]*0.8)+([xm2(i); ym2(i); thm2(i)]*0.9))/1.7)-C*Xest(:,i));
    Xcm(:,i)=(([xm(i); ym(i); thm(i)]*0.8)+([xm2(i); ym2(i); thm2(i)*0.9]))/1.7;
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

fprintf(fileID,'%12f %12.8f %12.8f %12.8f\n',l,r_x,r_y,r_th);

%subplot(311)

%plot(t,Xest(2,:),'r',t,vtrue,'b')

%xlabel('time [sec]');

%ylabel('velocity [m/s]');
%title('Velocity');

%legend('estimated velocity','true velocity')

t=0:1:400;



for i=1:1:length(t)
diff_x(i)=abs(xtrue(i)-Xest(1,i));
diff_y(i)=abs(ytrue(i)-Xest(2,i));
diff_th(i)=abs(thtrue(i)-Xest(3,i));
diff_xm(i)=abs(xtrue(i)-Xcm(1,i));
diff_ym(i)=abs(ytrue(i)-Xcm(2,i));
diff_thm(i)=abs(thtrue(i)-Xcm(3,i));

end
k=k+1;
figure(k);
hold all
bar(t(301:400),diff_x(301:400));
data_mean=mean(diff_x(301:400));
hline = refline([0 data_mean]);
hline.Color = 'r';
xlabel('time[sec]');
ylabel('abs(Xtrue-Xest)[cm]');
str = strcat('Error in prediction with measurement velocity data of SD =',num2str(j));
title(str);

figure(k+5);
hold all
bar(t(2:300),diff_x(2:300));
data_mean=mean(diff_x(2:300));
hline = refline([0 data_mean]);
hline.Color = 'r';
xlabel('time[sec]');
ylabel('abs(Xtrue-Xest)[cm]');
str = strcat('Error in prediction with measurement velocity data of SD =',num2str(j));
title(str);

figure(k+10);
hold all
bar(t(301:400),diff_y(301:400));
data_mean=mean(diff_y(301:400));
hline = refline([0 data_mean]);
hline.Color = 'r';
xlabel('time[sec]');
ylabel('abs(Ytrue-Yest)[cm]');
str = strcat('Error in prediction with measurement velocity data of SD =',num2str(j));
title(str);

figure(k+15);
hold all
bar(t(2:300),diff_y(2:300));
data_mean=mean(diff_y(2:300));
hline = refline([0 data_mean]);
hline.Color = 'r';
xlabel('time[sec]');
ylabel('abs(Ytrue-Yest)[cm]');
str = strcat('Error in prediction with measurement  velocity data of SD =',num2str(j));
title(str);

figure(k+20);
hold all
bar(t(301:400),diff_th(301:400));
data_mean=mean(diff_th(301:400));
hline = refline([0 data_mean]);
hline.Color = 'r';
xlabel('time[sec]');
ylabel('abs(thtrue-thest)[degree]');
str = strcat('Error in prediction with measurement velocity data of SD =',num2str(0.1*j));
title(str);

figure(k+25);
hold all
bar(t(2:300),diff_th(2:300));
data_mean=mean(diff_th(2:300));
hline = refline([0 data_mean]);
hline.Color = 'r';
xlabel('time[sec]');
ylabel('abs(thtrue-thest)[degree]');
str = strcat('Error in prediction with measurement velocity data of SD =',num2str(0.1*j));
title(str);

end
fclose(fileID);


saveas(figure(1),[pwd '/Results/Xerror_v/figure1.png']);
saveas(figure(2),[pwd '/Results/Xerror_v/figure2.png']);
saveas(figure(3),[pwd '/Results/Xerror_v/figure3.png']);
saveas(figure(4),[pwd '/Results/Xerror_v/figure4.png']);
saveas(figure(5),[pwd '/Results/Xerror_v/figure5.png']);
saveas(figure(6),[pwd '/Results/Xerror_v/figure6.png']);
saveas(figure(7),[pwd '/Results/Xerror_v/figure7.png']);
saveas(figure(8),[pwd '/Results/Xerror_v/figure8.png']);
saveas(figure(9),[pwd '/Results/Xerror_v/figure9.png']);
saveas(figure(10),[pwd '/Results/Xerror_v/figure10.png']);


saveas(figure(11),[pwd '/Results/Yerror_v/figure1.png']);
saveas(figure(12),[pwd '/Results/Yerror_v/figure2.png']);
saveas(figure(13),[pwd '/Results/Yerror_v/figure3.png']);
saveas(figure(14),[pwd '/Results/Yerror_v/figure4.png']);
saveas(figure(15),[pwd '/Results/Yerror_v/figure5.png']);
saveas(figure(16),[pwd '/Results/Yerror_v/figure6.png']);
saveas(figure(17),[pwd '/Results/Yerror_v/figure7.png']);
saveas(figure(18),[pwd '/Results/Yerror_v/figure8.png']);
saveas(figure(19),[pwd '/Results/Yerror_v/figure9.png']);
saveas(figure(20),[pwd '/Results/Yerror_v/figure10.png']);


saveas(figure(21),[pwd '/Results/Therror_v/figure1.png']);
saveas(figure(22),[pwd '/Results/Therror_v/figure2.png']);
saveas(figure(23),[pwd '/Results/Therror_v/figure3.png']);
saveas(figure(24),[pwd '/Results/Therror_v/figure4.png']);
saveas(figure(25),[pwd '/Results/Therror_v/figure5.png']);
saveas(figure(26),[pwd '/Results/Therror_v/figure6.png']);
saveas(figure(27),[pwd '/Results/Therror_v/figure7.png']);
saveas(figure(28),[pwd '/Results/Therror_v/figure8.png']);
saveas(figure(29),[pwd '/Results/Therror_v/figure9.png']);
saveas(figure(30),[pwd '/Results/Therror_v/figure10.png']);

close all;