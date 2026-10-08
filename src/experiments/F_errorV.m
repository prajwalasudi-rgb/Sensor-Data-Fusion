Ts=0.1; %define the sample time

A=[1 0 0 Ts 0 0; 0 1 0 0 Ts 0; 0 0 1 0 0 Ts; 0 0 0 1 0 0 ; 0 0 0 0 1 0; 0 0 0 0 0 1]; % the state matrix

C=[1 0 0 0 0 0 ; 0 1 0 0 0 0 ; 0 0 1 0 0 0]; % the output matrix

E=[0 0 0 1 0 0 ; 0 0 0 0 1 0 ; 0 0 0 0 0 1]; % the measurement mapping matrix for velocity parameters

B=[0.5*Ts^2 0 0;0 0.5*Ts^2 0;0 0 0.5*Ts^2;Ts 0 0;0 Ts 0; 0 0 Ts]; % the input matrix
B_new=[1 0 0;0 1 0;0 0 1;1 0 0;0 1 0; 0 0 1]; % the input matrix
F=[1 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0 ; 0 0 1 0 0 0 0 0 0;0 0 0 1 0 0 0 0 0;0 0 0 0 1 0 0 0 0;0 0 0 0 0 1 0 0 0]; % the output matrix
L=[1 0 0;0 1 0;0 0 1;1 0 0;0 1 0;0 0 1];
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

Xtrue=lsim(sys,u',t,x0);

xtrue=Xtrue(:,1);

ytrue=Xtrue(:,2);

ztrue= ones(1,400)*5;

thtrue=Xtrue(:,3);

vxtrue=Xtrue(:,4);

vytrue=Xtrue(:,5);

wtrue=Xtrue(:,6);

%defining V:

measurmentsV=[10 0 0; 0 10 0; 0 0 10];
measurments=[100 0 0 0 0 0; 0 100 0 0 0 0;  0 0 100 0 0 0;0 0 0 100 0 0 ;0 0 0 0 100 0 ;0 0 0 0 0 100];
measurmentsVel=[0.1 0 0; 0 0.1 0; 0 0 0.1];
%generating measurment data by adding noise to the true data:
k=0;
fileID = fopen(fullfile(results_dir('rms'),'RMS_velocity_noise_sweep.txt'),'w');
fprintf(fileID,'%s %s %s %s %s %s %s\n','SD','r_x1','r_y1','r_th1','r_x2','r_y2','r_th2');
for(j=1:5:25)
disp(j);
g=j*0.05;
xm=xtrue+normrnd(0,10,length(xtrue),1);
xm2=xtrue+normrnd(0,10,length(xtrue),1);

ym=ytrue+normrnd(0,10,length(ytrue),1);
ym2=ytrue+normrnd(0,10,length(ytrue),1);

thm=thtrue+normrnd(0,1,length(ytrue),1);
thm2=thtrue+normrnd(0,1,length(ytrue),1);
      
vxm=vxtrue+normrnd(0,g,length(ytrue),1);
vym=vytrue+normrnd(0,g,length(ytrue),1);
vwm=wtrue+normrnd(0,g,length(ytrue),1);
%initializing the matricies for the for loop (this will make the matlab run

%the for loop faster.

Xest=zeros(9,length(t));
A_new=[1 0 0 Ts 0 0 (0.5*Ts*Ts) 0 0; 0 1 0 0 Ts 0 0 (0.5*Ts*Ts) 0; 0 0 1 0 0 Ts 0 0 (0.5*Ts*Ts); 0 0 0 1 0 0 Ts 0 0 ; 0 0 0 0 1 0 0 Ts 0; 0 0 0 0 0 1 0 0 Ts;0 0 0 0 0 0 1 0 0;0 0 0 0 0 0 0 1 0;0 0 0 0 0 0 0 0 1]; % the state matrix
C_new=[1 0 0 0 0 0 0 0 0 ; 0 1 0 0 0 0 0 0 0 ; 0 0 1 0 0 0 0 0 0];
E_new=[0 0 0 1 0 0 0 0 0 ; 0 0 0 0 1 0 0 0 0 ; 0 0 0 0 0 1 0 0 0];
Xcm=zeros(3,length(t));
Cov=zeros(9,9,400);
KG=zeros(9,3,400);
Xcm(:,1)=([xm(1); ym(1); thm(1)]*0.8)+([xm2(1); ym2(1); thm2(1)]*0.9)/1.7;
%defining R and Q

R=measurmentsV*C*C';
R1=measurments*F*F';
R_vel=measurmentsVel*C*C';
%Q=[segmaux^2 0 0 ; 0 segmauy^2 0 ;0 0 segmaualpha^2];
%Q=[1 0 0 0 0 0 0 0 0  ; 0 1 0 0 0 0 0 0 0 ;0 0 1 0 0 0 0 0 0;0 0 0 1 0 0 0 0 0  ; 0 0 0 0 1 0 0 0 0 ;0 0 0 0 0 1 0 0 0;0 0 0 0 0 0 1 0 0;0 0 0 0 0 0 0 1 0;0 0 0 0 0 0 0 0 1];
%Initializing P
Q=eye(9)*100;

%P=B*R*B';
P=eye(9)*100;
%disp(P);

for(i=2:1:length(t))


if(i<=7)
    Q=eye(9)*100;
    P=A_new*P*A_new'+Q; %predicting P
    Xest(:,i)=A_new*Xest(:,i-1); %Predicitng the sta
    K=P*C_new'/(C_new*P*C_new'+R);
    Cov(:,:,i)=P;%covariance collection
    KG(:,:,i)=K ;%Kalman gain collection
    Xest(:,i)=Xest(:,i)+K*([xm(i); ym(i); thm(i)]-C_new*Xest(:,i));%Correcting: estimating the state
    Xcm(:,i)=[xm(i); ym(i); thm(i)]; %combined measurement for plotting purpose
    P=(eye(9)-K*C_new)*P;%Correcting: estimating P
    
elseif(i>7 && i<=300)
    Q=eye(9)*100;
    P=A_new*P*A_new'+Q; %predicting P
    Xest(:,i)=A_new*Xest(:,i-1); %Predicitng the sta
    K1=P*F'/(F*P*F'+R1);
    Cov(:,:,i)=P;%covariance collection
    KG(:,:,i)=K1*L ;%Kalman gain collection
    Xest(:,i)=Xest(:,i)+K1*(((([xm(i); ym(i); thm(i);vxm(i)/0.8;vym(i)/0.8;vwm(i)/0.8]*0.8)+([xm2(i); ym2(i); thm2(i);0;0;0]*0.9))/1.7)-F*Xest(:,i));
    Xcm(:,i)=(([xm(i); ym(i); thm(i)]*0.8)+([xm2(i); ym2(i); thm2(i)*0.9]))/1.7;
    P=(eye(9)-K1*F)*P;%Correcting: estimating P
    
else
    Q=eye(9)*10;
    P=A_new*P*A_new'+Q; %predicting P
    Xest(:,i)=A_new*Xest(:,i-1); %Predicitng the sta
    K=P*E_new'/(E_new*P*E_new'+R_vel);
    Cov(:,:,i)=P;%covariance collection
    KG(:,:,i)=K ;%Kalman gain collection
    Xest(:,i)= Xest(:,i)+ K*([vxm(i); vym(i); vwm(i)]-E_new*Xest(:,i));
    P=(eye(9)-K*E_new)*P;%Correcting: estimating P

end

 
end


%root mean square error calculation
est_x= Xest(1,:); 
est_y= Xest(2,:); 
est_th= Xest(3,:); 
r_x = sqrt( sum( (est_x(1,1:300)'-xtrue(1:300,1)).^2) / 300);
r_y = sqrt( sum( (est_y(1,1:300)'-ytrue(1:300,1)).^2) / 300);
r_th = sqrt( sum( (est_th(1,1:300)'-thtrue(1:300,1)).^2) / 300);
r_x2 = sqrt( sum( (est_x(1,300:400)'-xtrue(300:400,1)).^2) / 100);
r_y2 = sqrt( sum( (est_y(1,300:400)'-ytrue(300:400,1)).^2) / 100);
r_th2 = sqrt( sum( (est_th(1,300:400)'-thtrue(300:400,1)).^2) / 100);
fprintf(fileID,'%12f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n',j,r_x,r_y,r_th,r_x2,r_y2,r_th2);
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
%bar(t(301:400),diff_x(301:400));
%data_mean=mean(diff_x(301:400));
%hline = refline([0 data_mean]);
%hline.Color = 'r';
%p = 0:0.25:1;
%y = quantile(diff_x(301:400),p);
%z = [p;y];

%h=histfit(diff_x(301:400));
%set(h(1),'facecolor','g'); set(h(2),'color','m');
%xlabel('time[sec]');
boxplot(diff_x(301:400));
y = [mean(diff_x(301:400)),median(diff_x(301:400))];
ylabel('Histogram of estimation error in x stage2 [cm]');
str = strcat('Sigma of sensor(stage2)=' ,num2str(g),' mean =',num2str(y(1,1)),' median =',num2str(y(1,2)));
title(str);

figure(k+5);
hold all
%bar(t(2:300),diff_x(2:300));
%data_mean=mean(diff_x(2:300));
%hline = refline([0 data_mean]);
%hline.Color = 'r';
%{
histfit(diff_x(2:300));
xlabel('time[sec]');
ylabel('abs(Xtrue-Xest)[cm]');
str = strcat('Error in prediction with measurement data of SD =',num2str(j));
title(str);
%}
boxplot(diff_x(2:300));
y = [mean(diff_x(2:300)),median(diff_x(2:300))];
ylabel('Histogram of estimation error in x stage1[cm] ');
str = strcat('Sigma of sensor(stage2)=' ,num2str(g),' mean =',num2str(y(1,1)),' median =',num2str(y(1,2)));
title(str);

figure(k+10);
hold all
%bar(t(301:400),diff_y(301:400));
%data_mean=mean(diff_y(301:400));
%hline = refline([0 data_mean]);
%hline.Color = 'r';
%{
h=histfit(diff_y(301:400));
set(h(1),'facecolor','g'); set(h(2),'color','m');
xlabel('time[sec]');
ylabel('abs(Ytrue-Yest)[cm]');
str = strcat('Error in prediction with measurement data of SD =',num2str(j));
title(str);
%}
boxplot(diff_y(301:400));
y = [mean(diff_y(301:400)),median(diff_y(301:400))];
ylabel('Histogram of estimation error in y stage2[cm] ');
str = strcat('Sigma of sensor(stage2)=' ,num2str(g),' mean =',num2str(y(1,1)),' median =',num2str(y(1,2)));
title(str);

figure(k+15);
hold all
%bar(t(2:300),diff_y(2:300));
%data_mean=mean(diff_y(2:300));
%hline = refline([0 data_mean]);
%hline.Color = 'r';
%{
histfit(diff_y(2:300));
xlabel('time[sec]');
ylabel('abs(Ytrue-Yest)[cm]');
str = strcat('Error in prediction with measurement data of SD =',num2str(j));
title(str);
%}
boxplot(diff_y(2:300));
y = [mean(diff_y(2:300)),median(diff_y(2:300))];
ylabel('Histogram of estimation error in y stage1[cm]');
str = strcat('Sigma of sensor(stage2)=' ,num2str(g),' mean =',num2str(y(1,1)),' median =',num2str(y(1,2)));
title(str);

figure(k+20);
hold all
%bar(t(301:400),diff_th(301:400));
%data_mean=mean(diff_th(301:400));
%hline = refline([0 data_mean]);
%hline.Color = 'r';
%{
h=histfit(diff_th(301:400));
set(h(1),'facecolor','g'); set(h(2),'color','m');
xlabel('time[sec]');
ylabel('abs(thtrue-thest)[degree]');
str = strcat('Error in prediction with measurement data of SD =',num2str(0.1*j));
title(str);
%}
boxplot(diff_th(301:400));
y = [mean(diff_th(301:400)),median(diff_th(301:400))];
ylabel('Histogram of estimation error in angle stage2 [degree] ');
str = strcat('Sigma of sensor(stage2)=' ,num2str(g),' mean =',num2str(y(1,1)),' median =',num2str(y(1,2)));
title(str);

figure(k+25);
hold all
%bar(t(2:300),diff_th(2:300));
%data_mean=mean(diff_th(2:300));
%hline = refline([0 data_mean]);
%hline.Color = 'r';
%{
histfit(diff_th(2:300));
xlabel('time[sec]');
ylabel('abs(thtrue-thest)[degree]');
str = strcat('Error in prediction with measurement data of SD =',num2str(0.1*j));
title(str);
%}
boxplot(diff_th(2:300));
y = [mean(diff_th(2:300)),median(diff_th(2:300))];
ylabel('Histogram of estimation error in angle stage1 [degree]');
str = strcat('Sigma of sensor(stage2)=' ,num2str(j),' mean =',num2str(y(1,1)),' median =',num2str(y(1,2)));
title(str);

end


fclose(fileID);

saveas(figure(1),[results_dir('Xerror_v') '/figure1.png']);
saveas(figure(2),[results_dir('Xerror_v') '/figure2.png']);
saveas(figure(3),[results_dir('Xerror_v') '/figure3.png']);
saveas(figure(4),[results_dir('Xerror_v') '/figure4.png']);
saveas(figure(5),[results_dir('Xerror_v') '/figure5.png']);
saveas(figure(6),[results_dir('Xerror_v') '/figure6.png']);
saveas(figure(7),[results_dir('Xerror_v') '/figure7.png']);
saveas(figure(8),[results_dir('Xerror_v') '/figure8.png']);
saveas(figure(9),[results_dir('Xerror_v') '/figure9.png']);
saveas(figure(10),[results_dir('Xerror_v') '/figure10.png']);


saveas(figure(11),[results_dir('Yerror_v') '/figure1.png']);
saveas(figure(12),[results_dir('Yerror_v') '/figure2.png']);
saveas(figure(13),[results_dir('Yerror_v') '/figure3.png']);
saveas(figure(14),[results_dir('Yerror_v') '/figure4.png']);
saveas(figure(15),[results_dir('Yerror_v') '/figure5.png']);
saveas(figure(16),[results_dir('Yerror_v') '/figure6.png']);
saveas(figure(17),[results_dir('Yerror_v') '/figure7.png']);
saveas(figure(18),[results_dir('Yerror_v') '/figure8.png']);
saveas(figure(19),[results_dir('Yerror_v') '/figure9.png']);
saveas(figure(20),[results_dir('Yerror_v') '/figure10.png']);


saveas(figure(21),[results_dir('Therror_v') '/figure1.png']);
saveas(figure(22),[results_dir('Therror_v') '/figure2.png']);
saveas(figure(23),[results_dir('Therror_v') '/figure3.png']);
saveas(figure(24),[results_dir('Therror_v') '/figure4.png']);
saveas(figure(25),[results_dir('Therror_v') '/figure5.png']);
saveas(figure(26),[results_dir('Therror_v') '/figure6.png']);
saveas(figure(27),[results_dir('Therror_v') '/figure7.png']);
saveas(figure(28),[results_dir('Therror_v') '/figure8.png']);
saveas(figure(29),[results_dir('Therror_v') '/figure9.png']);
saveas(figure(30),[results_dir('Therror_v') '/figure10.png']);

close all;