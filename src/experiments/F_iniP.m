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

segmaux=35; %standard deviation ax

segmauy=35; %standard deviation ay

segmaualpha=3.5; %standard deviation angular acceleration


%define the input(accelerations):

ux=[zeros(1,30) 2*ones(1,20) 0.5*ones(1,20) 0.5*ones(1,length(t)-70)]+normrnd(0,segmaux,1,length(t));

uy=[zeros(1,10) 5*ones(1,60) -0.5*ones(1,length(t)-70)]+normrnd(0,segmauy,1,length(t));

ualpha=[zeros(1,30) 0.001*ones(1,20) 0.002*ones(1,20) -0.0003*ones(1,length(t)-70)]+normrnd(0,segmaualpha,1,length(t));

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

measurmentsV=[100 0 0; 0 100 0; 0 0 100];
measurmentsVel=[100 0 0; 0 100 0; 0 0 100];
%generating measurment data by adding noise to the true data:

xm=xtrue+normrnd(0,30,length(xtrue),1);
xm2=xtrue+normrnd(0,10,length(xtrue),1);

ym=ytrue+normrnd(0,30,length(ytrue),1);
ym2=ytrue+normrnd(0,10,length(ytrue),1);

thm=thtrue+normrnd(0,30,length(ytrue),1);
thm2=thtrue+normrnd(0,10,length(ytrue),1);
      
vxm=vxtrue+normrnd(0,10,length(ytrue),1);
vym=vytrue+normrnd(0,10,length(ytrue),1);
vwm=wtrue+normrnd(0,10,length(ytrue),1);
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
R_vel=measurmentsVel*C*C';
%Q=[segmaux^2 0 0 ; 0 segmauy^2 0 ;0 0 segmaualpha^2];
%Q=[1 0 0 0 0 0 0 0 0  ; 0 1 0 0 0 0 0 0 0 ;0 0 1 0 0 0 0 0 0;0 0 0 1 0 0 0 0 0  ; 0 0 0 0 1 0 0 0 0 ;0 0 0 0 0 1 0 0 0;0 0 0 0 0 0 1 0 0;0 0 0 0 0 0 0 1 0;0 0 0 0 0 0 0 0 1];
%Initializing P
Q=eye(9)*100;
for(j=1:100:500)
   P=eye(9)*j;
   Q=eye(9)*j;
%P=B*R*B';

%disp(P);

for(i=2:1:length(t))

P=A_new*P*A_new'+Q; %predicting P
%P=A*P*A';
Xest(:,i)=A_new*Xest(:,i-1); %Predicitng the state
%Xest(:,i)=A*Xest(:,i-1);
K=P*C_new'/(C_new*P*C_new'+R); %calculating the Kalman gains
Cov(:,:,i)=P;%covariance collection
KG(:,:,i)=K ;%Kalman gain collection

if(i<=100)
    Xest(:,i)=Xest(:,i)+K*([xm(i); ym(i); thm(i)]-C_new*Xest(:,i));%Correcting: estimating the state
    Xcm(:,i)=[xm(i); ym(i); thm(i)]; %combined measurement for plotting purpose
    P=(eye(9)-K*C_new)*P;%Correcting: estimating P
    
elseif(i>100 && i<=300)
    Xest(:,i)=Xest(:,i)+K*(((([xm(i); ym(i); thm(i)]*0.8)+([xm2(i); ym2(i); thm2(i)]*0.9))/1.7)-C_new*Xest(:,i));
    Xcm(:,i)=(([xm(i); ym(i); thm(i)]*0.8)+([xm2(i); ym2(i); thm2(i)*0.9]))/1.7;
    P=(eye(9)-K*C_new)*P;%Correcting: estimating P
    
else 
    K=P*E_new'/(E_new*P*E_new'+R_vel);
    Xest(:,i)= Xest(:,i)+ K*([vxm(i); vym(i); vwm(i)]-E_new*Xest(:,i));
    P=(eye(9)-K*E_new)*P;%Correcting: estimating P

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
cov_x = squeeze(Cov(1,1,:))';
cov_y = squeeze(Cov(2,2,:))';
cov_th = squeeze(Cov(3,3,:))';
figure(1);
hold all;
plot(t,cov_x,'r');
xlabel('time [sec]');
ylabel('error variance-x [cm^{2}] with different Q values ');
title('error variance in x with R=25');
str=num2str(j);
t_x=200;
text(t_x,cov_x(1,200),str);

figure(2);
%plot(t(1:100),Xcm(1,(1:100)),'r');
hold all;
plot(t,cov_y,'r');
xlabel('time [sec]');
ylabel('variance-y [cm^{2}]with different Q values');
title('error variance in y with R=25');
str=num2str(j);
t_x=200;
text(t_x,cov_y(1,200),str);

figure(3);
%plot(t(1:100),Xcm(1,(1:100)),'r');
hold all;
plot(t,cov_th,'r');
xlabel('time [sec]');
ylabel('variance-th [cm^{2}] with different Q values');
title('error variance in theta with R=25');
str=num2str(j);
t_x=200;
text(t_x,cov_th(1,200),str);

KG_x = squeeze(KG(1,1,:))';
KG_y = squeeze(KG(2,2,:))';
KG_th = squeeze(KG(3,3,:))';

figure(4);

hold all;
plot(t,KG_x,'r');
xlabel('time [sec]');
ylabel('KalmanGain-x with different Q values');
title('KalmanGain-x with R=25');
str=num2str(j);
t_x=200;
text(t_x,KG_x(1,200),str);

figure(5);

hold all;
plot(t,KG_y,'r');
xlabel('time [sec]');
ylabel('KalmanGain-y with different Q values');
title('KalmanGain-y with R=25');
str=num2str(j);
t_x=200;
text(t_x,KG_y(1,200),str);

figure(6);

hold all;
plot(t,KG_th,'r');
xlabel('time [sec]');
ylabel('KalmanGain-th with different Q values ');
title('KalmanGain-th with R=25');
str=num2str(j);
t_x=200;
text(t_x,KG_th(1,200),str);


end
%saveas(figure(1),[results_dir('Ini_P') '/figure13.png']);
%saveas(figure(2),[results_dir('Ini_P') '/figure14.png']);
%saveas(figure(3),[results_dir('Ini_P') '/figure15.png']);
%saveas(figure(4),[results_dir('Ini_P') '/figure16.png']);
%saveas(figure(5),[results_dir('Ini_P') '/figure17.png']);
%saveas(figure(6),[results_dir('Ini_P') '/figure18.png']);

%close all;