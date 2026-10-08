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

uy=[zeros(1,30) 2*ones(1,20) 0.5*ones(1,20) 0.5*ones(1,length(t)-70)]+normrnd(0,segmaux,1,length(t));

%uy=[zeros(1,10) 3*ones(1,60) -0.5*ones(1,length(t)-70)]+normrnd(0,segmauy,1,length(t));

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
k=0;
fileID = fopen('RMS.txt','w');
fprintf(fileID,'%s %s %s %s %s %s %s\n','SD','r_x1','r_y1','r_th1','r_x2','r_y2','r_th2');
for(j=1:5:100)

disp(j);
xm=xtrue+normrnd(0,j,length(xtrue),1);
xm2=xtrue+normrnd(0,j,length(xtrue),1);

ym=ytrue+normrnd(0,j,length(ytrue),1);
ym2=ytrue+normrnd(0,j,length(ytrue),1);

thm=thtrue+normrnd(0,0.1*j,length(ytrue),1);
thm2=thtrue+normrnd(0,0.1*j,length(ytrue),1);
      
vxm=vxtrue+normrnd(0,1,length(ytrue),1);
vym=vytrue+normrnd(0,1,length(ytrue),1);
vwm=wtrue+normrnd(0,1,length(ytrue),1);
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
 %calculating the Kalman gains
Cov(:,:,i)=P;%covariance collection
KG(:,:,i)=K ;%Kalman gain collection

if(i<=100)
    K=P*C'/(C*P*C'+R);
    Xest(:,i)=Xest(:,i)+K*([xm(i); ym(i); thm(i)]-C*Xest(:,i));%Correcting: estimating the state
    Xcm(:,i)=[xm(i); ym(i); thm(i)]; %combined measurement for plotting purpose
    P=(eye(6)-K*C)*P;%Correcting: estimating P
    
elseif(i>100 && i<=300)
    K=P*C'/(C*P*C'+R);
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

r_x = sqrt( sum( (est_x(1,1:300)'-xtrue(1:300,1)).^2) / 300);
r_y = sqrt( sum( (est_y(1,1:300)'-ytrue(1:300,1)).^2) / 300);
r_th = sqrt( sum( (est_th(1,1:300)'-thtrue(1:300,1)).^2) / 300);
r_x2= sqrt( sum( (est_x(1,302:400)'-xtrue(302:400,1)).^2) / 100);
r_y2= sqrt( sum( (est_y(1,302:400)'-ytrue(302:400,1)).^2) / 100);
r_th2 = sqrt( sum( (est_th(1,302:400)'-thtrue(302:400,1)).^2) / 100);

fprintf(fileID,'%12f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n',j,r_x,r_y,r_th,r_x2,r_y2,r_th2);

end
fclose(fileID);

