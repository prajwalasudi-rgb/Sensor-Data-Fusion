detectedLocations_x = num2cell(2*randn(1,40) + (1:40));
detectedLocations_x{1} = [];
  for idx = 16: 25
      detectedLocations_x{idx} = [];
  end

detectedLocations_y = num2cell(2*randn(1,40) + (1:40));
detectedLocations_y{1} = [];
  for idx = 16: 25
      detectedLocations_y{idx} = [];
  end  
detectedLocations_z = num2cell(2*randn(1,40) + (1:40));
detectedLocations_z{1} = [];
  for idx = 16: 25
      detectedLocations_x{idx} = [];
  end
  
x= linspace (0,2*pi*10,10);
y= sin(x);
z= ones(1,10)*10;
%detectedLocations = cat(x,y,z);
ylabel('y');
xlabel('x');
zlabel('z');
plot3(x,y,z);
mat_g =zeros(10,3);
for i = 1:10
    mat_g(i,1) =x(i);
    mat_g(i,2) =y(i);
    mat_g(i,3) =z(i);
end

mat_l =zeros(10,3);


M = makehgtform('translate',[0 0 0],'xrotate',0,'yrotate',0,'zrotate',0);



for i= 1:10
    B=reshape(mat_g(i,:),[],1);
    disp(B);
    disp([0;1;0]);
 
    mat_l(i,:);
    %= global2localcoord(B,'rr',[1;1;1]);
    ret = global2localcoord(B,'rr',[0;0;1]);
    mat_l(i,:)= ret.';
end 

  
%{
figure;
hold on;
ylabel('Location');
ylim([0,50]);
xlabel('Time');
xlim([0,length(detectedLocations)]);
kalman = [];
for idx = 1: length(detectedLocations)
   location = detectedLocations{idx};
   if isempty(kalman)
     if ~isempty(location)

       stateModel = [1 1;0 1];
       measurementModel = [1 0];
       kalman = vision.KalmanFilter(stateModel,measurementModel,'ProcessNoise',1e-4,'MeasurementNoise',4);
      kalman.State = [location, 0];
     end
   else
     trackedLocation = predict(kalman);
     if ~isempty(location)
       plot(idx, location,'k+');
      d = distance(kalman,location);
       title(sprintf('Distance:%f', d));
       trackedLocation = correct(kalman,location);
     else
       title('Missing detection');
     end
     pause(0.2);
     plot(idx,trackedLocation,'ro');
   end
 end
legend('Detected locations','Predicted/corrected locations');


x = 1;
len = 100;
z = x + 0.1 * randn(1,len);


stateTransitionModel = 1;
measurementModel = 1;
obj = vision.KalmanFilter(stateTransitionModel,measurementModel,'StateCovariance',1,'ProcessNoise',1e-5,'MeasurementNoise',1e-2);

z_corr = zeros(1,len);
for idx = 1: len
 predict(obj);
 z_corr(idx) = correct(obj,z(idx));
end



figure, plot(x * ones(1,len),'g-'); 
hold on;
plot(1:len,z,'b+',1:len,z_corr,'r-');
legend('Original signal','Noisy signal','Filtered signal');
%}



