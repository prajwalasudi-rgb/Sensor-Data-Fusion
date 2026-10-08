images = imageDatastore(fullfile(toolboxdir('vision'),'visiondata', 'calibration', 'webcam'));
[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);
squareSize = 29;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
I = readimage(images,1);
imageSize = [size(I,1), size(I,2)];
cameraParams = estimateCameraParameters(imagePoints,worldPoints, 'ImageSize',imageSize);
imOrig = imread(fullfile(matlabroot,'toolbox','vision','visiondata', 'calibration','slr','image9.jpg'));
figure
imshow(imOrig);
title('Input Image');
[im,newOrigin] = undistortImage(imOrig,cameraParams,'OutputView','full');
[imagePoints,boardSize] = detectCheckerboardPoints(im);
imagePoints = [imagePoints(:,1) + newOrigin(1),imagePoints(:,2) + newOrigin(2)];
[rotationMatrix, translationVector] = extrinsics(imagePoints,worldPoints,cameraParams);
[orientation, location] = extrinsicsToCameraPose(rotationMatrix,translationVector);
figure
plotCamera('Location',location,'Orientation',orientation,'Size',20);
hold on
pcshow([worldPoints,zeros(size(worldPoints,1),1)],'VerticalAxisDir','down','MarkerSize',40);