images = imageDatastore(fullfile(toolboxdir('vision'),'visiondata', 'calibration','webcam'));
imageFileNames = images.Files(1:5);
[imagePoints,boardSize] = detectCheckerboardPoints(imageFileNames);
squareSide = 25;
worldPoints = generateCheckerboardPoints(boardSize,squareSide);
I = readimage(images,1); 
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints,worldPoints, 'ImageSize',imageSize);
figure
showExtrinsics(cameraParams,'patternCentric');
                              