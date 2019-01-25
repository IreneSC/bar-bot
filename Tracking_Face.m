% lab2 part3

%% Setup
% For camera
% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam('HD Pro Webcam C920');

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

gainsTracking = [1, 0.1, 0.001]; % kP, kD, kI
errorTracking = zeros(1,3); % error, derivative of error, and integration of error

% For Hebi
clear *;
close all;
HebiLookup.initialize();

familyName = 'Arm';
moduleNames = 'tapedispenser';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Open-Loop Controller (Position)
% The command struct has fields for position, velocity, and effort.  
% Fields that are empty [] or NaN will be ignored when sending.
cmd = CommandStruct(); 

%% Loop
while true
    sweep
    centroidx = faceTrack(faceDetector, pointTracker, cam, videoPlayer);
    frameMin = 0;
    frameMax = frameSize(2);
    errorP = geterrorcam(centroidx, frameMin, frameMax);
    
    % PID shit
    errorTracking(2) = errorP - errorTracking(1);
    errorTracking(3) = errorTracking(3) + errorP;
    errorTracking(1) = errorP;
    
    response = PID(errorTracking, grainsTracking);

    cmd.position = response + cmd.position;
    group.send(cmd); 
end


%% End

% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);

% returns the normed error of the position of an object from the center of
% the camera
% @param pixel positions of object and max/min of screen edge
% returns error on [-1,1]
function [error] = getErrorCam(position, bound1, bound2)
    center = (bound1 + bound2)/2;
    width = (bound2 - bound1)/2;
    error = (position - center)/width;
end

function response = PID(error, gains)
    response = error(1) * gains(1) + error(2) * gains(2) + error(3) * gains(3);
end