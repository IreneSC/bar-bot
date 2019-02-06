% Send position commands, log in the background, and plot offline.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% June 2018

%% Setup
clear *;
close all;

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
histogramTracker = vision.HistogramBasedTracker();

% Create the webcam object.
cam = webcam('HD Pro Webcam C920');

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

HebiLookup.initialize();

familyName = 'Arm';
moduleNames = 'tapedispenser';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Open-Loop Controller (Position)
% The command struct has fields for position, velocity, and effort.  
% Fields that are empty [] or NaN will be ignored when sending.
cmd = CommandStruct(); 

% Parameters for sin/cos function
freqHz = 0.12;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = deg2rad( 90 );    % [rad]

original_position = group.getNextFeedback().position;
start_angle = deg2rad(90);

% Starts logging in the background
group.startLog( 'dir', 'logs' );  

gains = GainStruct();

gains.positionKp = 1.5;
gains.velocityKp = 0;
gains.positionTargetLowpassGain = 0.1;
group.send('gains', gains);

duration = 120; % [sec]
overall_timer = tic();
sinusoid_timer = tic();
tracking = false;
camera_rate = 10;
index = 0;
numPts = 0;
oldPoints = 0;
bboxPoints = 0;
while toc(overall_timer) < duration
    if (mod(index, camera_rate) == 0)
        [centroidx, centroidy, isFaceFirstDetected, isFaceDetected, numPts, oldPoints, bboxPoints, histogramTra] = faceTrackHistogram(faceDetector, histogramTracker, cam, videoPlayer, numPts,oldPoints, bboxPoints);
    end
    fbk = group.getNextFeedback();
    if(~isFaceDetected && tracking && ~isFaceFirstDetected)
        %restart scan from this position
        tracking = false;
        original_position = fbk.position;
        sinusoid_timer = tic();
        cmd.position = original_position;
        group.send(cmd);
    elseif(~isFaceDetected)
        target_pos = original_position - amp + amp * sin( freq * toc(sinusoid_timer) + start_angle);   
        cmd.position = target_pos;
        group.send(cmd); 
    else
        frameMin = 0;
        frameMax = frameSize(2);
        cmd.position = -deg2rad(78.0/2)*getErrorCam(centroidx, frameMin, frameMax) + fbk.position;
        tracking = true;
        group.send(cmd);
    end
    
end

% Stop logging and plot the position data using helper functions
log = group.stopLog();
HebiUtils.plotLogs( log , 'position' );

% FUNCTIONS
% returns the normed error of the position of an object from the center of
% the camera
% @param pixel positions of object and max/min of screen edge
% returns error on [-1,1]
function [error] = getErrorCam(position, bound1, bound2)
    center = (bound1 + bound2)/2;
    width = (bound2 - bound1)/2;
    error = (position - center)/width;
end
