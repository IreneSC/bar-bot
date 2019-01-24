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
HebiLookup.initialize();

familyName = 'Arm';
moduleNames = 'tapedispenser';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Open-Loop Controller (Position)
% The command struct has fields for position, velocity, and effort.  
% Fields that are empty [] or NaN will be ignored when sending.
cmd = CommandStruct(); 

% Starts logging in the background
group.startLog( 'dir', 'logs' );  

% Parameters for sin/cos function
freqHz = 1;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = deg2rad( 45 );    % [rad]

original_position = group.getNextFeedback().position;
start_angle = deg2rad(90);

duration = 10; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Even though we don't use the feedback, getting feedback conveniently 
    % limits the loop rate to the feedback frequency
    fbk = group.getNextFeedback();  

    % Update position set point
    cmd.position = (original_position - amp) + amp * sin( freq * toc(timer) + start_angle);   
    group.send(cmd); 
    
end

% Stop logging and plot the position data using helper functions
log = group.stopLog();
HebiUtils.plotLogs( log, 'position' );
HebiUtils.plotLogs( log, 'velocity' );
