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

target_pos = deg2rad(45);

goToPosition(target_pos, group);
holdPosition(target_pos, group);

% FUNCTIONS
function effort_g = getGravityTorque(position)
    max_torque = 2.005;
    effort_g = max_torque * sin(position);
end

function goToPosition(target_position, group)
    cmd = CommandStruct(); 

    % Starts logging in the background
    group.startLog( 'dir', 'logs' );  

    gains = GainStruct();

    % positionKps = [50 100];
    % velocityKps = [50 100];

    gains.positionKp = 50;
    gains.velocityKp = 50;
    group.send('gains', gains);

    duration = 3; % [sec]
    timer = tic();
    while toc(timer) < duration

        % Even though we don't use the feedback, getting feedback conveniently 
        % limits the loop rate to the feedback frequency
        fbk = group.getNextFeedback();  

        % Update position set point
        cmd.position = target_position; 
        group.send(cmd); 
    end
end

function holdPosition(target_position, group)
    % TODO: support going to a position less than current
    % as well as negative positions
    cmd = CommandStruct(); 

    % Starts logging in the background
    group.startLog( 'dir', 'logs' );  

    gains = GainStruct();

    % positionKps = [50 100];
    % velocityKps = [50 100];

    gains.positionKp = 0;
    gains.positionKi = 0;
    gains.positionKd = 0;
    gains.positionFF = 0;
    gains.velocityKp = 0;
    gains.velocityKi = 0;
    gains.velocityKd = 0;
    gains.velocityFF = 0;
    group.send('gains', gains);

    duration = 3; % [sec]
    timer = tic();
    while toc(timer) < duration
        % Even though we don't use the feedback, getting feedback conveniently 
        % limits the loop rate to the feedback frequency
        fbk = group.getNextFeedback();  
        %curr_position = fbk.position;
        %target_position = min(curr_position + 0.1, position);
        %fprintf("curr: %f, target: %f", curr_position, target_position);

        % Update position set point
        cmd.effort = getGravityTorque(target_position); 
        cmd.position = target_position;
        group.send(cmd); 
    end

    % Stop logging and plot the position data using helper functions
    log = group.stopLog();
    HebiUtils.plotLogs( log , 'position' );

end
