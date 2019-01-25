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

% Parameters for sin/cos function
freqHz = 0.25;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = deg2rad( 90 );    % [rad]

original_position = group.getNextFeedback().position;
start_angle = deg2rad(90);

% Starts logging in the background
group.startLog( 'dir', 'logs' );  

gains = GainStruct();

% positionKps = [50 100];
% velocityKps = [50 100];

gains.positionKp = 4;
gains.positionKi = 0.05;
gains.velocityKp = 0;
group.send('gains', gains);

duration = 120; % [sec]
overall_timer = tic();
sin_timer = tic();
while toc(overall_timer) < duration
    % Even though we don't use the feedback, getting feedback conveniently 
    % limits the loop rate to the feedback frequency
    fbk = group.getNextFeedback();
    %curr_position = fbk.position;
    %fprintf("curr: %f, target: %f", curr_position, target_position);

    % Update position set point
    if (abs(fbk.effort - getGravityTorque(fbk.position)) > 1.5 && toc(sin_timer) > 0.5)
        wait_timer = tic();
        sin_angle = freq * toc(sin_timer) + start_angle; % current angle of where we are in the sin function
        % figure out how long to wait in order to change directions
        diff_angle = mod(sin_angle, pi);
        if (diff_angle < pi / 2)
            target_angle = sin_angle + 2 * ((pi / 2) - diff_angle);
        else
            %target_angle = 2 * pi - 2 * (diff_angle - (pi / 2));
            target_angle = sin_angle - 2 * (diff_angle - (pi / 2));
        end
        fprintf('curr_speed: %f, ', amp * cos( freq * toc(sin_timer) + start_angle));
        %target_time = (target_angle / (2 * pi)) * (1 / freqHz);
        start_angle = target_angle;
        fprintf('diff_angle: %f, target pos before: %f, ',diff_angle, cmd.position);
        % Wait until the sinusoid is headed the other direction
        % And then wait one more full period, to ensure a decent wait time
        while toc(wait_timer) < 1 % wait 1 second
           % do nothing 
            cmd.position = fbk.position;
            cmd.effort = getGravityTorque(fbk.position);
            group.send(cmd);
            group.getNextFeedback();
        end
        cmd.effort = [];
        
        sin_timer = tic();
        fprintf('new_speed: %f, ', amp * cos( freq * toc(sin_timer) + start_angle));
        fprintf('target after: %f\n', original_position - amp + amp * sin( freq * toc(sin_timer) + start_angle));
    end
    target_pos = original_position - amp + amp * sin( freq * toc(sin_timer) + start_angle);   
    cmd.position = target_pos;
    %gains.effortFF = getGravityTorque(target_pos);
    %group.send('gains', gains);
    group.send(cmd); 
end

% Stop logging and plot the position data using helper functions
log = group.stopLog();
HebiUtils.plotLogs( log , 'position' );

% FUNCTIONS
function effort_g = getGravityTorque(position)
    max_torque = 2.005;
    effort_g = max_torque * sin(position);
end

function scan
    
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
