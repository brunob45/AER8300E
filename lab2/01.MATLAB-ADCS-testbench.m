%#! /usr/local/bin/octave -qf
%#Plausible interpreter paths /usr/bin/octave or /usr/local/bin/octave or ..
%%
clc
clear all
close all

%%
for t = 0:10
    [t, targetPos(t)]
end

%%

% Time constants (feel free to change them)
timestep = 1; % seconds
t0 = 0;       % start time in seconds from Epoch
t1 = 5 * 60;  % experiment duration in seconds


% Set initial state for each axis: 0 attitude and 0 rot speed
satroll  = [0 0]; % sat body x
satpitch = [0 0]; % sat body y
satyaw   = [0 0]; % sat body z

for t = t0:t1
	
	% Get yaw, pitch, roll to the target in radiants
	[toTargetY, toTargetP, toTargetR] = toTarget(satyaw, satpitch, satroll, t);
    printf("to targetRPY = %f, %f, %f\n", toTargetR * 180 / pi, toTargetP * 180 / pi, toTargetY * 180 / pi);
	
    % Develop a (PID) controller to apply torque (in the opposite sense)
	
	% YOUR CODE HERE
	% YOUR CODE HERE
	% YOUR CODE HERE
	
    uroll = 0;
    upitch = 0;
    uyaw = 0;
    printf('u = %f, %f, %f\n', uroll, upitch, uyaw);
	
    % Calculate new satellite attitude in radiants
    satroll  = satupdate(satroll,  uroll,  timestep);
    satpitch = satupdate(satpitch, upitch, timestep);
    satyaw   = satupdate(satyaw,   uyaw,   timestep);
	
    printf('\n');
	
end

%% Functions
function [x,y,z] = targetPos(t)
    theta_0 = 0;
    d_theta = 1;
    
    phi_0 = 0;
    d_phi = 1;
    
    radius = 1;
    theta = theta_0 + d_theta * t;
    phi = phi_0 + d_phi * t;
    
    x = radius * cos (theta) * sin (phi);
    y = radius * sin (theta) * sin (phi);
    z = radius * cos(phi);
end

function [yaw,pitch,roll] = toTarget(satyaw, satpitch, satroll, t)
    % compute yaw, pitch, roll between the traget and the satellite coordinate reference system
	% the target can move on any circular path around the absolute reference system
	
	%compute yaw, pitch, roll w.r.t. the satellite
	yaw = 0;
	pitch = 0;
	roll = 0;
	
	% YOUR CODE HERE
	% YOUR CODE HERE
	% YOUR CODE HERE
end

% Updates the state of the satellite.
% It's a simple Euler integrator for 1-axis rotation.
% PARAM  state:    a 2d array [attitude rot_speed]
% PARAM  control:  the torque
% PARAM  timestep: the integration time step
% RETURN newstate: a 2d array [new_attitude new_rot_speed]
function newstate = satupdate(state, control, timestep)
    newstate = [0 0];
    newstate(1) = state(1) + state(2) * timestep + 0.5 * control * timestep^2;
    newstate(2) = state(2) + control * timestep;
end














