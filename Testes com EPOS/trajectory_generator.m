function [y, t] = trajectory_generator(y0, yf,stepSize)
%TRAJECTORY_GENERATOR generates a smooth trajectory for EPOS
%
%   [y, t] = TRAJECTORY_GENERATOR(y0, yf)
%   Generates  equally spaced (1000 steps by default) trajectory points.
%
%   [y, t] = TRAJECTORY_GENERATOR(y0, yf, stepSize)
%   Generates  equally spaced trajectory points using the number os steps.
%
%   Returns:
%   y is a vector of the trajectory.
%   t is the time vector.
% 
% The objective is to create a trajectory for positioning the steering
% wheel but with a specific profile:
%
%  * trajectory should never go above max speed.
%  * trajectory should never go above max acceleration.
%  * accelerations and velocities at starting and ending poinst should be
% zero.
%
%
%
% The formulas used in these function was derived from the paper:
% <https://www.researchgate.net/publication/267794207_Motion_profile_planning_for_reduced_jerk_and_vibration_residuals>
%

% already on final point.
if (~exist('y0','var') || ~exist('yf','var'))
	fprintf('Starting point or final point not given\n');
	return;
end

if(y0 == yf)
	fprintf('Starting point is equal to final point\n');
	return;
end

% if steps is not supplied, calculate step distance using 200 number of
% steps.
if ~exist('steps','var') || isempty(stepSize)
  stepSize = 0.25;
end
% use always positive values
stepSize = abs(stepSize);


Tmax = 1.7; % max period for 1 rotation;

% 1 rev = 3600*4 [qc]
countsPerRev = 3600*4;

%
% 1Hz = 60rpm = 360degrees/s
%
% 360 degrees = sensor resolution * 4
%
% this yields: 1Hz = (sensor resolution * 4)/s 
%
% Fmax = 1 / Tmax;
%
% maxSpeed = 60 rpm/Tmax [rpm]= 
%          = 360degrees/Tmax [degrees/s]=
%          = (sensor resolution *4)/Tmax [qc/s]

maxSpeed = countsPerRev/Tmax; % degrees per sec

% max acceleration must be experimental obtained.
% reduced and fixed. 
maxAcceleration = 4000;  % [qc]/s^2

% maximum interval for both the accelleration  and deceleration phase are:
%
T1max = 2 * maxSpeed/maxAcceleration;

% the max distance covered by these two phase (assuming acceleration equal 
% deceleration) is 2* 1/4 * Amax * T1max^2 = 1/2 * Amax * T1max^2 = 2Vmax^2/Amax
maxL13 = 2* maxSpeed^2/maxAcceleration;

% absolute of displacement
l = abs(yf-y0);

% do we need  a constant velocity phase? 
if(l>maxL13)
	T2 = 2*(l- maxL13)/(maxAcceleration*T1max);
	T1 = T1max;
	T3 = T1max;
else
	T1 = sqrt(2*l/maxAcceleration);
	T2 = 0;
	T3 = T1;
end

%create a time vectors.
TotalT = (T1+T2+T3);
t = 0:stepSize:TotalT;

t1 = T1;
t2 = T2+t1;
%t3 = t2+T3;

% determine the sign of movement
moveUp_or_down = sign(yf-y0);
y = zeros(1, length(t));
% create first movement
for I = 1:length(t)
	if(t(I) <= t1)
		y(I) = y0 + moveUp_or_down * maxAcceleration/2 * (T1/(2*pi))^2 * (1/2 * (2* pi/T1 * t(I))^2 - (1-cos(2/T1 * pi * t(I))));
	elseif(T2 > 0 && (t(I)>t1 && t(I)<= t2))
		y(I) =y0 + moveUp_or_down * (1/4 * maxAcceleration * T1^2 + 1/2 * maxAcceleration*T1* (t(I)-t1));
	else
		y(I) = y0 + moveUp_or_down * (1/4 * maxAcceleration * T1^2 + 1/2 * maxAcceleration * T1*T2 + ...
			maxAcceleration/2 * (T1/(2*pi))^2 * ((2*pi)^2*(t(I)-t2)/T1 -1/2*(2*pi/T1 *(t(I)-t2))^2 + (1- cos(2*pi/T1*(t(I)-t2)))));
	end
end






