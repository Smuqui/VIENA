function [tin,inVar,tout,outVar,config] = move_to_position(yf)
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
if(nargin<1)
	fprintf('no final point given!\n');
	return;
end
%% constants
%
%
%
% Tmax = 1.7 seems to be the limit before oscillations.
%
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
maxAcceleration = 6000;  % [qc]/s^2

% maximum interval for both the accelleration  and deceleration phase are:
%
T1max = 2 * maxSpeed/maxAcceleration;

% the max distance covered by these two phase (assuming acceleration equal
% deceleration) is 2* 1/4 * Amax * T1max^2 = 1/2 * Amax * T1max^2 = 2Vmax^2/Amax
maxL13 = 2* maxSpeed^2/maxAcceleration;

% max error in quadrature counters
MAXERROR = 5000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% connect to epos
addpath('/home/bruno/DATA/Dropbox/Tese/Fiat-Elektra/Maxon-Epos/Matlab Lib');
if(~evalin('base','exist(''epos'', ''var'')'))
	evalin('base', 'epos =  Epos()');
	epos = evalin('base','epos');
else
	epos = evalin('base','epos');
end
epos.begin('/dev/ttyUSB0');

if(~epos.setPositionControlParam(50,1,3,5,3))
	warndlg('Failed to change Epos position parameters');
	return;
end

if(~epos.changeEposState('fault reset'))
	warndlg('Failed to change Epos state');
	return;
end
if(~epos.changeEposState('shutdown'))
	warndlg('Failed to change Epos state');
	return;
end
if(~epos.changeEposState('switch on'))
	warndlg('Failed to change Epos state');
	return;
end
% if(~epos.changeEposState('enable operation'))
% 	warndlg('Failed to change Epos state');
% 	return;
% end

% get ref
[y0,OK] = epos.readPositionValue;

if (~OK)
	return
end

y0=double(y0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find remaining constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% time constanst
t1 = T1;
t2 = T2+t1;
t3 = T3+t2; % final time

% determine the sign of movement
moveUp_or_down = sign(yf-y0);


figure();
% plot for inVar;
h1 = plot(0,y0);
xlabel('Time[s]');
ylabel('Position[qc]');
title('Position Follow test');
ylim(1.1*sort([y0 yf]));
hold on;
% plot for outVar
h2 = plot(0,y0,'g');
inVar = [];
outVar = [];
tin = [];
tout = [];
t0 = clock;
I = 1;
flag = true;

ref_error = [];
while (flag)
	% check current time
	tin(I,1) = etime(clock,t0);
	%time to exit?
	if( tin(I,1) > t3)
		flag = false;
		inVar(I,1) = yf;
		epos.setPositionModeSetting(yf);
		outVar(I,1) = epos.readPositionValue;
		tout(I,1) = etime(clock,t0);
		ref_error(I,1) = inVar(I)-outVar(I,1);
	else
		% get reference position for that time
		if(tin(I,1) <= t1)
			inVar(I,1) =y0+ moveUp_or_down * maxAcceleration/2 * (T1/(2*pi))^2 * (1/2 * (2* pi/T1 * tin(I,1))^2 - (1-cos(2/T1 * pi * tin(I,1))));
		elseif(T2 > 0 && (tin(I,1)>t1 && tin(I,1)<= t2))
			inVar(I,1) = y0+ moveUp_or_down * (1/4 * maxAcceleration * T1^2 + 1/2 * maxAcceleration*T1* (tin(I,1)-t1));
		else
			inVar(I,1) = y0+ moveUp_or_down * (1/4 * maxAcceleration * T1^2 + 1/2 * maxAcceleration * T1*T2 + ...
				maxAcceleration/2 * (T1/(2*pi))^2 * ((2*pi)^2*(tin(I,1)-t2)/T1 -1/2*(2*pi/T1 *(tin(I,1)-t2))^2 + (1- cos(2*pi/T1*(tin(I,1)-t2)))));
		end
		epos.setPositionModeSetting(inVar(I,1));
		outVar(I,1) = epos.readPositionValue;
		tout(I,1) = etime(clock,t0);
		ref_error(I,1) = inVar(I)-outVar(I,1);
		if(abs(ref_error(I,1))>MAXERROR)
			epos.changeEposState('shutdown');
			waitfor(warndlg('Something seems wrong, error is growing to mutch!!!'));
			return;
		end
	end
	I = I+1;
	h1.XData = tin;
	h1.YData = inVar;
	h2.XData = tout;
	h2.YData = outVar;
	drawnow limitrate nocallbacks;
end
axis auto;
legend('reference','output');
drawnow;
if(~epos.changeEposState('shutdown'))
	warndlg('Failed to change Epos state');
	return;
end
config = epos.readPositionControlParam;
