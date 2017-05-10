function [outVar, tout, inVar, tin] = testStability( increment, tfinal, period)
%TESTSTABILITY check fixed value stability
%   Using a fixed value square signal, check how good is the controller 
%   system in terms of stability.

%--------------------------------------------------------------------------
% Constants sections
% For information about these values check trajectories generation examples
%--------------------------------------------------------------------------
% Tmax = 1.7; % max period for 1 rotation;
% countsPerRev = 3600*4;
% maxSpeed = countsPerRev/Tmax; % qc per sec
% maxAcceleration = 6000;  % [qc]/s^2
% 
% % maximum interval for both the accelleration  and deceleration phase are:
% T1max = 2 * maxSpeed/maxAcceleration;
% 
% % the max distance covered by these two phase (assuming acceleration equal
% % deceleration) is 2* 1/4 * Amax * T1max^2 = 1/2 * Amax * T1max^2 = 2Vmax^2/Amax
% maxL13 = 2* maxSpeed^2/maxAcceleration;

% max error in quadrature counters
MAXERROR = 5000;

%% connect to epos
addpath('/home/bruno/DATA/Dropbox/Tese/Fiat-Elektra/Maxon-Epos/Matlab Lib');

%--------------------------------------------------------------------------
% is epos already instanciated?
%--------------------------------------------------------------------------
if(~evalin('base','exist(''epos'', ''var'')'))
	evalin('base', 'epos =  Epos()');
	epos = evalin('base','epos');
else
	epos = evalin('base','epos');
end
%--------------------------------------------------------------------------
% configure and setup for position mode
%--------------------------------------------------------------------------
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
if(~epos.changeEposState('enable operation'))
	warndlg('Failed to change Epos state');
	return;
end
%--------------------------------------------------------------------------
% get current point
%--------------------------------------------------------------------------
[y0,OK] = epos.readPositionValue;
if (~OK)
	return
end
y0=double(y0);
%--------------------------------------------------------------------------
% Find remaining constants
%--------------------------------------------------------------------------
% % absolute of displacement
% l = abs(yf-y0);
% 
% % do we need  a constant velocity phase?
% if(l>maxL13)
% 	T2 = 2*(l- maxL13)/(maxAcceleration*T1max);
% 	T1 = T1max;
% 	T3 = T1max;
% else
% 	T1 = sqrt(2*l/maxAcceleration);
% 	T2 = 0;
% 	T3 = T1;
% end
% 
% % time constanst
% t1 = T1;
% t2 = T2+t1;
% t3 = T3+t2; % final time
% 
% % determine the sign of movement
% moveUp_or_down = sign(yf-y0);

%% start preparations

fig = figure('Name','Stability test');
%--------------------------------------------------------------------------
% Refenrece plot
%--------------------------------------------------------------------------
h1 = plot(0,y0);
xlabel('Time[s]');
ylabel('Position[qc]');
title('Stability hold test');
hold on;
%--------------------------------------------------------------------------
% plot for outVar
%--------------------------------------------------------------------------
h2 = plot(0,y0,'g');

%
inVar = [];
outVar = [];
tin = [];
tout = [];
I = 1;
flag = true;
ref_error = [];
ref = y0;
t0 = clock;
tchange = period;
invertFlag = true;
while (flag)
	% check current time
	tin(I,1) = etime(clock,t0);
	%time to exit?
	if( tin(I,1) > tfinal)
		flag = false;
		inVar(I,1) = ref;
		outVar(I,1) = epos.readPositionValue;
		tout(I,1) = etime(clock,t0);
		ref_error(I,1) = inVar(I)-outVar(I,1);
		% return to initial position if not
		epos.setPositionModeSetting(y0);
		pause(0.5);
	else
		% is time to change ref?
		if(tin(I,1)>=tchange)
			tchange =tin(I,1)+period;
			if(invertFlag)
				invertFlag = false;
				ref=y0+increment;
				epos.setPositionModeSetting(ref);
				
			else
				invertFlag = true;
				epos.setPositionModeSetting(y0);
				ref=y0;
			end
		end
		inVar(I,1) = ref;
		outVar(I,1) = epos.readPositionValue;
		tout(I,1) = etime(clock,t0);
		ref_error(I,1) = inVar(I)-outVar(I,1);
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%                        USE AS PRECAUTION
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		if(abs(ref_error(I,1))>MAXERROR)
			epos.changeEposState('shutdown');
			waitfor(warndlg('Something seems wrong, error is growing to mutch!!!'));
			return;
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
end

