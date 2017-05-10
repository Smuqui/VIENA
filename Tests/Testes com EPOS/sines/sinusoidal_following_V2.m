clc;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Add EPOS library to path
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('/home/bruno/DATA/Dropbox/Tese/Fiat-Elektra/Maxon-Epos/Matlab Lib');

% Instanciate class or use previous created
if(~exist('epos', 'var'))
	epos = Epos(); 
end

% open device
epos.begin('/dev/ttyUSB0');

% At the beggining we should check if the EPOS has any error.
% In a cold start, it will complain about the fact there is no CAN
% connected ( we are using rs232 port) so apply a fault reset.
% Even if no error, applying a fault reset will do no harm.
%

if(~epos.changeEposState('fault reset'))
	warndlg('Failed to change Epos state');
	return;
end

% change position control parameter gains?
if(~epos.setPositionControlParam(50, 1, 3, 5, 3))
	warndlg('Failed to set position control parameters');
	return;
end

% just in case it is not by default in position mode
if(~epos.setOpMode(-1))
	warndlg('Failed to set in position control mode');
	return;
end

% procedure for enabling position control mode
%
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


% create a sine wave
%
% T = period
% !!!Use carefully!!! 
% Velocities above 6rpm = 36 degrees/s start causing instability and
% oscilations.
% This means T = 10 for 1 turn amplitude value. If changing amplitudeFactor
% please adjust also the T accordingly.

T = 20;
F = 1/T;
amplitudeFactor = 1; % how many turns should we do?
% make 3 complete cycles
tend = 3*T;

% get the current position as starting point.
[ref,OK] = epos.readPositionValue;
if (~OK)
	return
end
ref=double(ref);

% create figure to plot values "online"
figure();
% line with 1 turn values
line([0 tend],ref+[(3600*4) (3600*4)],'LineStyle','--', 'Color','r');
hold on;
line([0 tend],ref+[-(3600*4) -(3600*4)],'LineStyle','--', 'Color','r');
xlabel('Time[s]');
ylabel('Position[qc]');
title('Sinusoidal Follow test');
% plot for inVar;
h1 = plot(0,ref);
% plot for outVar
h2 = plot(0,ref,'g');

% variables to store data
inVar = []; % input position, reference.
outVar = []; % output position. Sensor reading

% sending commands to EPOS is really SLOOOOOOWWWWW in Matlab. We have to
% take care of this things.
tin = []; % time for the input values
tout = []; % time for the output values
t0 = clock; % inicial time value

I = 1;
flag = true; % boolean flag to end the loop, when destination is reached.

ref_error = [];
while(flag)
	tin(I,1) = etime(clock,t0);
	% have we reached target?
	if(tin(I,1)>tend)
		flag = false; % yes we have
	else
		% using current time, what is the value expected for the reference?
		inVar(I,1) = ref+amplitudeFactor*3600*4*sin(2*pi*F*tin(I));
		% send reference value to EPOS
		epos.setPositionModeSetting(inVar(I,1));
		% read current position value
		outVar(I,1) = epos.readPositionValue;
		% what is the time for that position?
		tout(I,1) = etime(clock,t0);
		ref_error(I,1) = inVar(I,1)-outVar(I,1);
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%                        USE AS PRECAUTION
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		if(abs(ref_error(I,1))>5000)
			epos.changeEposState('shutdown');
			waitfor(warndlg('Something seems wrong, error is growing to mutch!!!'));
			return;
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% fast plot, update only when possible
		I = I+1;
		h1.XData = tin;
		h1.YData = inVar;
		h2.XData = tout;
		h2.YData = outVar;
		drawnow limitrate nocallbacks; 
	end
end
% update final graphics
legend('+1 turn','-1 turn','ref','output');
drawnow;
% disable EPOS
if(~epos.changeEposState('shutdown'))
	warndlg('Failed to change Epos state');
	return;
end
config = epos.readPositionControlParam;

