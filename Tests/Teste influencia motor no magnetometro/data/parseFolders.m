%% load and parse data
%
% load, parse and save data as .mat file for rapid load
% 
%  
clear;
clc;
close all;

%----------------------------------------------------------------------
% Load folder
%----------------------------------------------------------------------
folder_name = uigetdir;
if(folder_name==0)
    return;
end
varname = strsplit(folder_name,filesep);
varname = strcat(varname(end),'.mat');

%----------------------------------------------------------------------
% Import Data
%----------------------------------------------------------------------
razor = importRazorData(strcat(folder_name,'/razor.csv'));



%----------------------------------------------------------------------
%change to nanoteslas
%----------------------------------------------------------------------
razor.magx=razor.magx/(1300*1E-5);
razor.magy=razor.magy/(1300*1E-5);
razor.magz=razor.magz/(1300*1E-5);

%----------------------------------------------------------------------
% remove salt and pepper noise type from razors
%----------------------------------------------------------------------
razor_filt = medfilt1(table2array(razor(:,4:12)));
razor(:,4:12) = array2table(razor_filt);

clear razor_filt;
save(char(varname));


%----------------------------------------------------------------------
% EXTRA: plot mag axis
%----------------------------------------------------------------------

Fs = 82; % Sample frequency for razors 
Fcut = 4; % Cut frequency of filter to apply on razor readings

% using an online lowpass method filter
% y(n)=(1−alpha)y(n−1)+alphax(n)
% see: http://dsp.stackexchange.com/questions/29355/detecting-outliers-noise-from-sensor-data/29445
%
%--------------------------------------------------------------------------
% define anonymous function (matlab is a pain in the *** and do not suport 
% function definitions inside any regular script!!!)
%--------------------------------------------------------------------------
onlineLowPass=@(in, yprevious, alpha) (1-alpha)*yprevious + alpha*in;

alpha = (1/Fs)/(1/Fcut + 1/Fs);
%--------------------------------------------------------------------------
% Run filter on razor 1
%--------------------------------------------------------------------------
aux = table2array(razor(:,4:12));

sizes = size(aux);
for I = 1:sizes(2)
 for ii=2:sizes(1)
	aux(ii,I) = onlineLowPass(aux(ii,I),aux(ii-1,I),alpha);
 end
end
acc  = aux(:,1:3);
gyro = aux(:,4:6);
mag  = aux(:,7:9);

%--------------------------------------------------------------------------
% Correct time for the case there is bad values (delayed)
%--------------------------------------------------------------------------
t    = table2array(razor(:,2));
lenT = length(t);
t    = t-t(1);
t    = milliseconds(t);
p    = polyfit((0:lenT-1)',t,1);
t = (p(1)*(1:lenT)-p(1))';
%==========================================================================
%--------------------------------------------------------------------------
% Plot magnetometermeter
%--------------------------------------------------------------------------
%==========================================================================
h1 = zeros(1,3);
%co = get(groot,'defaultAxesColorOrder');
figure();
%  (x-axis)
h1(1)=subplot(3,1,1);
plot(t,mag(:,1),'Color',[0  0.4470  0.7410]);
title('\color[rgb]{0  0.4470  0.7410} x-axis magnetic field [uT]',...
	'FontSize',9, 'FontWeight','normal');
set(gca, 'XGrid','on');
%  (y-axis)
h1(2)=subplot(3,1,2);
plot(t,mag(:,2),'Color',[0.8500    0.3250    0.0980]);
title('\color[rgb]{0.8500    0.3250    0.0980} y-axis magnetic field [uT]',...
	'FontSize',9, 'FontWeight','normal');
set(gca, 'XGrid','on');
% (z-axis)
h1(3)=subplot(3,1,3);
plot(t,mag(:,3),'Color',[0.9290    0.6940    0.1250]);
title('\color[rgb]{0.9290    0.6940    0.1250} z-axis magnetic field [uT]',...
	'FontSize',9, 'FontWeight','normal');
set(gca, 'XGrid','on');
xlabel('time [s]');	

linkaxes(h1, 'x');
zoom xon;

