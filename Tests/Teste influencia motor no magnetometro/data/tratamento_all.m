clear;
clc;
close all;

load('all.mat');

maxsizes = [size(fiat.speed_0hz.mag);
	size(fiat.speed_10hz.mag);
	size(fiat.speed_20hz.mag);
	size(fiat.speed_30hz.mag)];

maxDim = min(maxsizes(:,1));

mag = [fiat.speed_0hz.mag(1:maxDim,:),...
	fiat.speed_10hz.mag(1:maxDim,:),...
	fiat.speed_20hz.mag(1:maxDim,:),...
	fiat.speed_30hz.mag(1:maxDim,:)];
t = [fiat.speed_0hz.t(1:maxDim,:),...
	fiat.speed_10hz.t(1:maxDim,:),...
	fiat.speed_20hz.t(1:maxDim,:),...
	fiat.speed_30hz.t(1:maxDim,:)];


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
plot(t(:,1),mag(:,1),t(:,2),mag(:,4),t(:,3),mag(:,7),t(:,4),mag(:,10));
title(' x-axis magnetic field [uT]',...
	'FontSize',9, 'FontWeight','normal');
set(gca, 'XGrid','on');
legend('speed 0hz','speed 10hz','speed 20hz','speed 30hz','Location','northeastoutside');
%  (y-axis)
h1(2)=subplot(3,1,2);
plot(t(:,1),mag(:,2),t(:,2),mag(:,5),t(:,3),mag(:,8),t(:,4),mag(:,11));
title('y-axis magnetic field [uT]',...
	'FontSize',9, 'FontWeight','normal');
set(gca, 'XGrid','on');
legend('speed 0hz','speed 10hz','speed 20hz','speed 30hz','Location','northeastoutside');
% (z-axis)
h1(3)=subplot(3,1,3);
plot(t(:,1),mag(:,3),t(:,2),mag(:,6),t(:,3),mag(:,9),t(:,4),mag(:,12));
title('z-axis magnetic field [uT]',...
	'FontSize',9, 'FontWeight','normal');
set(gca, 'XGrid','on');
legend('speed 0hz','speed 10hz','speed 20hz','speed 30hz','Location','northeastoutside');
xlabel('time [ms]');	

linkaxes(h1, 'x');
zoom xon;