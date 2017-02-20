clc;
close all;
clear all;


[filename, pathname] = uigetfile('*.mat');

if( filename == 0)
	return;
end
load(strcat(pathname,filename));


% plot reference input variable
% plot output variable
figure('name',filename);
plot(tx,x, ty, y,'g');
xlabel('Time [s]');
ylabel('Position [qc]');
title('Trajectory following test');
legend('input','output')
set(gcf,'PaperPositionMode','auto');
print(strcat(pathname,filename(1:end-4),'_position.svg'),'-dsvg');



% plot error estimation
t = 0:0.125:ty(end);
input_est = interp1(tx, x, t,'pchip');
output_est = interp1(ty, y, t,'pchip');
following_error =input_est-output_est;
figure('name','Error');
plot(t, following_error);
xlabel('Time [s]');
ylabel('Position [qc]');
title('Trajectory following test - Error');
set(gcf,'PaperPositionMode','auto');
print(strcat(pathname,filename(1:end-4),'_error.svg'),'-dsvg');

% calculate approximation of acceleration and velocity
velocity = gradient(output_est,0.125);
acceleration = gradient(velocity, 0.125);
figure('name','Acc and Vel');
subplot(3,1,1)
plot(t,output_est);
xlabel('Time [s]');
ylabel('qc');
title('Position');
subplot(3,1,2)
plot(t,velocity,'r');
xlabel('Time [s]');
ylabel('qc/s');
title('Velocity');
subplot(3,1,3)
plot(t,acceleration,'g');
xlabel('Time [s]');
ylabel('qc/s^2');
title('acceleration');
set(gcf,'PaperPositionMode','auto');
print(strcat(pathname,filename(1:end-4),'_vel_acc.svg'),'-dsvg');
