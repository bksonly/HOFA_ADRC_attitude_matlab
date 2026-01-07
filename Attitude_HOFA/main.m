clear;
% Parameters
fs = 200; % Control frequency in Hz
T = 10; % Total simulation time in seconds
t = 0:1/fs:T; % Time vector

dwd1 = square(2 * pi  * t);
wd1 = cumtrapz(t, dwd1);
Rd1 = cumtrapz(t, wd1);

dwd = [dwd1', 0*t', 0*t'];
wd = [wd1', 0*t', 0*t'];
Rd = eul2rotm([ 0*t', 0*t',Rd1']);
% Plotting the results
figure;
subplot(3, 1, 1);
plot(t, dwd);
title('Desired Angular Acceleration (First Axis)');
xlabel('Time (s)');
ylabel('Angular Acceleration (rad/s^2)');

subplot(3, 1, 2);
plot(t, wd);
title('Desired Angular Velocity (First Axis)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');

subplot(3, 1, 3);
plot(t, Rd1);
title('Desired Attitude (First Axis)');
xlabel('Time (s)');
ylabel('Attitude (rad)');

% Save the results to workspace
save('desired_trajectory.mat', 't', 'dwd', 'wd', 'Rd');
