data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '');

figure(1);
plot(data.time_speed, data.speed);

%%
% Extract section which includes free spinning/deceleration
t0 = 1.13;
t1 = 2.719;
[~, idx0] = min(abs(data.time - t0));
[~, idx1] = min(abs(data.time - t1));
time_out = data.time(idx0:idx1);

% Compute speed from encoder ticks
step = 1; % discretization step size (number of samples - 1)
time0 = data.time(1:step:end-step);
time1 = data.time(1+step:step:end);
encoder0 = data.encoder(1:step:end-step);
encoder1 = data.encoder(1+step:step:end);
vel{step}.time = (time0 + time1) / 2;
vel{step}.speed = 2*pi/TicksPrRev * (encoder1 - encoder0) ./ (time1 - time0);

% Fit speed to exponential curve
speed_fitted = VelocityFitFromEncoder(vel{step}.time, vel{step}.speed, t0, t1, time_out);
figure(1);
plot(vel{step}.time, vel{step}.speed);
hold on;
plot(time_out, speed_fitted);
hold off;