data = LoadDump('C:\Users\Thomas\Documents\MotorDriver-MATLAB\Processing\Data\', 'DeceleratingBackEMF.csv');

%% Calibrate Bemf with Voltage divider on
% Extract section where Bemf without voltage divider is not saturated
t0 = 1.95;
t1 = 2.68;
[~, idx0] = min(abs(data.time - t0));
[~, idx1] = min(abs(data.time - t1));
time_out = data.time(idx0:idx1);

% Calibrate Bemf with Voltage divider on
v_bemf1 = data.bemf(1:2:end-1);
t_bemf1 = data.time(1:2:end-1);
v_bemf2 = data.bemf(2:2:end);
t_bemf2 = data.time(2:2:end);

if (max(v_bemf1) > max(v_bemf2))
    v_bemf_raw = v_bemf1;
    t_bemf_raw = t_bemf1;
    v_bemf_divided = v_bemf2;
    t_bemf_divided = t_bemf2;
else
    v_bemf_raw = v_bemf2;
    t_bemf_raw = t_bemf2;
    v_bemf_divided = v_bemf1;
    t_bemf_divided = t_bemf1;
end

figure(1);
subplot(2,1,1);
plot(t_bemf_raw, v_bemf_raw);
hold on;
plot(t_bemf_divided, v_bemf_divided);
hold off;
legend('Raw', 'Voltage divided');

bemf_calibration = Line();
bemf_calibration = bemf_calibration.fit(v_bemf_divided(idx0/2:idx1/2), v_bemf_raw(idx0/2:idx1/2))

subplot(2,1,2);
plot(v_bemf_raw(floor(idx0/2):floor(idx1/2)));
hold on;
plot(bemf_calibration.a * v_bemf_divided(idx0/2:idx1/2) + bemf_calibration.b);
hold off;
title('Compensated');
legend('Raw', 'Voltage divided');

%% Extract section which includes free spinning/deceleration
t0 = 1.13;
t1 = 2.6;
[~, idx0] = min(abs(data.time - t0));
[~, idx1] = min(abs(data.time - t1));
time_out = t_bemf_divided(floor(idx0/2):floor(idx1/2));

%% Compute speed from encoder ticks
step = 1; % discretization step size (number of samples - 1)
time0 = data.time(1:step:end-step);
time1 = data.time(1+step:step:end);
encoder0 = data.encoder(1:step:end-step);
encoder1 = data.encoder(1+step:step:end);
vel{step}.time = (time0 + time1) / 2;
vel{step}.speed = 2*pi/TicksPrRev * (encoder1 - encoder0) ./ (time1 - time0);

%% Fit speed to exponential curve
speed_fitted = VelocityFitFromEncoder(vel{step}.time, vel{step}.speed, t0, t1, time_out);
figure(2);
plot(vel{step}.time, vel{step}.speed);
hold on;
plot(time_out, speed_fitted);
hold off;

%% Estimate the Ke parameter
v_bemf = bemf_calibration.a * v_bemf_divided(floor(idx0/2):floor(idx1/2)) + bemf_calibration.b;

figure(7);
plot(speed_fitted, v_bemf, '*');

l_ke = Line(true);
l_ke = l_ke.fit(speed_fitted, v_bemf);
l_ke.plot(min(speed_fitted), max(speed_fitted), 'r');

Ke = l_ke.a