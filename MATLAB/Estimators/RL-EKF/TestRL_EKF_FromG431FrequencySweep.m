f = fopen('C:\Users\Thomas\Dropbox\DC motor driver\Data and measurements\G431\data\frequency_sweep_end_sampled5.txt');
data = uint8(fread(f));
fclose(f);

G = 9.143;
Voffset = 2.057;
Rsense = 0.015; % 15 mOhm

Vin_average = 12; % volt
ADCsampleTimeOffset = 10e-6; % micro seconds

time_resolution = 1 / 100000; % 100 kHz timer
sweep_samples = 2000;
skip_samples = 0;
remove_end_samples = 1;

% Process data
% Create parsed struct
offset = 1;
[time_low, offset] = parseData(data, offset, 'uint32', sweep_samples);
[sample_low, offset] = parseData(data, offset, 'int16', sweep_samples);
[time_high, offset] = parseData(data, offset, 'uint32', sweep_samples);
[sample_high, offset] = parseData(data, offset, 'int16', sweep_samples);

% Change type
time_low = double(time_low(1+skip_samples:end-remove_end_samples)) * time_resolution;
sample_low = double(sample_low(1+skip_samples:end-remove_end_samples));
time_high = double(time_high(1+skip_samples:end-remove_end_samples)) * time_resolution;
sample_high = double(sample_high(1+skip_samples:end-remove_end_samples));

t0 = min([time_high; time_low]);
time_low = time_low - t0;
time_high = time_high - t0;

% Combine data
time = [time_high; time_low];
samples = [sample_high; sample_low];
dataRaw = [time, samples];
dataRaw = sortrows(dataRaw, 1);

clear data;
data.time = dataRaw(:,1); % skip the first 2 data points
data.raw = dataRaw(:,2);

% Process data
sample_index = (0:(length(time_low)-1))';
sample_index2(1:2:2*length(sample_index),1) = sample_index;
sample_index2(2:2:2*length(sample_index),1) = sample_index;

data.sense_voltage = (data.raw / 4095) * 3.3;
data.current = (data.sense_voltage - Voffset) / (G * Rsense);

%% Construct switching times
PWM_freq_ = 500 + 5000/sweep_samples * (skip_samples:(sweep_samples-remove_end_samples-1))';

PWM_freq(1:2:(2*length(PWM_freq_))) = PWM_freq_;
PWM_freq(2:2:(2*length(PWM_freq_))) = PWM_freq_;

time_high2low = time_high + ADCsampleTimeOffset;
time_low2high = time_low + ADCsampleTimeOffset;

PWM_time = [time_low2high; time_high2low];
PWM_time = sortrows(PWM_time, 1);

PWM = zeros(size(time));
PWM(1:2:(end-1)) = 1;

Vin = ones(length(PWM),1) * Vin_average;

%%
figure(1);
ax1 = subplot(2,1,1);
stairs(PWM_time, PWM);
ylim([-0.1, 1.1]);
ax2 = subplot(2,1,2);
plot(data.time, data.sense_voltage);
linkaxes([ax1, ax2], 'x');

figure(2);
stairs(PWM_time, PWM);
hold on;
plot(PWM_time, PWM, '*');
plot(time_low, zeros(length(time_low)), 'bx');
plot(time_high, ones(length(time_high)), 'rx');
hold off;
ylim([-0.1, 1.1]);


%% Compute and visualize PWM frequency sweep
PWM_freq_computed = 1./(PWM_time(3:2:end) - PWM_time(1:2:end-2));
figure(10);
plot(0.5*(PWM_time(3:2:end)+PWM_time(1:2:end-2)), PWM_freq_computed);
hold on;
plot(PWM_time(1:2:end), PWM_freq_);
hold off;

%% Perform EKF
X = [0; 1; 1e-3];
P = diag([1, 0.1, 0.001]);

for (s = 1:3)
i_avg_est = [];
i_min_est = [];
R_est = [];
L_est = [];

for (i = 1:(length(time_high)-2))
    t_PWM_period_start = PWM_time(2*i+1);
    t_sample_high = time_high(i+1);
    t_PWM_period_duty = PWM_time(2*i+2);
    t_sample_low = time_low(i+2); % add one due to synchronization
    t_PWM_period_end = PWM_time(2*i+3);

    i_sample1 = data.current(2*i+2);
    i_sample2 = data.current(2*i+3);

    dt = (t_PWM_period_end - t_PWM_period_start);
    f = 1/dt;

    duty = (t_PWM_period_duty - t_PWM_period_start) / dt;
    location_sample1 = (t_sample_high - t_PWM_period_start) / dt;
    location_sample2 = (t_sample_low - t_PWM_period_start) / dt;
        
    [X, P, i_avg] = RL_EKF2(X, P, ...
        Ke, ... % constants/parameters
        f, duty, Vin_average, 0, ... % inputs (u)
        i_sample1, location_sample1, ...
        i_sample2, location_sample2);  % measurements (z)

    i_min_est(end+1) = X(1);
    i_avg_est(end+1) = i_avg;
    R_est(end+1) = X(2);
    L_est(end+1) = X(3);        
end

fprintf('\nR_est = %1.2f Ohm [%1.2f Ohm]\n', R_est(end), mean(R_est));
fprintf('L_est = %1.1f mH [%1.1f mH]\n', 1e3*L_est(end), 1e3*mean(L_est));

figure(5);
ax1 = subplot(2,1,1);
plot(data.time, data.current);
hold on;
plot(time_low(2:end-1), i_min_est, 'ro');
plot(0.5*(time_high(1:end-2)+time_low(2:end-1)), i_avg_est, 'go');
hold off;
title('Low-side Current');
ylabel('Current [A]');
xlabel('Time [s]');
legend('Measured current', 'Min estimated current');
xlim([0, time(end)]);

ax2 = subplot(2,1,2);
plot(time_high(1:end-2), R_est, time_high(1:end-2), 1000*L_est);
linkaxes([ax1,ax2], 'x');
ylabel('Estimated parameter');
xlabel('Time [s]');
legend('R_{est} [Ohm]', 'L_{est} [mH]');
xlim([0, time(end)]);
pause;
end