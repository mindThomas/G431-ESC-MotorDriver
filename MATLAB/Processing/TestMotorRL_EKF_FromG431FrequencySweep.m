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

Vin = ones(length(PWM)) * Vin_average;

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

%% Perform EKF
time = PWM_time;
Current = data.current;
toffset_low = -ADCsampleTimeOffset;
toffset_high = -ADCsampleTimeOffset;

R0 = 1; % ohm
L0 = 1e-3; % Henry
X = [R0; L0];

idx_start = find(PWM == 1); % we need to start at the instance where PWM goes high
idx_start = idx_start(1);

portion_to_include = 1.0;

P = 0.1 * diag([1e-2, 1e-4]);

%%
for (i = 1:100)
i_est_high = [];
i_peak = [];
i_est_low = 0;
R_est = [];
L_est = [];

indices = idx_start:2:(portion_to_include*length(time)-3);
%indices = [indices(end:-1:1), indices(end:-1:1)];
%indices = indices(end:-1:1);
for (i = indices)
    t_low2high = time(i);
    t_highend = time(i+1);
    t_high2low = time(i+1);
    t_lowend = time(i+2);
    
    Vin_high = Vin(i+1);
    Vin_low = Vin(i+2);  
    
    i0 = Current(i);
    i_highend = Current(i+1);
    i_lowend = Current(i+2);
    
    % Low to high transition
    tdelta_high = t_highend - t_low2high;
    
    % High to low transition
    tdelta_low = t_lowend - t_high2low;    

    % Perform EKF
    [X_out, P_out] = MotorRL_EKF_WithOffsetSampling(X, P, ...
                            i0, Vin_high, tdelta_high, tdelta_low, toffset_low, toffset_high, ...
                            i_highend, i_lowend);                                              
    
    X = X_out;
    P = P_out;  
    
    R2 = X(1);
    L2 = X(2);    
    
    i_peak(end+1) = getCurrentUp(R2, L2, Vin_high, i0, tdelta_high - toffset_low);
    i_est_high(end+1) = getCurrentDown(R2, L2, i_peak(end), toffset_high);
    i_low = getCurrentDown(R2, L2, i_highend, tdelta_low);    
    i_est_low(end+1) = getCurrentUp(R2, L2, Vin_high, i_low, toffset_low);
    R_est(end+1) = R2;
    L_est(end+1) = L2;
end

fprintf('\nR_est = %1.2f Ohm [%1.2f Ohm]\n', R_est(end), mean(R_est));
fprintf('L_est = %1.1f mH [%1.1f mH]\n', 1e3*L_est(end), 1e3*mean(L_est));

X = [mean(R_est); L_est(end)];
end

%i_est_high = i_est_high(end:-1:1);
%i_est_low = i_est_low(end:-1:1);
%R_est = R_est(end:-1:1);
%L_est = L_est(end:-1:1);

figure(5);
ax1 = subplot(2,1,1);
plot(time(idx_start:end), Current(idx_start:end));
hold on;
plot(time(idx_start:2:(portion_to_include*end-1)), i_est_low, 'ro');
plot(time(idx_start+1:2:(portion_to_include*end-2)), i_est_high, 'go');
hold off;
title('Low-side Current');
ylabel('Current [A]');
xlabel('Time [s]');
legend('Measured current', 'OFF-period estimated current', 'ON-period estimated current');
xlim([0, time(end)]);

ax2 = subplot(2,1,2);
plot(time(idx_start+1:2:(portion_to_include*end-2)), R_est, time(idx_start+1:2:(portion_to_include*end-2)), 1000*L_est);
linkaxes([ax1,ax2], 'x');
ylabel('Estimated parameter');
xlabel('Time [s]');
legend('R_{est} [Ohm]', 'L_{est} [mH]');
xlim([0, time(end)]);

return;

%% Plot estimated ON and OFF periods using estimated R & L
i_est_high = [];
i_est_low = 0;
for (i = indices)
    t_low2high = time(i);
    t_highend = time(i+1);
    t_high2low = time(i+1);
    t_lowend = time(i+2);
    
    Vin_high = Vin(i+1);
    Vin_low = Vin(i+2);  
    
    i0 = Current(i);
    i_highend = Current(i+1);
    i_lowend = Current(i+2);
    
    % Low to high transition
    tdelta_high = t_highend - t_low2high;
    
    % High to low transition
    tdelta_low = t_lowend - t_high2low;     
    
    %i_est_high(end+1) = getCurrentUp(R2, L2, Vin_high, i0, tdelta_high);
    %i_est_low(end+1) = getCurrentDown(R2, L2, i_highend, tdelta_low);    
    
    i_peak = getCurrentUp(R2, L2, Vin_high, i0, tdelta_high - toffset_low);
    i_est_high(end+1) = getCurrentDown(R2, L2, i_peak, toffset_high);
    i_low = getCurrentDown(R2, L2, i_highend, tdelta_low);    
    i_est_low(end+1) = getCurrentUp(R2, L2, Vin_high, i_low, toffset_low);
end

figure(5);
subplot(2,1,1);
hold on;
plot(time(idx_start:2:(portion_to_include*end-1)), i_est_low, 'ko');
plot(time(idx_start+1:2:(portion_to_include*end-2)), i_est_high, 'ko');
hold off;