f = fopen('C:\Users\Thomas\Desktop\duty_sweep.txt');
data = uint8(fread(f));
fclose(f);

G = 9.143;
Voffset = 2.057;
Rsense = 0.015; % 15 mOhm

% Create parsed struct
offset = 1;
[time_low, offset] = parseData(data, offset, 'uint32', 1000);
[sample_low, offset] = parseData(data, offset, 'int16', 1000);
[time_high, offset] = parseData(data, offset, 'uint32', 1000);
[sample_high, offset] = parseData(data, offset, 'int16', 1000);
[time_vin, offset] = parseData(data, offset, 'uint32', 2000);
[sample_vin, offset] = parseData(data, offset, 'int16', 2000);

% Process data
time_resolution = 1 / 100000; % 100 kHz timer

skip_samples = 2;

% Change type
time_low = double(time_low(1+skip_samples:end)) * time_resolution;
sample_low = double(sample_low(1+skip_samples:end));
time_high = double(time_high(1+skip_samples:end)) * time_resolution;
sample_high = double(sample_high(1+skip_samples:end));

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

% Extract Vin samples
time_vin = double(time_vin(1+2*skip_samples:end)) * time_resolution - t0;
sample_vin = double(sample_vin(1+2*skip_samples:end));

%% Process data
Vin = (sample_vin / 4095) * 3.3 * (169+18) / (18) * 0.982;

duty_cycle = 0.1 + 0.8 * time_vin/0.5;
Vmot = duty_cycle .* Vin;

data.sense_voltage = (data.raw / 4095) * 3.3;
data.current = (data.sense_voltage - Voffset) / (G * Rsense);

%% Construct switching times
dt = time_low(2:end) - time_low(1:end-1);
dt2 = time_high(2:end) - time_high(1:end-1);

PWM_freq = 2 ./ (dt + dt2);
PWM_freq(end+1) = PWM_freq(end);

time_high2low = time_low - ((1-duty_cycle(1:2:end))*1./PWM_freq)/2;
time_low2high = time_low + ((1-duty_cycle(1:2:end))*1./PWM_freq)/2;

PWM_time = [time_low2high; time_high2low];
PWM_time = sortrows(PWM_time, 1);

PWM = zeros(size(time));
PWM(2:2:end) = 1;

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
R0 = 1; % ohm
L0 = 1e-3; % Henry
X = [R0; L0];

idx_start = find(PWM == 1); % we need to start at the instance where PWM goes high
idx_start = idx_start(1);

portion_to_include = 1.0;

P = 0.1 * diag([1e-2, 1e-4]);

%
time = PWM_time;
sample_time = data.time;
Current = data.current;
toffset_low = 0;
toffset_high = 0;

%for (i = 1:100)
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
    
    Vin_high = Vmot(i+1);
    Vin_low = Vmot(i+2);  
    
    i0 = Current(i);
    i_highend = Current(i+1);
    i_lowend = Current(i+2);
       
    % Low to high transition
    tdelta_high = t_highend - t_low2high;
    
    % High to low transition
    tdelta_low = t_lowend - t_high2low;        
    
    % Compute sample time offsets    
    tsample_high = sample_time(i);
    tsample_low = sample_time(i+1);    
    toffset_low = tsample_low - t_low2high        
    toffset_high = tsample_low - t_high2low
    
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
fprintf('L_est = %1.1f mH [%1.1f mH]\n', 1e3*mean(L_est), 1e3*L_est(end));

X = [mean(R_est); L_est(end)];
%end

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