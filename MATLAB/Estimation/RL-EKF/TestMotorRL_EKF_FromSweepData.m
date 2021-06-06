f = fopen('C:\Users\Thomas\Dropbox\DC motor driver\Data and measurements\Stalled test\vnh7070_dump6.bin');
data = uint8(fread(f));
fclose(f);

%% Create parsed struct
offset = 1;
[time_low, offset] = parseData(data, offset, 'uint32', 400);
[sample_low, offset] = parseData(data, offset, 'int16', 400);
[time_high, offset] = parseData(data, offset, 'uint32', 400);
[sample_high, offset] = parseData(data, offset, 'int16', 400);

%% Change type
time_low = double(time_low);
sample_low = double(sample_low);
time_high = double(time_high);
sample_high = double(sample_high);

%% Process data
time_resolution = 1 / 100000;

time_offset = min([time_high; time_low]);
time = [time_high-time_offset; time_low-time_offset] * time_resolution;
samples = [sample_high; sample_low];
data = [time, samples];
data = sortrows(data, 1);

time = data(:,1);
VNH_CS = (data(:,2) / 4095) * 3.3;

%% Visualize current
%INA180_CS = data(:,3) - data(1,3);
%iINA180 = INA180_CS / 0.4;
correction = 1.16;
iVNH = VNH_CS * 1.98529 * correction;

%iINA180 = iVNH
%INA180_CS / 0.4 = VNH_CS * 1.98529 * correction
%INA180_CS = 0.4 * VNH_CS * 1.98529 * correction


figure(1);
plot(time, iVNH);
xlabel('Time [s]');
ylabel('Current [A]');

%% Construct switching times
toffset_high = 20e-6;
toffset_low = 10e-6;
Current = iVNH;

time_corrected = time;
PWM = zeros(size(time));
% First sample is when PWM goes from 0 to 1 and thus where the 'low'
% current sample is taken
PWM(1:2:end) = 1;
time(1:2:end) = time(1:2:end) - toffset_low; 
time(2:2:end) = time(2:2:end) - toffset_high;

Vin = 8.33 * ones(size(time)); % Volt
U = Vin .* PWM;

%%
figure(2);
stairs(time, PWM);
hold on;
plot(time, PWM, '*');
plot(time_corrected, Current, '*');
hold off;

figure(1);
hold on;
plot(time_corrected, Current, '*');
plot(time, U, '*');
stairs(time, Vin, 'o');
hold off;
legend('iVNH','iINA180','Current samples','Vmot samples','Vin samples');

%% Perform EKF
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

indices = idx_start:2:(portion_to_include*length(U)-3);
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
fprintf('L_est = %1.1f mH [%1.1f mH]\n', 1e3*mean(L_est), 1e3*L_est(end));

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