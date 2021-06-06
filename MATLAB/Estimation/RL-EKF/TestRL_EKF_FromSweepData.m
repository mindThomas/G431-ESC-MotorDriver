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


%%
figure(1);
ax1 = subplot(2,1,1);
stairs(time, PWM);
ylim([-0.1, 1.1]);
ax2 = subplot(2,1,2);
plot(time_corrected, Current);
linkaxes([ax1, ax2], 'x');

figure(2);
stairs(time, PWM);
hold on;
plot(time, PWM, '*');
plot(time_corrected(1:2:end), zeros(size(time_corrected(1:2:end))), 'bx');
plot(time_corrected(2:2:end), ones(size(time_corrected(2:2:end))), 'rx');
hold off;
ylim([-0.1, 1.1]);


%% Compute and visualize PWM frequency sweep
PWM_freq_computed = 1./(time(3:2:end) - time(1:2:end-2));
figure(10);
plot(0.5*(time(3:2:end)+time(1:2:end-2)), PWM_freq_computed);

%% Perform EKF
X = [0; 1; 1e-3];
P = diag([1, 0.1, 0.001]);

for (s = 1:3)
i_avg_est = [];
i_min_est = [];
R_est = [];
L_est = [];

for (i = 1:(length(time)/2-2))
    t_PWM_period_start = time(2*i+1);
    t_sample_high = time_corrected(2*i+1);
    t_PWM_period_duty = time(2*i+2);
    t_sample_low = time_corrected(2*i+2);
    t_PWM_period_end = time(2*i+3);

    i_sample1 = Current(2*i+1);
    i_sample2 = Current(2*i+2);

    dt = (t_PWM_period_end - t_PWM_period_start);
    f = 1/dt;

    duty = (t_PWM_period_duty - t_PWM_period_start) / dt;
    location_sample1 = (t_sample_high - t_PWM_period_start) / dt;
    location_sample2 = (t_sample_low - t_PWM_period_start) / dt;
        
    [X, P, i_avg] = RL_EKF2(X, P, ...
        Ke, ... % constants/parameters
        f, duty, Vin(2*i+1), 0, ... % inputs (u)
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
plot(time_corrected, Current);
hold on;
plot(time_corrected(1:2:end-4), i_min_est, 'ro');
plot(0.5*(time_corrected(1:2:end-4)+time_corrected(2:2:end-4)), i_avg_est, 'go');
hold off;
title('Low-side Current');
ylabel('Current [A]');
xlabel('Time [s]');
legend('Measured current', 'Min estimated current');
xlim([0, time(end)]);

ax2 = subplot(2,1,2);
plot(time(2:2:end-4), R_est, time(2:2:end-4), 1000*L_est);
linkaxes([ax1,ax2], 'x');
ylabel('Estimated parameter');
xlabel('Time [s]');
legend('R_{est} [Ohm]', 'L_{est} [mH]');
xlim([0, time(end)]);
end