time = simout.Time;
PWM = simout.Data(:,1);
U = 9*PWM;
Current = simout.Data(:,2);

% fs = 10000; % 10 kHz
% ts = 1/fs;
% time2 = 0:ts:time(end);
% 
% U = interp1(time, U, time2);
% Current = interp1(time, Current, time2);
% time = time2;

figure(1);
ax1 = subplot(2,1,1);
plot(time, U);
title('Vmot Input (PWM with VBAT) [V]');
ylabel('Voltage [V]');
ax2 = subplot(2,1,2);
plot(time, Current);
title('Low-side Current');
ylabel('Current [A]');
xlabel('Time [s]');
linkaxes([ax1,ax2], 'x');

%% Downsample by selecting instances where the PWM transitions
idx_up_before = find(diff(U) > 0);
idx_up_after = idx_up_before + 1;
t_up_before = time(idx_up_before);
t_up_after = time(idx_up_after);

idx_down_before = find(diff(U) < 0);
idx_down_after = idx_down_before + 1;
t_down_before = time(idx_down_before);
t_down_after = time(idx_down_after);

idx_downsampled = unique([idx_up_before, idx_up_after, idx_down_before, idx_down_after]); 
time_downsampled = time(idx_downsampled);
U_downsampled = U(idx_downsampled);
Current_downsampled = Current(idx_downsampled);

figure(1);
ax1 = subplot(2,1,1);
plot(time, U);
hold on;
plot(time_downsampled, U_downsampled, 'o');
hold off;
title('Vmot Input (PWM with VBAT) [V]');
ylabel('Voltage [V]');
ax2 = subplot(2,1,2);
plot(time, Current);
hold on;
plot(time_downsampled, Current_downsampled, 'o');
hold off;
title('Low-side Current');
ylabel('Current [A]');
xlabel('Time [s]');
linkaxes([ax1,ax2], 'x');


%%
%data = iddata(Current', U', ts);
data = iddata(Current_downsampled, U_downsampled, [], 'SamplingInstants', time_downsampled);
%sys = ssest(data, 1)
%sys = tfest(data, 1, 0)

%
% Transfer function estimation     
Options = tfestOptions;           
Options.Display = 'on';           
Options.WeightingFilter = [];     
                                   
tf1 = tfest(data, 1, 0, Options)   

L2 = 1/tf1.Numerator;
R2 = L2*tf1.Denominator(2);


fprintf('\nResistance estimation error: %1.1f%%\n', abs(100* (R2-R) / R));
fprintf('Inductance estimation error: %1.1f%%\n', abs(100* (L2-L) / L));


%%
t_low = time(idx_up_before);
t_high = time(idx_down_before);
i_low = Current(idx_up_before);
i_high = Current(idx_down_before);

if (t_high(1) < t_low(1))
    t_high(1) = [];
    i_high(1) = [];
end

figure(1);
ax1 = subplot(2,1,1);
plot(time, U);
title('Vmot Input (PWM with VBAT) [V]');
ylabel('Voltage [V]');
ax2 = subplot(2,1,2);
plot(time, Current);
hold on;
plot(t_high(1:2), i_high(1:2), 'ro');
plot(t_low(1:2), i_low(1:2), 'go');
hold off;
title('Low-side Current');
ylabel('Current [A]');
xlabel('Time [s]');
linkaxes([ax1,ax2], 'x');

%%
i_low = 0;
i_high = [];
for (i = 1:(min(length(t_low), length(t_high)) - 1))
    % Low to high transition
    tdelta = t_high(i) - t_low(i);
    i_high(i) = getCurrentUp(R2, L2, 9, i_low(i), tdelta);
    
    % High to low transition
    tdelta = t_low(i+1) - t_high(i);
    i_low(i+1) = getCurrentDown(R2, L2, i_high(i), tdelta);
end

figure(1);
ax1 = subplot(2,1,1);
plot(time, U);
title('Vmot Input (PWM with VBAT) [V]');
ylabel('Voltage [V]');
ax2 = subplot(2,1,2);
plot(time, Current);
hold on;
plot(t_high(1:end-1), i_high, 'ro');
plot(t_low, i_low, 'go');
hold off;
title('Low-side Current');
ylabel('Current [A]');
xlabel('Time [s]');
linkaxes([ax1,ax2], 'x');