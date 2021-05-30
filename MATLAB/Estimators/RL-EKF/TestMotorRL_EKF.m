%% Prepare input from simulated data
time = simout.Time;
PWM = simout.Data(:,1);
U = 9*PWM;
Current = simout.Data(:,2);

% Downsample by selecting instances where the PWM transitions
idx_up_before = find(diff(U) > 0);
idx_down_before = find(diff(U) < 0);

if (length(idx_up_before) > length(idx_down_before))
    idx_up_before = idx_up_before(1:end-1);
elseif (length(idx_up_before) < length(idx_down_before))
    idx_down_before = idx_down_before(1:end-1);
end

idx_up_after = idx_up_before + 1;
t_up_before = time(idx_up_before);
t_up_after = time(idx_up_after);

idx_down_after = idx_down_before + 1;
t_down_before = time(idx_down_before);
t_down_after = time(idx_down_after);

idx_downsampled = unique([idx_up_before, idx_up_after, idx_down_before, idx_down_after]); 
time_downsampled = time(idx_downsampled);
U_downsampled = U(idx_downsampled);
Current_downsampled = Current(idx_downsampled);

t_low = time(idx_up_before);
t_high = time(idx_down_before);
i_low = Current(idx_up_before);
i_high = Current(idx_down_before);

if (t_high(1) < t_low(1))
    t_high(1) = [];
    i_high(1) = [];
end

%% Add Gaussian noise to current measurements
Var_i_low = 0*0.2 ^ 2;
Var_i_high = 0*0.1 ^ 2;

i_low = i_low + sqrt(Var_i_low)*randn(size(i_low));
i_high = i_high + sqrt(Var_i_high)*randn(size(i_high));


%% Perform EKF
R0 = 1; % ohm
L0 = 1e-3; % Henry
X = [R0; L0];
P = 0.0001 * diag([1, 1e-3]);

Vbat = 9;

i_est_high = [];
i_est_low = 0;
R_est = [];
L_est = [];
for (i = 1:(min(length(t_low), length(t_high)) - 1))
    % Low to high transition
    tdelta_high = t_high(i) - t_low(i);    
    
    % High to low transition
    tdelta_low = t_low(i+1) - t_high(i);    
    
    % Perform EKF
    [X_out, P_out] = MotorRL_EKF(X, P, ...
                            i_low(i), Vbat, tdelta_high, tdelta_low, ...
                            i_high(i), i_low(i+1));                       
                        
    X = X_out;
    P = P_out;  
    
    R2 = X(1);
    L2 = X(2);
    
    i_est_high(i) = getCurrentUp(R2, L2, 9, i_low(i), tdelta_high);
    i_est_low(i+1) = getCurrentDown(R2, L2, i_high(i), tdelta_low);
    R_est(i) = R2;
    L_est(i) = L2;
end

figure(1);
ax1 = subplot(2,1,1);
plot(time, Current);
hold on;
plot(t_high(1:end-1), i_est_high, 'ro');
plot(t_low, i_est_low, 'go');
hold off;
title('Low-side Current');
ylabel('Current [A]');
xlabel('Time [s]');

ax2 = subplot(2,1,2);
plot(t_low(2:end), R_est, t_low(2:end), 1000*L_est);
linkaxes([ax1,ax2], 'x');

fprintf('\nR_est = %1.2f Ohm\n', R_est(end));
fprintf('L_est = %1.1f uH\n', 1e6*L_est(end));
fprintf('Resistance estimation error: %1.1f%%\n', abs(100* (R_est(end)-R) / R));
fprintf('Inductance estimation error: %1.1f%%\n', abs(100* (L_est(end)-L) / L));