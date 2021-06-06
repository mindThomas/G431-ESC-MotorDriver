%% Load measured data
data = csvread('C:\Users\Thomas\Dropbox\DC motor driver project\Data and measurements\FrequencySweep5.csv', 1, 0);

%%
f = fopen('C:\Users\Thomas\Dropbox\DC motor driver project\Data and measurements\FrequencySweep5.csv');
fgetl(f)
fclose(f);

%%
% Data format:
% ' Time [s],VNH CS-Analog,INA180 CS-Analog,Vin-Analog, Time [s],PWM-Digital'
endIdx = find(data(:,5) == 0);
endIdx = endIdx(2)-1;
time = data(1:endIdx,5);
PWM = data(1:endIdx,6);

% Loop through PWM events and get Vin voltage
Vin = zeros(size(time));
oldIdx = 1;
for (i = 1:length(time))
    t = time(i);
    I = oldIdx;
    while (data(I,1) < t)
        I = I + 1;
    end
    oldIdx = I;
    Vin(i) = data(I,4);
end

U = [Vin(2:end); Vin(end-1)] .* PWM;

figure(1);
stairs(time, U);
hold on;
stairs(time, Vin);
hold off;

%% Visualize current
time_highres = data(:,1);
VNH_CS = data(:,2) - data(1,2);
INA180_CS = data(:,3) - data(1,3);

iINA180 = INA180_CS / 0.4;
correction = 1.16;
iVNH = VNH_CS * 1.98529 * correction;

% % Filter out switching spikes
% oldIdx = 2;
% removalTimeBefore = 50e-6; % 2 us
% removalTimeAfter = 50e-6; % 2 us
% for (i = 1:length(PWM))    
%     %if (PWM(i) == 1)
%         t = time(i);
%         I = oldIdx;
%         while (time_highres(I,1) < (t-removalTimeBefore))
%             I = I + 1;
%         end
%         iTmp1 = iVNH(I-1);
%         iTmp2 = iINA180(I-1);                
%         while (time_highres(I,1) < (t+removalTimeAfter))
%             iVNH(I) = iTmp1;
%             iINA180(I) = iTmp2;
%             I = I + 1;
%         end               
%         oldIdx = I;
%     %end
% end

figure(2);
plot(time_highres, iVNH, time_highres, iINA180);
xlabel('Time [s]');
ylabel('Current [A]');

%% Sample current switching times
toffset_high = 20e-6;
toffset_low = 10e-6;
% Loop through PWM events and get Current voltage
Current = zeros(size(PWM));
oldIdx = 1;
time_corrected = time;
for (i = 1:length(PWM))
    if (PWM(i) == 0) % PWM  from high to low, take current reading at peak
        t = time(i) + toffset_high;
    else % from low to high, offset the reading by toffset_low
        t = time(i) + toffset_low;
    end
    time_corrected(i) = t;
    I = oldIdx;
    while (time_highres(I,1) < t)
        I = I + 1;
    end
    oldIdx = I;
    Current(i) = iVNH(I);
end

figure(3);
stairs(time, Current);
hold on;
plot(time, Current, '*');
plot(time, PWM, '*');
stairs(time, PWM);
hold off;

figure(2);
hold on;
plot(time_corrected, Current, '*');
plot(time, U, '*');
stairs(time, Vin, 'o');
hold off;
legend('iVNH','iINA180','Current samples','Vmot samples','Vin samples');

%% Compute resistance
R = [];
for (i = 1:length(PWM))
    if (PWM(i) == 1)
        R(end+1) = U(i) / Current(i);
    end
end

figure(4);
plot(R);

%% Perform EKF
R0 = 1; % ohm
L0 = 1e-3; % Henry
X = [R0; L0];

idx_start = find(PWM == 1); % we need to start at the instance where PWM goes high
idx_start = idx_start(1);

portion_to_include = 1.0;

P = 1 * diag([1e-2, 1e-2]);

%%
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

fprintf('\nR_est = %1.2f Ohm\n', R_est(end));
fprintf('L_est = %1.1f mH\n', 1e3*L_est(end));

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