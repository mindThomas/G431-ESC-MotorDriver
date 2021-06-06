scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../DataProcessing/functions'));
addpath(fullfile(scriptDir, '../../DataProcessing/CSV'));
addpath(fullfile(scriptDir, '../../DataProcessing/CSV/functions'));

data = LoadRawDump('/home/thomas/Dropbox/Private share/DC motor driver/Data and measurements/G431/csv/', '2020-05-16_15-05-47-797.csv');
%data = LoadRawDump('/home/thomas/Private/G431-ESC-MotorDriver/Python', '2021-05-31_01-03-29-278_raw.csv');
%data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '', false);
idx = find(data.DutyCycle > 0 & data.TimerFrequency >= 500 & data.TimerFrequency <= 5000);
data = Extract(data, idx(1), idx(end));
%data.RPM = mean(data.RPM) * ones(size(data.RPM));
%data.RPM = movmean(data.RPM, 50);
data.omega = 2*pi/60 * data.RPM;

figure(100);
plot(data.TimerFrequency, data.CurrentON, '.');
hold on;
plot(data.TimerFrequency, data.CurrentOFF, '.');
hold off;
legend('Current ON', 'Current OFF');

figure(101);
plot(data.time, data.CurrentON, '.');
hold on;
plot(data.time, data.CurrentOFF, '.');
hold off;
legend('Current ON', 'Current OFF');


figure(102);
ax1 = subplot(3,1,1);
plot(data.time, data.TimerFrequency);
ylabel('Frequency [Hz]');
ax2 = subplot(3,1,2);
plot(data.time, data.DutyCycle);
ylabel('Duty Cycle [%]');
ax3 = subplot(3,1,3);
plot(data.time, data.RPM);
ylabel('Speed [RPM]');
xlabel('Time [s]');
linkaxes([ax1,ax2,ax3], 'x');

%%
[imax, idx] = max(data.CurrentON);

%R_guess = data.VinON(idx) / data.CurrentON(idx) * data.DutyCycle(idx)
R_guess = data.VinON(end) / data.CurrentON(end) * data.DutyCycle(end)

dt = 1./data.TimerFrequency;

t_OFF = (data.TriggerOFF - data.DutyCycle) .* dt;
%L_guess = real(-R_guess * t_OFF(idx) / log( data.CurrentOFF(idx) / data.CurrentON(idx) ))
L_guess = real(-R_guess * t_OFF(end) / log( data.CurrentOFF(end) / data.CurrentON(end) ))
Ke_guess = 0.3;

%%
%R_guess = 3
%L_guess = 0.002;

%%
i_ON = @(t_SAMPLE_OFF_previous,t_OFF_previous,i_OFF_previous, t_SAMPLE, t_ON,omega,vin,R,L,Ke)  ( (i_OFF_previous + Ke*omega/R).*exp(-R/L*(t_OFF_previous - t_SAMPLE_OFF_previous)) - vin/R).*exp(-R/L*t_SAMPLE) + (vin - Ke*omega)/R;
i_OFF = @(t_SAMPLE_ON_previous,t_ON_previous,i_ON_previous,  t_SAMPLE, t_OFF,omega,vin,R,L,Ke)  ( (i_ON_previous - (vin - Ke*omega)/R).*exp(-R/L*(t_ON_previous - t_SAMPLE_ON_previous)) + vin/R).*exp(-R/L*t_SAMPLE) - Ke*omega/R

i_ON_ = @(R,L,Ke, vin, duty, s_sample_off_previous, i_off_previous, s_sample, freq, omega) i_ON((s_sample_off_previous-duty)./freq, (1-duty)./freq, i_off_previous, s_sample./freq, duty./freq, omega, vin, R, L, Ke);
i_OFF_ = @(R,L,Ke, vin, duty, s_sample_on_previous, i_on_previous,  s_sample, freq, omega) i_OFF(s_sample_on_previous./freq, duty./freq, i_on_previous, (s_sample-duty)./freq, (1-duty)./freq, omega, vin, R, L, Ke);

%% Fit based on combined ON+OFF current
modelfun = @(b,x) (x(:, 8) .* i_ON_(b(1), b(2), b(3), x(:, 2), x(:, 3), x(:, 4), x(:, 5), x(:, 6), x(:, 1), x(:, 7)) + ...
                   (1-x(:, 8)) .* i_OFF_(b(1), b(2), b(3), x(:, 2), x(:, 3), x(:, 4), x(:, 5), x(:, 6), x(:, 1), x(:, 7)));
beta0 = [R_guess, L_guess, Ke_guess];
PreviousTriggerOFF = [data.TriggerOFF(1); data.TriggerOFF(1:end-1)];
PreviousI_OFF = [data.CurrentOFF(1); data.CurrentOFF(1:end-1)];
X = [[data.TimerFrequency, data.VinON, data.DutyCycle, PreviousTriggerOFF, PreviousI_OFF, data.TriggerON, data.omega, ones(size(data.VinON))];
     [data.TimerFrequency, data.VinOFF, data.DutyCycle, data.TriggerON, data.CurrentON, data.TriggerOFF, data.omega, zeros(size(data.VinOFF))]];
Y = [data.CurrentON;
     data.CurrentOFF];
mdl = fitnlm(X, Y, modelfun, beta0);

R_fit = mdl.Coefficients.Estimate(1)
L_fit = mdl.Coefficients.Estimate(2)
Ke_fit = mdl.Coefficients.Estimate(3)

%%
% figure(100);
% plot(data.TimerFrequency, data.CurrentON, '.');
% hold on;
% plot(data.TimerFrequency, data.CurrentOFF, '.');
% hold off;
% legend('Current ON', 'Current OFF');
% 
% hold on;
% plot(data.TimerFrequency, i_ON_(R_guess, L_guess, Ke_guess, data.VinON, data.DutyCycle, data.TriggerOFF, data.TimerFrequency, data.omega));
% plot(data.TimerFrequency, i_ON_(R_fit, L_fit, Ke_fit, data.VinON, data.DutyCycle, data.TriggerOFF, data.TimerFrequency, data.omega));
% plot(data.TimerFrequency, i_OFF_(R_guess, L_guess, Ke_guess, data.VinOFF, data.DutyCycle, data.TriggerOFF, data.TimerFrequency, data.omega));
% plot(data.TimerFrequency, i_OFF_(R_fit, L_fit, Ke_fit, data.VinOFF, data.DutyCycle, data.TriggerOFF, data.TimerFrequency, data.omega));
% hold off;

%%
figure(101);
plot(data.time, data.CurrentON, '.');
hold on;
plot(data.time, data.CurrentOFF, '.');

%plot(data.time, i_ON_(R_guess, L_guess, Ke_guess, data.VinON, data.DutyCycle, data.TriggerON, data.TimerFrequency, data.omega));
plot(data.time, i_ON_(R_fit, L_fit, Ke_fit, data.VinON, data.DutyCycle, PreviousTriggerOFF, PreviousI_OFF, data.TriggerON, data.TimerFrequency, data.omega));
%plot(data.time, i_OFF_(R_guess, L_guess, Ke_guess, data.VinOFF, data.DutyCycle, data.TriggerOFF, data.TimerFrequency, data.omega));
plot(data.time, i_OFF_(R_fit, L_fit, Ke_fit, data.VinOFF, data.DutyCycle, data.TriggerON, data.CurrentON, data.TriggerOFF, data.TimerFrequency, data.omega));
hold off;

legend('Current ON', 'Current OFF', 'Fit ON', 'Fit OFF');
xlabel('Time [s]');
ylabel('Current [A]');