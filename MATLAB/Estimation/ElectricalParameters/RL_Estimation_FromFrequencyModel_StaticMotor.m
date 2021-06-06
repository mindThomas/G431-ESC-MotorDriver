scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../DataProcessing/functions'));
addpath(fullfile(scriptDir, '../../DataProcessing/CSV'));

data = LoadDump('/home/thomas/Private/G431-ESC-MotorDriver/Python', '2021-05-31_01-08-03-603_raw.csv');
%data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '');
idx = find(data.DutyCycle > 0 & data.TimerFrequency >= 250 & data.TimerFrequency <= 5000);
data = Extract(data, idx(1), idx(end));

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
ax1 = subplot(2,1,1);
plot(data.time, data.TimerFrequency);
ylabel('Frequency [Hz]');
ax2 = subplot(2,1,2);
plot(data.time, data.DutyCycle);
ylabel('Duty Cycle [%]');
xlabel('Time [s]');
linkaxes([ax1,ax2], 'x');

%%
R_guess = data.VinON(end) / data.CurrentON(end) * data.DutyCycle(end)

dt = 1./data.TimerFrequency;

t_OFF = (data.TriggerOFF - data.DutyCycle) .* dt;
L_guess = real(-R_guess * t_OFF(end) / log( data.CurrentOFF(end) / data.CurrentON(end) ))

L_guess = 0.002;

%%
i_ON = @(t_SAMPLE,t_ON,t_OFF,vin,R,L) 1/R*( (vin.*(exp(-R/L*t_OFF)-1))./(1-exp(-R/L*(t_ON+t_OFF))) ).*exp(-R/L*t_SAMPLE) + vin/R;
i_OFF = @(t_SAMPLE,t_ON,t_OFF,vin,R,L) vin/R .* exp(-R/L*t_SAMPLE).*( (1-exp(-R/L*t_ON))./(1-exp(-R/L*(t_ON+t_OFF))) );

i_ON_ = @(R,L, vin, duty, s_sample, freq) i_ON(s_sample./freq, duty./freq, (1-duty)./freq, vin, R, L);
i_OFF_ = @(R,L, vin, duty, s_sample, freq) i_OFF((s_sample-duty)./freq, duty./freq, (1-duty)./freq, vin, R, L);

%% Fit based on ON current
modelfun = @(b,x) i_ON_(b(1), b(2), x(:, 2), x(:, 3), x(:, 4), x(:, 1));
beta0 = [R_guess, L_guess];
X = [data.TimerFrequency, data.VinON, data.DutyCycle, data.TriggerON];
Y = data.CurrentON;
mdl = fitnlm(X, Y, modelfun, beta0);

R_fit = mdl.Coefficients.Estimate(1)
L_fit = mdl.Coefficients.Estimate(2)

%% Fit based on OFF current
modelfun = @(b,x) i_OFF_(b(1), b(2), x(:, 2), x(:, 3), x(:, 4), x(:, 1));
beta0 = [R_guess, L_guess];
X = [data.TimerFrequency, data.VinOFF, data.DutyCycle, data.TriggerOFF];
Y = data.CurrentOFF;
mdl = fitnlm(X, Y, modelfun, beta0);

R_fit = mdl.Coefficients.Estimate(1)
L_fit = mdl.Coefficients.Estimate(2)

%% Fit based on combined ON+OFF current
modelfun = @(b,x) (x(:, 5) .* i_ON_(b(1), b(2), x(:, 2), x(:, 3), x(:, 4), x(:, 1)) + ...
                   (1-x(:, 5)) .* i_OFF_(b(1), b(2), x(:, 2), x(:, 3), x(:, 4), x(:, 1)));
beta0 = [R_guess, L_guess];
X = [[data.TimerFrequency, data.VinON, data.DutyCycle, data.TriggerON, ones(size(data.VinON))];
     [data.TimerFrequency, data.VinOFF, data.DutyCycle, data.TriggerOFF, zeros(size(data.VinOFF))]];
Y = [data.CurrentON;
     data.CurrentOFF];
mdl = fitnlm(X, Y, modelfun, beta0);

R_fit = mdl.Coefficients.Estimate(1)
L_fit = mdl.Coefficients.Estimate(2)

%%
figure(100);
plot(data.time, data.CurrentON, '.');
hold on;
plot(data.time, data.CurrentOFF, '.');
hold off;
legend('Current ON', 'Current OFF');

testFrequency = (1:min(data.TimerFrequency))';

hold on;
plot(data.time, i_ON_(R_guess, L_guess, data.VinON, data.DutyCycle, data.TriggerON, data.TimerFrequency));
plot(data.time, i_ON_(R_fit, L_fit, data.VinON, data.DutyCycle, data.TriggerON, data.TimerFrequency));
plot(data.time, i_OFF_(R_guess, L_guess, data.VinOFF, data.DutyCycle, data.TriggerOFF, data.TimerFrequency));
plot(data.time, i_OFF_(R_fit, L_fit, data.VinOFF, data.DutyCycle, data.TriggerOFF, data.TimerFrequency));

%plot(testFrequency, i_ON_(R_fit, L_fit, data.VinON(1), data.DutyCycle(1), data.TriggerON(1), testFrequency));
%plot(testFrequency, i_OFF_(R_fit, L_fit, data.VinOFF(1), data.DutyCycle(1), data.TriggerOFF(1), testFrequency));
hold off;