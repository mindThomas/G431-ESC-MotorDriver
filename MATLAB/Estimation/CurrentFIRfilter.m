clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../Parameters'));
Parameters_Model
Parameters_Controller

%% Discrete moving average FIR filter
AveragingSamples = PWM_Frequency/CurrentController_Freq;
AvgFirTaps = 1/AveragingSamples * ones(1,AveragingSamples);
AvgFir = tf(AvgFirTaps, 1, 1/PWM_Frequency, 'Variable','z^-1')

AvgFirCont = d2c(AvgFir, 'tustin');
%AvgFirDownSampled = d2d(AvgFir, 1/CurrentController_Freq, 'tustin');
AvgFirDownSampled = c2d(AvgFirCont, 1/CurrentController_Freq, 'tustin');

figure(1);
h = bodeplot(AvgFir);
setoptions(h,'FreqUnits','Hz');
hold on;
bode(AvgFirCont);
bode(AvgFirDownSampled);
hold off;

figure(2);
freqz(AvgFirTaps, 1);

figure(3);
[b,a] = tfdata(AvgFirDownSampled);
freqz(b{1}, a{1});

%% Design custom FIR filter for anti-aliasing before downsampling
omega_n = (2*pi* PWM_Frequency/2) / AveragingSamples;
omega_s = 2*pi*PWM_Frequency/2;
customFir = fir1(AveragingSamples-1, omega_n/omega_s)

figure(4);
freqz(h, 1)
hold on;
freqz(AvgFirTaps, 1);
hold off;

%% Compare step response
figure(5);
stepSignal = ones(1,20);
avgOut = filter(AvgFirTaps, 1, stepSignal);
firOut = filter(customFir, 1, stepSignal);
stairs(avgOut);
hold on;
stairs(firOut);
hold off;
legend('Avg', 'Custom');