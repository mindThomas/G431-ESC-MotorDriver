clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../Parameters'));
Parameters_Model

s = tf('s');

%%
% Motor (plant) model
G = 1 / (R + L*s);

% Current filter
f_LPF = 500;
omega_LPF = 2*pi * f_LPF; w = omega_LPF;
H_LPF = omega_LPF / (s + omega_LPF);

% Controller parameters
fs = 2000; % controller frequency [Hz]
Ts = 1/fs;
%K = 0.1*R;
%Ti = 0.3*R/L; % 350
K = 1.0;
Ti = 500;%R/L; % 350
Kp = K;
Ki = K * Ti;

% PI controller
D = K * (s + Ti)/s;

% Pade delay (sample approximation)
P = (4/Ts - s) / (4/Ts + s);

% Open loop and direct term
Direct = D * G;
Feedback = P * H_LPF;
OL = Direct * Feedback;

% Feedforward gain
FF = 0*R; % inverse DC gain of plant

% Closed loop
%CL = feedback(Direct, Feedback);
% Closed loop with feedforward
CL = (D+FF)*G / (1 + D*G*P*H_LPF);

figure(1);
subplot(1,2,1);
margin(OL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;
subplot(2,2,2);
rlocus(OL);
subplot(2,2,4);
step(CL);

%%
figure(3);
bode(CL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;

%%
% Compute steady-state error
SSerror = 1 - evalfr(minreal(CL), 0)

bode(CL); setoptions(gcr, 'FreqUnits', 'Hz');

OutputNoiseSupression = feedback(H_LPF, D * G * P);
bode(OutputNoiseSupression); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;

%% Convert to discrete time
Gd = c2d(G, Ts, 'zoh')

% Controller gains
% K = 2.0*R;
% Ti = 200;%0*R/L;
% Kp = K;
% Ki = K * Ti;
% 
K = 2;
Ti = 0*500;%R/L; % 350
Kp = K;
Ki = K * Ti;

% Discrete PI Controller
Dd = c2d(D, Ts, 'tustin')
% u[k]/e[k] = (5.28 - 4.32 z^-1) / (1 - z^-1)
% (1 - z^-1) * u[k] = (5.28 - 4.32 z^-1) * e[k]
% PI controller: u[k] = u[k-1] + 5.28*e[k] - 4.32*e[k-1]
z = tf('z', Ts);
Dd2 = ( Kp*(z-1) + Ki*Ts/2*(z+1) ) / (z - 1);
% Luckily the above are the same :)   (see derivation in OneNote)

Hd_LPF = 1; %c2d(H_LPF, Ts, 'tustin');

% Open loop and direct term
Direct = Dd2 * Gd;
Feedback = Hd_LPF;
OL = Direct * Feedback;

FF = R;

% Closed loop
%CL = feedback(Direct, Feedback);
% Closed loop with feedforward
CL = (Dd2+FF)*Gd / (1 + Dd2*Gd*Hd_LPF); % ref -> output
CL_OutputDisturbance = 1 / (1 + Dd2*Gd*Hd_LPF); % additive disturbance on output -> output
CL_InputDisturbance = Gd / (1 + Dd2*Gd*Hd_LPF); % additive disturbance on plant input -> output

figure(2);
subplot(1,2,1);
margin(OL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;
%bode(CL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;
subplot(2,2,2);
rlocus(OL);
subplot(2,2,4);
step(CL);

%%
figure(3);
bode(CL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;


%%
z = tf('z', 1/20000);
MA = 0;
l = 18;
for (i = 0:(l-1))
    MA = MA + z^(-i);
end
MA = MA / l;

figure(4);
bode(MA); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;

%%
[N,Wn,BTA,FILTYPE] = kaiserord([200 600], [1 0], [0.004 0.1], 20000)
B = fir1(N, Wn, FILTYPE, kaiser( N+1,BTA ), 'noscale' )
B = 1/18 * ones(1, 18);
a_coeff = zeros(1, length(B));
a_coeff(1) = 1;
lpf = tf(B, a_coeff, 1/20000)

bode(lpf); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;

f_LPF = 200;
omega_LPF = 2*pi * f_LPF; w = omega_LPF;
H_LPF = omega_LPF / (s + omega_LPF);

%%
hold on;
bode(lpf);
hold off;

%% Downsampled
