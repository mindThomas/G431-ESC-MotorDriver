%% Model a sine-wave State Space
f = 65; % example frequency
omega = 2*pi*f;

% Oscillator definition
A = [0, omega;
    -omega, 0];
B = [];
C = [1, 0];
C2 = [1, 0; 0, 1];
D = [];

sinewave = ss(A, B, C, D);
f_sample = 200;
sinewaveTustin = c2d(sinewave, 1/f_sample, 'tustin');
sinewaveZOH = c2d(sinewave, 1/f_sample, 'zoh');

% Simulate the response for 2 cycles
x0 = [2; 0];
tFinal = 2/f;
figure(1);
initial(sinewave, x0, tFinal);
hold on;
initial(sinewaveTustin, x0, tFinal);
initial(sinewaveZOH, x0, tFinal);

ts = 1/f_sample;
t = (0:ts:tFinal)';
states = initial(ss(A, B, C2, D), x0, t);
plot(t, states(:,1));

hold off;
legend('Continuous', 'Tustin', 'ZOH', 'Sampled continuous');

amplitude = sqrt(diag(states*states'))

%% Symbolic derivation of A matrix
syms omega ts real;
A = [0, omega;
    -omega, 0];
Ad = expm(A*ts)

%% Test Tracking EKF
tFinal = 50/f; % create measurements from 50 cycles
t = (0:ts:tFinal)';
states = initial(ss(A, B, C2, D), x0, t);

z = states(:,1);
ts = 1/f_sample;
t = (0:ts:(ts*(length(z)-1)))';
f_initialGuess = 10; % it is very important to choose this within range
X = [2*pi*f_initialGuess; 0.1; 0];
P = diag([1, 1, 1]);

% If the Kalman filter diverges, i.e. by dropping the frequency estimate to
% 0, please try and increase the frequency initial guess.
% Due to the non-linearity in the model, coming from the cyclic nature of sine/cosines
% which is also experienced as aliasing, it is important to chose the
% initial guess properly

Xout = [];
Pout = {};
innovOut = [];
for (i = 1:length(z))
    [X, P, innov] = SineWaveEKF(X, P, ts, z(i));
    Xout(:,i) = X;
    Pout{i} = P;
    innovOut(i) = innov;
end

figure(3);
subplot(3,1,1);
plot(t,z);
hold on;
plot(t,Xout(2,:));
hold off;
legend('Signal', 'Estimate');

subplot(3,1,2);
plot(t,Xout(1,:)/(2*pi));
legend('Frequency estimate');
ylabel('Frequency [Hz]');

subplot(3,1,3);
plot(t, innovOut);
legend('Innovation');
xlabel('Time [s]');




%% Test Tracking EKF
tFinal = 50/f; % create measurements from 50 cycles
t = (0:ts:tFinal)';
states = initial(ss(A, B, C2, D), x0, t);
states = states(30:end,:); % create phase-shift

z = states(:,1);
ts = 1/f_sample;
t = (0:ts:(ts*(length(z)-1)))';
f_initialGuess = f_sample/4; % it is very important to choose this within range
X = [deg2rad(0); 2*pi*f_initialGuess; 0.1];
P = diag([(pi/3)^2, 100, 2]);

% If the Kalman filter diverges, i.e. by dropping the frequency estimate to
% 0, please try and increase the frequency initial guess.
% Due to the non-linearity in the model, coming from the cyclic nature of sine/cosines
% which is also experienced as aliasing, it is important to chose the
% initial guess properly

Xout = [];
Pout = {};
signalEst = [];
innovOut = [];
for (i = 1:length(z))
    [X, P, signal, innov] = SineWaveEKF2(X, P, ts, z(i));
    Xout(:,i) = X;
    Pout{i} = P;
    signalEst(i) = signal;
    innovOut(i) = innov;
end

figure(3);
subplot(4,1,1);
plot(t,z);
hold on;
plot(t,signalEst);
hold off;
legend('Signal', 'Estimate');

subplot(4,1,2);
plot(t,Xout(2,:)/(2*pi));
legend('Frequency estimate');
ylabel('Frequency [Hz]');

subplot(4,1,3);
plot(t,Xout(3,:));
legend('Amplitude estimate');
ylabel('Amplitude');

subplot(4,1,4);
plot(t, innovOut);
legend('Innovation');
xlabel('Time [s]');