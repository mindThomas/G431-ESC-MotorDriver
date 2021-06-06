s = tf('s');

Fs = 200;
delay = (1-(1/Fs * s/2)) / (1+(1/Fs * s/2)); % pade delay
H = tf(1/R); % i(s) / u(s)

Kp = 1;

OpenLoop = H*Kp*delay;
ClosedLoop = feedback(H, Kp*delay)

bode(ClosedLoop);

rlocus(H*delay)