fs = 1000;
lpf_freq = 100;

[b,a] = butter(6, lpf_freq / (fs/2));

figure(1);
subplot(2,1,1);
freqz(b,a);
%subplot(2,1,2);
%grpdelay(b,a);

%%
Ts = 1/fs;
lpf_freq = 400;
omega = 2*pi*lpf_freq;
omega_digital = 2*pi* lpf_freq / fs;
omega_prewarped = 2/Ts * tan(omega_digital/2);
tau = 1/omega_prewarped;

s = tf('s');
lpf = (1/tau) / (s+(1/tau));
dlpf = c2d(lpf, 1/fs, 'tustin')
bode(dlpf);