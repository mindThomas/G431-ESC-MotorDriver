ts = 1/1000000;
t = 0:ts:0.010;
t = t(1:end-1);

pwm_freq = 1000;
duty = 100/256;
period_samples = round((1/pwm_freq) / ts);
on_samples = round(period_samples * duty);
off_samples = period_samples - on_samples;
actualDuty = on_samples / (on_samples + off_samples)

y = repmat([ones(1,on_samples),zeros(1,off_samples)], 1, round(size(t,2)/period_samples));

stairs(t,y)

%
freq = 1000;
omeg = 2*pi*freq;
s = tf('s');
lpf = omeg / (s+omeg);
dlpf = c2d(lpf, ts, 'tustin');

filtered = lsim(lpf, y, t);

sampleRate = 10000;
skipIdx = round(1 / (ts * sampleRate));
actualSampleRate = 1 / (skipIdx * ts)

plot(t, filtered);
hold on;
plot(t(1:skipIdx:end), filtered(1:skipIdx:end), 'ro');
hold off;

samples = filtered(1:skipIdx:end);
err = 100 * abs(samples(end) - actualDuty) / actualDuty