% Script to visualize the gain compensations needed to correct the current
% samples collected from the center of the ON-period to get the average
% current

PWM_Frequency = 20000;

duty = 0.01:0.01:0.99;
G_compensation_ON = SampleONcompensationGain(PWM_Frequency, duty, R, L)
G_compensation_OFF = SampleOFFcompensationGain(PWM_Frequency, duty, R, L)

figure(1);
subplot(1,2,1);
plot(duty, G_compensation_ON);
xlabel('Duty [%]');
ylabel('Gain');
title('Compensation Gain from mid ON-period current sample to Average current)');

subplot(1,2,2);
plot(duty, G_compensation_OFF);
xlabel('Duty [%]');
ylabel('Gain');
title('Compensation Gain from mid OFF-period current sample to Average current)');