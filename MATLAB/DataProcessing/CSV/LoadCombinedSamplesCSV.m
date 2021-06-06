data = LoadRawDump('C:\Users\Thomas\Documents\motordriver-python\', '');

figure(2);
ax1 = subplot(2,1,1);
plot(data.time, data.Vin);
ax2 = subplot(2,1,2);
plot(data.time, data.CurrentON);
%plot(data.current_on);
hold on;
plot(data.time, data.CurrentOFF);
%plot(data.current_off);
hold off;
legend('Current ON', 'Current OFF');
linkaxes([ax1, ax2], 'x');

figure(3);
ax1 = subplot(2,1,1);
plot(data.time, data.DutyCycle);
ax2 = subplot(2,1,2);
plot(data.time, data.TriggerON);
hold on;
plot(data.time, data.TriggerOFF);
hold off;
legend('Trigger ON', 'Trigger OFF');
linkaxes([ax1, ax2], 'x');


figure(4);
ax1 = subplot(2,1,1);
plot(data.time, data.RPM);
ax2 = subplot(2,1,2);
plot(data.time, data.bemf);
linkaxes([ax1, ax2], 'x');

%% Calibrate Bemf with Voltage divider on
v_bemf1 = data.Bemf(1:2:end-1);
v_bemf2 = data.Bemf(2:2:end);

if (max(v_bemf1) > max(v_bemf2))
    v_bemf_raw = v_bemf1;
    v_bemf_divided = v_bemf2;
else
    v_bemf_raw = v_bemf2;
    v_bemf_divided = v_bemf1;
end

figure(2);
subplot(2,1,1);
plot(v_bemf_raw);
hold on;
plot(v_bemf_divided);
hold off;
legend('Raw', 'Voltage divided');

idx0 = 500;
idx1 = 678;

bemf_calibration = Line();
bemf_calibration = bemf_calibration.fit(v_bemf_divided(idx0:idx1), v_bemf_raw(idx0:idx1))

subplot(2,1,2);
plot(v_bemf_raw);
hold on;
plot(bemf_calibration.a * v_bemf_divided + bemf_calibration.b);
hold off;
title('Compensated');
legend('Raw', 'Voltage divided');

v_bemf = bemf_calibration.a * v_bemf_divided + bemf_calibration.b

%%
idx0 = find(data.DutyCycle == 0);
idx0 = idx0(1)+2;
idx1 = 80;

speed = data.Speed(1:2:end-1);
v_bemf_ = v_bemf(idx0/2:end);
speed = speed(idx0/2:end);
v_bemf_ = v_bemf_(1:idx1);
speed = speed(1:idx1);

% figure(6);
% plot(v_bemf);
% hold on;
% plot(speed);
% hold off;
% 
% l_speed = Line();
% l_bemf = Line();
% 
% l_speed = l_speed.fit((1:length(speed))', speed);
% l_bemf = l_bemf.fit((1:length(v_bemf))', v_bemf);
% 
% l_speed.plot(1, length(speed), 'r');
% l_bemf.plot(1, length(v_bemf), 'r');


figure(7);
plot(speed, v_bemf_, '*');
l_ke = Line(true);
l_ke = l_ke.fit(speed, v_bemf_)
l_ke.plot(min(speed), max(speed), 'r');