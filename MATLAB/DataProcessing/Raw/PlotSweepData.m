f = fopen('C:\Users\Thomas\Desktop\frequency_sweep.txt');
data = uint8(fread(f));
fclose(f);

G = 9.143;
Voffset = 2.057;
Rsense = 0.015; % 15 mOhm

time_resolution = 1 / 100000; % 100 kHz timer
sweep_samples = 1000;
skip_samples = 2;

% Process data
% Create parsed struct
offset = 1;
[time_low, offset] = parseData(data, offset, 'uint32', sweep_samples);
[sample_low, offset] = parseData(data, offset, 'int16', sweep_samples);
[time_high, offset] = parseData(data, offset, 'uint32', sweep_samples);
[sample_high, offset] = parseData(data, offset, 'int16', sweep_samples);
[time_vin, offset] = parseData(data, offset, 'uint32', 2*sweep_samples);
[sample_vin, offset] = parseData(data, offset, 'int16', 2*sweep_samples);

% Change type
time_low = double(time_low(1+skip_samples:end)) * time_resolution;
sample_low = double(sample_low(1+skip_samples:end));
time_high = double(time_high(1+skip_samples:end)) * time_resolution;
sample_high = double(sample_high(1+skip_samples:end));

t0 = min([time_high; time_low]);
time_low = time_low - t0;
time_high = time_high - t0;

% Combine data
time = [time_high; time_low];
samples = [sample_high; sample_low];
dataRaw = [time, samples];
dataRaw = sortrows(dataRaw, 1);

clear data;
data.time = dataRaw(:,1); % skip the first 2 data points
data.raw = dataRaw(:,2);

% Extract Vin samples
time_vin = double(time_vin(1+2*skip_samples:end)) * time_resolution - t0;
sample_vin = double(sample_vin(1+2*skip_samples:end));

%% Process data
Vin = (sample_vin / 4095) * 3.3 * (169+18) / (18) * 0.982;

sample_index = (0:(length(time_low)-1))';
sample_index2(1:2:2*length(sample_index),1) = sample_index;
sample_index2(2:2:2*length(sample_index),1) = sample_index;

duty_cycle = 0.1 + 0.8 * (sample_index2 - mod(sample_index2,10)) ./ sweep_samples;
Vmot = duty_cycle .* Vin;

data.sense_voltage = (data.raw / 4095) * 3.3;
data.current = (data.sense_voltage - Voffset) / (G * Rsense);

figure(1);
plot(time_vin, Vin);

figure(2);
plot(data.time(1:2:end), data.sense_voltage(1:2:end));
hold on;
plot(data.time(2:2:end), data.sense_voltage(2:2:end));
hold off;
legend('OFF-period voltage', 'ON-period voltage');

%%
figure(3);
plot(data.time(1:2:end), data.current(1:2:end));
hold on;
plot(data.time(2:2:end), data.current(2:2:end));
hold off;
legend('OFF-period current', 'ON-period current');

%%
figure(4);
plot(time_vin, duty_cycle);
title('Input duty cycle')

figure(5);
plot(time_vin, Vmot);

%%
idx = 1:length(data.time);
idx = idx(0.1*length(idx):end);

R = Vmot ./ data.current;

figure(6);
plot(data.time, R);
ylim([0, 10]);

figure(7);
plot(Vmot(idx), data.current(idx), '*')

l = Line(false);
l = l.fit(Vmot(idx), data.current(idx))

Rest = 1/l.a

hold on;
l.plot(0, max(Vmot(idx)));
hold off;