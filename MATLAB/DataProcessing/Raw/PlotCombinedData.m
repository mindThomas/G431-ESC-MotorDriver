f = fopen('C:\Users\Thomas\Desktop\combined_dump.txt');
data = uint8(fread(f));
fclose(f);

G = 9.143;
Voffset = 2.057;
Rsense = 0.015; % 15 mOhm

ticksInternal = 64;
nGear = 30; % gear ratio

time_resolution = 1 / 100000; % 100 kHz timer
sweep_samples = 2000;
skip_samples = 2;

% Process data
% Create parsed struct
offset = 1;
[time, offset] = parseData(data, offset, 'uint32', sweep_samples);
[sample_current_sense, offset] = parseData(data, offset, 'int16', sweep_samples);
[sample_encoder, offset] = parseData(data, offset, 'int32', sweep_samples);

% Change type
time = double(time(1+skip_samples:end)) * time_resolution;
sample_current_sense = double(sample_current_sense(1+skip_samples:end));
sample_encoder = double(sample_encoder(1+skip_samples:end));

clear data;
data.time = time - time(1);
data.raw = sample_current_sense;
data.encoder = sample_encoder - sample_encoder(1);
data.angle_inner = mod(2*pi * data.encoder / ticksInternal, 2*pi);
data.angle_outer = mod(2*pi * data.encoder / (ticksInternal*nGear), 2*pi);
data.sense_voltage = (data.raw / 4095) * 3.3;
data.current = (data.sense_voltage - Voffset) / (G * Rsense);

%%
figure(1);
ax1 = subplot(2,1,1);
plot(data.time, data.current);
vline(data.time(abs(diff(data.angle_inner)) > 1));
ylabel('Average Current sense [A]')
xlabel('Time [s]');

ax2 = subplot(2,1,2);
plot(data.time, rad2deg(data.angle_inner));
vline(data.time(abs(diff(data.angle_inner)) > 1));
ylabel('Inner rotor angle [deg]')
xlabel('Time [s]');

linkaxes([ax1, ax2], 'x');