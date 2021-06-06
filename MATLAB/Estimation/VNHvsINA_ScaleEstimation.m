f = fopen('C:\Users\Thomas\Dropbox\DC motor driver project\Data and measurements\Stalled test\vnh7070_scale_dump2.bin');
data = uint8(fread(f));
fclose(f);

%% Create parsed struct
offset = 1;
[time_low, offset] = parseData(data, offset, 'uint32', 400);
[sample_low, offset] = parseData(data, offset, 'int16', 400);
[time_high, offset] = parseData(data, offset, 'uint32', 400);
[sample_high, offset] = parseData(data, offset, 'int16', 400);

time_low(end) = [];
sample_low(end) = [];
time_high(end) = [];
sample_high(end) = [];
time_low(end) = [];
sample_low(end) = [];
time_high(end) = [];
sample_high(end) = [];

%% Change type
time_low = double(time_low);
sample_low = double(sample_low);
time_high = double(time_high);
sample_high = double(sample_high);

%% Process data
time_resolution = 1 / 100000;

time_offset = min([time_high; time_low]);
time = [time_high-time_offset; time_low-time_offset] * time_resolution;
samples = [sample_high; sample_low];
data = [time, samples];
data = sortrows(data, 1);

INA_time = data(sort([1:4:length(data), 2:4:length(data)]),1);
INA_CS = (data(sort([1:4:length(data), 2:4:length(data)]),2) / 4095) * 3.3;
VNH_time = data(sort([3:4:length(data), 4:4:length(data)]),1);
VNH_CS = (data(sort([3:4:length(data), 4:4:length(data)]),2) / 4095) * 3.3;

% Compute current
INA_Current = INA_CS / 0.4;
VNH_Current = VNH_CS * 1.98529;

%% Visualize
scale = 1.14;
offset = 0.07;
% Least squares parameter fitting
% A*x = B
% Parameters x = [scale, offset]
B = INA_Current; % true current
A = [VNH_Current, ones(size(VNH_Current))];
A(2:2:end,:) = 4 * A(2:2:end,:);
B(2:2:end,:) = 4 * B(2:2:end,:);
x = A\B;
scale = x(1)
offset = x(2)

figure(1);
subplot(2,1,1);
plot(INA_time, INA_Current, 'r*');
hold on;
plot(VNH_time, VNH_Current, 'b*');
hold off;
xlabel('Time [s]');
ylabel('Current [A]');
legend('INA180', 'VNH7070');
title('Before scale and offset calibration');

subplot(2,1,2);
plot(INA_time, INA_Current, 'r*');
hold on;
plot(VNH_time, VNH_Current*scale+offset, 'b*');
hold off;
xlabel('Time [s]');
ylabel('Current [A]');
legend('INA180', 'VNH7070');
title('After scale and offset calibration');