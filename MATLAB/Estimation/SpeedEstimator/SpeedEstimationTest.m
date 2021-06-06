data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '2020-05-16_13-01-24-195.csv');

t_downsampled = data.time(1):0.01:data.time(end);
encoder_downsampled = interp1(data.time, data.encoder, t_downsampled);
    TicksPrRev = 1920; % this should be moved out of here and loaded from a config
    speed = 2*pi/TicksPrRev * diff(encoder_downsampled) ./ diff(t_downsampled);
    speed(end+1) = speed(end);
    RPM = 60 * speed / (2*pi);
    time_speed = (t_downsampled(1:end-1) + t_downsampled(2:end)) / 2;
    time_speed(end+1) = 2*time_speed(end) - time_speed(end-1);

figure(1);
plot(data.time, data.encoder);
hold on;
plot(t_downsampled, encoder_downsampled);
hold off;

figure(2);
plot(data.time, data.speed);
hold on;
plot(time_speed, speed);
hold off;