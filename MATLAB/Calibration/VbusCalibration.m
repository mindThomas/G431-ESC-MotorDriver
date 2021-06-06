%% CH1 Current sense (forward direction)
data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '2020-05-09_22-30-55-636.csv');
figure(1);
plot(data.vin_off);

%%
X = [
    9.705, 1090
    9.19, 2875
    8.6, 4758
    8.3, 6463
    7.5, 8976
    10.2, 10446
    10.6, 11862
    11.1, 14196
    12.0, 15780
];

Y = data.vin_off(uint32(X(:,2)));

figure(1);
plot(Y, X(:,1), '.');

l = Line(true);
l = l.fit(Y, X(:,1))
l.plot(min(Y), max(Y), 'r');

xlabel('Vbus Sense ADC [V]');
ylabel('Vbus measured [V]');

Voffset = -l.b/l.a