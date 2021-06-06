%% CH3 current sense @ Vref=3.28666234 V
X = [
    1, 2.17693
    1.5, 2.2427299
    2, 2.30210805
    2.5, 2.36951017
    3, 2.43450522
    3.5, 2.50030279
    4, 2.55887842
    4.5, 2.63189769
    5, 2.69430232
];

figure(1);
plot(X(:,2), X(:,1), '.');

l = Line();
l = l.fit(X(:,2), X(:,1))
l.plot(min(X(:,2)), max(X(:,2)), 'r');

xlabel('Vsense ADC [V]');
ylabel('Current setpoint [A]');

Voffset = -l.b/l.a

%% CH1 Current sense @ Vref=3.28666234 V

X = [
    1, 2.14483595
    1.5, 2.21077132
    2, 2.27562857
    2.5, 2.3398211
    3, 2.41042757
    3.5, 2.46737146
    4, 2.53239894
    4.5, 2.59899879
    5, 2.66399384
];

figure(1);
plot(X(:,2), X(:,1), '.');

l = Line();
l = l.fit(X(:,2), X(:,1))
l.plot(min(X(:,2)), max(X(:,2)), 'r');

xlabel('Vsense ADC [V]');
ylabel('Current setpoint [A]');

Voffset = -l.b/l.a


%% CH1 Current sense (forward direction)
data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '2020-05-09_22-06-20-789.csv');
figure(1);
plot(data.current_on);

%%
X = [
    0.5, 373
    1.0, 971
    1.5, 1566
    2.0, 1981
    2.5, 2611
    3, 3360
    3.5, 4084
    4, 4726
    4.5, 5295
    5, 5743
];

figure(1);
plot(data.current_on(uint32(X(:,2))), X(:,1), '.');

l = Line();
l = l.fit(data.current_on(uint32(X(:,2))), X(:,1))
l.plot(min(data.current_on(uint32(X(:,2)))), max(data.current_on(uint32(X(:,2)))), 'r');

xlabel('Vsense ADC [V]');
ylabel('Current setpoint [A]');

Voffset = -l.b/l.a

%% CH3 Current sense (forward direction)
data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '2020-05-09_22-17-24-880.csv');
figure(1);
plot(data.current_on);

%%
X = [
    0.5, 169
    1.0, 613
    1.5, 1205
    2.0, 1559
    2.5, 1928
    3, 2406
    3.5, 2863
    4, 3347
    4.5, 3774
    5, 4280
];

figure(1);
plot(data.current_on(uint32(X(:,2))), X(:,1), '.');

l = Line();
l = l.fit(data.current_on(uint32(X(:,2))), X(:,1))
l.plot(min(data.current_on(uint32(X(:,2)))), max(data.current_on(uint32(X(:,2)))), 'r');

xlabel('Vsense ADC [V]');
ylabel('Current setpoint [A]');

Voffset = -l.b/l.a