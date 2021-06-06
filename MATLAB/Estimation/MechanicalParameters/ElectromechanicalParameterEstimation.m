figure(1);
subplot(2,1,1);
plot(ramp_data(2:end,2), ramp_data(2:end,1));
ylabel('Angular velocity [rad/s]');
xlabel('Current [A]');

l = Line(false);
l = l.fit(ramp_data(2:end,2), ramp_data(2:end,1));
l.plot(min(ramp_data(2:end,2)), max(ramp_data(2:end,2)), 'r');

B2 = Kt/l.a
tau_c2 = -l.b*B2
tau_c

subplot(2,1,2);
plot(ramp_data(2:end,3), ramp_data(2:end,1));
ylabel('Angular velocity [rad/s]');
xlabel('Applied Voltage [V]');

l = Line(false);
l = l.fit(ramp_data(2:end,3), ramp_data(2:end,1));
l.plot(min(ramp_data(2:end,3)), max(ramp_data(2:end,3)), 'r');

% l.a = Kt / (B*R + Kt*Ke)
% l.b = -R / (B*R + Kt*Ke)
Ke2 = 1/l.a - B*R/Kt
Ke

c = 1/B * (-tau_c/l.b - B);
Ke3 = 1/l.a * (c / (1+c))

R2 = Kt/B * (1/l.a - Ke3)
R