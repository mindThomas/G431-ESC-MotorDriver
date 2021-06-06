data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '', true);
idx = find(data.duty_cycle > 0 & data.current_on ~= 0);
data = ExtractIndices(data, idx);

figure(1);
ax1 = subplot(4,1,1);
plot(data.time, data.timer_frequency);
ylabel('Frequency [Hz]');
ax2 = subplot(4,1,2);
plot(data.time, data.duty_cycle);
ylabel('Duty Cycle [%]');
ax3 = subplot(4,1,3);
plot(data.time, data.RPM);
ylabel('Speed [RPM]');
ax4 = subplot(4,1,4);
plot(data.time, data.current_on, '.');
hold on;
plot(data.time, data.current_off, '.');
hold off;
legend('Current ON', 'Current OFF');
xlabel('Time [s]');
linkaxes([ax1,ax2,ax3,ax4], 'x');

% figure(2);
% plot(data.time, data.trigger_on);
% hold on;
% plot(data.time, data.trigger_off);
% plot(data.time, data.duty_cycle);
% hold off;


%%
idx_change = find(diff(data.duty_cycle) ~= 0);
step_dt = data.time(idx_change(2)) - data.time(idx_change(1));
t_settling = 0.25;

dt_mat = data.time - data.time(idx_change)';
[idx_steady_state, ~] = ind2sub(size(dt_mat), find(dt_mat > t_settling & dt_mat < step_dt));

steady_state = ExtractIndices(data, idx_steady_state);
i_mean = @(R,Ke, duty_cycle, Vin, omega) duty_cycle .* Vin/R - Ke/R*omega;

figure(1);
ax1 = subplot(3,2,1);
plot(steady_state.time, steady_state.duty_cycle, '.');
ylabel('Duty Cycle [%]');
ax2 = subplot(3,2,2);
plot(steady_state.time, steady_state.vin, '.');
ylabel('Vin [V]');
ax3 = subplot(3,2,3);
plot(steady_state.time, steady_state.RPM, '.');
ylabel('Speed [RPM]');
ax4 = subplot(3,1,3);
plot(steady_state.time, steady_state.current_on, '.');
hold on;
plot(steady_state.time, i_mean(R,Ke, steady_state.duty_cycle, steady_state.vin, abs(steady_state.speed)), 'x');
hold off;
legend('Current ON', 'Current OFF');
xlabel('Time [s]');
linkaxes([ax1,ax2,ax3], 'x');

%% Calculate steady state current and velocities from the steady state groups
idx_end = find(diff(steady_state.time) > t_settling);
idx_start = [1; idx_end(1:end-1)+1];

current_steadystate = [];
omega_steadystate = [];
for (i = 1:length(idx_start))
    current_steadystate(i,1) = mean(steady_state.current_on(idx_start(i):idx_end(i)));
    omega_steadystate(i,1) = mean(abs(steady_state.speed(idx_start(i):idx_end(i))));
end

current_steadystate(omega_steadystate < 15) = [];
omega_steadystate(omega_steadystate < 15) = [];


i = 10 - 1;
j = 14 - 1;
figure(1);
subplot(2,1,1);
plot(steady_state.current_on(idx_start(i):idx_end(i))); mean(steady_state.current_on(idx_start(i):idx_end(i)))
hold on;
plot(steady_state.current_on(idx_start(j):idx_end(j))); mean(steady_state.current_on(idx_start(j):idx_end(j)))
hold off;
subplot(2,1,2);
plot(steady_state.speed(idx_start(i):idx_end(i))); mean(steady_state.speed(idx_start(i):idx_end(i)))
hold on;
plot(steady_state.speed(idx_start(j):idx_end(j))); mean(steady_state.speed(idx_start(j):idx_end(j)))
hold off;

%%
figure(1);
subplot(2,1,1);
plot(current_steadystate);
subplot(2,1,2);
plot(omega_steadystate);

figure(2);
plot(current_steadystate, omega_steadystate, '.');
hold on;
plot(i_mean(R,Ke, steady_state.duty_cycle, steady_state.vin, abs(steady_state.speed)), abs(steady_state.speed), '.');
hold off;

l = Line();
l = l.fit(current_steadystate, omega_steadystate)
hold on;
l.plot(min(current_steadystate), max(current_steadystate), 'r');
hold off;

%% Least Squares fitting (2)
% omega_steadystate = Kt/B*i_a - 1/B*tau_c
A = [current_steadystate, ones(length(current_steadystate), 1)];
y = omega_steadystate;
X = lscov(A, y);

Kt_B = X(1);
B
B2 = Kt / Kt_B
tauc_B = -X(2);
tau_c
tau_c2 = B2 * tauc_B

%% Least Squares fitting (3)
% -B/J*t = log( (omega(t) + 1/B*tau_c) / (omega0 + 1/B*tau_c) )
include_idx = (idx_coast+1:length(omega))';
include_idx(omega(include_idx) <= 0.1) = [];
A = [time(include_idx) - t0];
y =  log( (omega(include_idx) + 1/B*tau_c) / (omega0 + 1/B*tau_c) );
X = lscov(A, y);

B_J = -X(1);
J
J2 = B / B_J

rls = RecursiveLeastSquares(A(1,1), y(1))
for (i = 2:length(A))
    rls = rls.AddMeasurement(A(i,:), y(i));    
end
J2 = -B / rls.ComputeEstimate()

% Manual recursive least squares - should yield same result as above
a_squared = A(1,1)^2;
ay = A(1,1) * y(1);
for (i = 2:length(A))
    a_squared = a_squared + A(i,1)^2;
    ay = ay + A(i,1)*y(i);
end
    
% Compute estimate
x_est = ay / a_squared;
J3 = -B / x_est

%% Least Squares fitting (4)
% Vmot = Ke*omega
include_idx = (idx_coast+1:length(omega))';
include_idx(omega(include_idx) <= 0.1) = [];
A = [omega(include_idx)];
y = vmot(include_idx);
X = lscov(A, y);

Ke
Ke2 = X(1)