% omega_steadystate = Kt/B * i - 1/B * tau_c
% omega_steadystate = Kt/(B*R + Kt*Ke) * Vmot - R/(B*R + Kt*Ke) * tau_c
% omega_coast = 

% x(:, 1) = steadystate current mode
% x(:, 2) = steadystate voltage mode
% x(:, 3) = coast mode
% x(:, 4) = i, current
% x(:, 5) = Vmot, motor voltage
% x(:, 6) = delta_t
% b(1) = R
% b(2) = B
% b(3) = Ke
% b(4) = tau_c
% b(5) = J
% b(6) = omega0
coast_idx = find(ramp_data(:,4) == 0);
coast_idx = coast_idx(1);

omega = ramp_data(2:(coast_idx-1),1);
current = ramp_data(2:(coast_idx-1),2);
vmot = ramp_data(2:(coast_idx-1),3);


omega0 = ramp_data(coast_idx,1);
t0 = ramp_data(coast_idx,5);
delta_t = ramp_data((coast_idx+1):(coast_idx+1+17),5) - t0;
omega_decel = ramp_data((coast_idx+1):(coast_idx+1+17),1);

omega_decel_test = (850.5 + 1/B*tau_c)*exp(-B/J.*delta_t) - 1/B*tau_c;

beta0 = [1, 1e-6, 1e-3, 1e-4, 1e-7, omega0/2];
omega_steadystate = @(b,x) x(:, 1) .* (Kt/b(2) * x(:, 4) - 1/b(2) * b(4)) + ...
                           x(:, 2) .* (Kt/(b(2)*b(1) + Kt*b(3)) * x(:, 5) - b(1)/(b(2)*b(1) + Kt*b(3)) * b(4)) + ...
                           x(:, 3) .* (b(6).*exp(-x(:, 6) * b(2)/b(5)) - b(4)/b(2));

X = [ones(length(current),1), zeros(length(current),1), zeros(length(current),1), current, zeros(length(current),1), zeros(length(current),1);
    zeros(length(current),1), ones(length(current),1), zeros(length(current),1), zeros(length(current),1), vmot, zeros(length(current),1);
    zeros(length(delta_t),1), zeros(length(delta_t),1), ones(length(delta_t),1), zeros(length(delta_t),1), zeros(length(delta_t),1), delta_t];
Y = [omega; omega; omega_decel];

mdl = fitnlm(X, Y, omega_steadystate, beta0);


%%
beta0 = [omega0/2,1,1];
omega_steadystate = @(b,x) b(1).*exp(-x(:, 1) * b(2)) - b(3);

X = [delta_t];
Y = [omega_decel];

mdl = fitnlm(X, Y, omega_steadystate, beta0);

figure(2);
subplot(2,1,1);
plot(X, Y);
subplot(2,1,2);
fplot(@(x) omega_steadystate(mdl.Coefficients.Estimate,x), [min(X), max(X)], 'r');