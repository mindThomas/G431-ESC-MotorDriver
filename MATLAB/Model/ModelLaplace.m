scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../Parameters'));
addpath(fullfile(scriptDir, '../MATLAB-tools'));

% Define parameters as symbolic
syms R L positive; % electrical
syms J B tau_c positive; % mechanical
syms Ke Kt positive; % electromechanical

% Define input
syms u; % voltage/armature input
syms tau_l; % load torque

% Define states
syms i; % motor current
syms omega; % angular velocity
syms theta; % angle/position

% Define Laplace operator
syms s imaginary;

%% Define electrical model
electrical = u == R*i + L*s*i + Ke*omega;

%% Define mechanical model
tau_m = Kt*i;
mechanical = J*s*omega == tau_m - tau_l - B*omega;

%% Extract model to i (electrical only)
i_electrical = solve(electrical, i);
pretty(i_electrical);

%% Extract model to omega (mechanical only)
omega_mechanical= solve(mechanical, omega);
pretty(omega_mechanical);

%% Extract model from u to i
omega_ = solve(electrical, omega);
eq = subs(mechanical, omega, omega_);

components = coeffs(collect(simplify(solve(eq, i)), u), u, 'all');
tf_i_from_u = components(1); % G(s) = i(s) / u(s)
pretty(tf_i_from_u)

%% Extract model from u to omega
i_ = solve(electrical, i);
eq = subs(mechanical, i, i_);

components = coeffs(collect(simplify(solve(eq, omega)), u), u, 'all');
tf_omega_from_u = components(1); % G(s) = omega(s) / u(s)
pretty(tf_omega_from_u)

%% ===== Mechanical model =====
% Extract model from i to omega
u_ = solve(electrical, u);
eq = subs(mechanical, u, u_);

components = coeffs(collect(simplify(solve(eq, omega)), i), i, 'all');
tf_omega_from_i = components(1); % G(s) = omega(s) / i(s)
pretty(tf_omega_from_i)

%% ===== Electrical model =====
% Extract model from u to i without mechanical model (assumed static motor)
eq = subs(electrical, omega, 0);

components = coeffs(collect(simplify(solve(eq, i)), u), u, 'all');
tf_i_from_u_nomech = components(1); % G(s) = i(s) / u(s)
pretty(tf_i_from_u_nomech)

%% Identify combined poles
[num,den] = numden(tf_omega_from_u);
quadraticComponents = coeffs(collect(den, s), s, 'all');

% Solve quadratic equation to find roots (s)
a = quadraticComponents(1);
b = quadraticComponents(2);
c = quadraticComponents(3);

% b^2 - 4*a*c > 0   -->  2 real roots (we expect this)
pole1 = (-b + sqrt(b^2 - 4*a*c)) / (2*a);
pole2 = (-b - sqrt(b^2 - 4*a*c)) / (2*a);


%% Get transfer functions with model parameters
Parameters_Model;

Geval = syms2tf(vpa(subs(tf_i_from_u_nomech)))
figure(1);
subplot(1,2,1);
pzmap(Geval);
title('Electrical model');
subplot(1,2,2);
bode(Geval);

Geval = syms2tf(vpa(subs(tf_omega_from_i)))
figure(2);
subplot(1,2,1);
pzmap(Geval);
title('Mechanical model');
subplot(1,2,2);
bode(Geval);

Geval = syms2tf(vpa(subs(tf_i_from_u)))
figure(3);
subplot(1,2,1);
pzmap(Geval);
title('Combined current model (u to i)');
subplot(1,2,2);
bode(Geval);

Geval = syms2tf(vpa(subs(tf_omega_from_u)))
figure(4);
subplot(1,2,1);
pzmap(Geval);
title('Combined omega model (u to omega)');
subplot(1,2,2);
bode(Geval);