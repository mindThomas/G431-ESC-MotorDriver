scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../Parameters'));
addpath(fullfile(scriptDir, '../MATLAB-tools'));

% Define parameters as symbolic
syms R L positive; % electrical
syms J B tau_c positive; % mechanical
syms Ke Kt positive; % electromechanical

% Define input
syms u real; % voltage/armature input
syms tau_l real; % load torque

% Define states
syms i di real; % motor current
syms omega domega real; % angular velocity
syms theta dtheta real; % angle/position

%% Define electrical model
electrical = u == R*i + L*di + Ke*omega;

%% Define mechanical model
tau_m = Kt*i;
mechanical1 = J*domega == tau_m - tau_l - B*omega;
mechanical2 = dtheta == omega;

%% Combine model
diff_eq = simplify([electrical; mechanical1; mechanical2]);
states = [i; omega; theta];
dstates = [di; domega; dtheta];
inputs = [u; tau_l];

res = solve(diff_eq, dstates);

fields = fieldnames(res);
dstates_eq = sym('dstates_eq', size(fields))
for (j = 1:size(fields,1))
    field = fields(j);
    field = field{1};
    dstates_eq(j) = res.(field);
    txt = sprintf('%s\t= %s', field, res.(field));
    disp(txt);
end

%% Extract A matrix
A = sym('A', [size(states,1), size(states,1)]);

fields = fieldnames(res);
for (j = 1:size(fields))
    for (k = 1:size(states))
        eq = dstates_eq(j);
        components = coeffs(collect(eq, states), states(k), 'all');
        if (size(components,2) > 1)
            A(j,k) = components(1);
            dstates_eq(j) = eq - A(j,k)*states(k);
        else
            A(j,k) = 0;
        end
    end
end


%% Extract B matrix
B = sym('B', [size(states,1), size(inputs,1)]);

fields = fieldnames(res);
for (j = 1:size(fields))
    for (k = 1:size(inputs))
        eq = dstates_eq(j);
        components = coeffs(collect(eq, inputs), inputs(k), 'all');
        if (size(components,2) > 1)
            B(j,k) = components(1);
            dstates_eq(j) = eq - B(j,k)*inputs(k);
        else
            B(j,k) = 0;
        end
    end
end
return;

%% State space to Transfer function
% Select from u to i
C = diff(states, i)';
Bin = diff(inputs, u);

% Transfer function
syms s;
G = C * inv(s*eye(size(A)) - A) * B * Bin;

pretty(G)

%% Get transfer functions with model parameters
Parameters_Model;
Geval = syms2tf(vpa(subs(G)))

bode(Geval);