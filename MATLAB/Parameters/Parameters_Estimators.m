%% Motor Current estimator
X_init = [0; R*1.01];
P_init = diag([0.01, 0.0005]);

RLekf_X_init = [0; 1; 1e-3]; %[0; R*1.1; L];
RLekf_P_init = diag([1, 0.1, 0.001]);