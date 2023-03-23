clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../Parameters'));
addpath(fullfile(scriptDir, '../../Model'));

%% Load State space model and replace symbolic values with loaded parameters
ModelStateSpace
A_ = A;
B_ = B;

Parameters_Model
Parameters_Controller
Parameters_Estimators

A_ = eval(vpa(subs(A_)))
B_ = eval(vpa(subs(B_)))
states
inputs

%% Design current controller
% Remove theta state
states = states(1:2)
A_ = A_(1:2, 1:2)
B_ = B_(1:2, 1)

C_ = [1,0]; % output is current
D_ = 0;

sys = ss(A_, B_, C_, D_);

sys.StateName = string(states);
sys.InputName = 'u';
sys.OutputName = 'i';


% Current filter
s = tf('s');
w = 2*pi*CurrentController_CurrentLPF_Freq;
LPF = w/(s+w);
filt = ss(LPF);

% % Combine filter and system
% sys = append(sys, filt);
% sys.InputName{end} = 'filt_in';
% sys.OutputName{end} = 'i_filt';
% sys.StateName{end} = 'i_filt_int';
% % Reorder such that the filter input is connected to the current output
% sys.A(3,1) = sys.B(3,2);
% sys.B(3,2) = 0;
% %sys = feedback(sys,1,2,1,1)
% sys = ss(sys.A, sys.B(:,1), sys.C, sys.D(:,1), 'InputName', sys.InputName(1), 'OutputName', sys.OutputName, 'StateName', sys.StateName);
% sys

% Add error integrator
sys = append(sys, ss(0,0,0,0));
sys.StateName{end} = 'e_integral';
sys.InputName{end} = 'i_ref';
% d e_integral / dt = error
% error = i_ref - i
sys.A(end,1) = -1;
%sys.A(end,end-1) = -sys.C(2,3); % compute i_filt from i_filt_int
sys.B(end,end) = 1;
sys = ss(sys.A, sys.B, sys.C(1:end-1,:), sys.D(1:end-1,:), 'InputName', sys.InputName, 'OutputName', sys.OutputName(1:end-1), 'StateName', sys.StateName);
sys

sys = ss(sys.A, sys.B, eye(3), zeros(3,2), 'InputName', sys.InputName, 'OutputName', sys.OutputName(1:end-1), 'StateName', sys.StateName);

%% LQR gains
%% SOMETHING IS WRONG HERE. THE FEEDBACK IS PROBABLY NOT RIGHT, WHY WOULD U END UP GOING NEGATIVE
%lqr for controller
%lqe for estimator/observer
Q = diag([10, 0, 1]);
R = 0.01;
K = lqr(sys.A, sys.B(:,1), Q, R, 0)


CL = feedback(sys, K, 1, [1,2,3])
CL.B(1,2) = K(1);
[Acl, Bcl, Ccl, Dcl] = ssselect(CL.A, CL.B, CL.C, CL.D, 2, 1);
CL_ = ss(Acl, Bcl, Ccl, Dcl);

[p,z] = pzmap(CL_)

figure(2);
steps = step(CL);
i = steps(:,1,2);
omega = steps(:,2,2);
e_integral = steps(:,3,2);
X = [i, omega, e_integral];
u = (K * X')';
subplot(4,1,1);
plot(i);
subplot(4,1,2);
plot(omega);
subplot(4,1,3);
plot(e_integral);
subplot(4,1,4);
plot(u);