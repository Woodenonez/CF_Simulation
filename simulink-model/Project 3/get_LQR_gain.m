close all
clear

%% Customize
nstates = 5; % 5, 6 or 12
is_discrete = 1; % get discrete state space or not
r0 = 1; % overwrite the complementary factor, [0,1]

%% Load parameters
user_params
r = r0; % overwrite the complementary factor
clear r0 Acc_x Acc_y Acc_z Gyro_x Gyro_y Gyro_z
try
    set_param('closed_loop', 'StopTime', '10');
catch
    disp('Skip closed_loop.')
end

%% Get state space
[A,B,C,D] = genStateSpaceMat(nstates, is_discrete);
% B
%% Compute LQR gain
[K, ~, ~] = dlqr(A,B,Q,R,N);    % dlqr for discrete state space
% [K, ~, ~] = lqrd(A,B,Q,R,N,Ts); % lqrd for continuous state space
% K

