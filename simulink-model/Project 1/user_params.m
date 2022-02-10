try
    params
    disp('Parameters loaded ...')
catch
    disp('No parameter found')
end

%% Set the trust factor (for Gyro) (t=R*C, f_cut=1/(2*pi*t))
r = 0.98;

%% Constant
rad2deg = 180/pi; % rad2deg

%% Load and set simulink parameters
load('FlightData.mat');
Ts = Acc_x.time(2)-Acc_x.time(1);
disp(['The final time is ',num2str(Acc_x.time(end))]);
disp(['The sampling time is ',num2str(Ts)]);

try
    set_param('user_test', 'StopTime', ...
               num2str(Acc_x.time(end)));
catch
    disp('Skip user_test.')
end
try
    set_param('closed_loop', 'StopTime', ...
           num2str(Acc_x.time(end)));
catch
    disp('Skip closed_loop.')
end

%% Tuning matrix (hide here)
try
    if nstates == 5
        Q = diag([100,100,10,10,80])*100;
        R = diag([1,1,1,1]/10);
        N = zeros(nstates,4);
    elseif nstates == 6
        Q = diag([1000,1000,1,10,10,10])*100; % [1000,1000,1,10,10,10]
        R = diag([1,1,1,1]);
        N = zeros(nstates,4);
    else
        error('Supported numbers of states: 5 or 6.')
    end
catch
    disp('No states specified.')
end
    
