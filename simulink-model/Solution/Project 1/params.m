%%% Parameter values
m = 0.027;                  % Mass
g = 9.81;                   % Gravitational acceleration
d = 0.046;                  % Distance from center of mass to rotor axis
k = 2.5383e-11;             % Drag constant k
b = 1.9796e-9;              % Lift constant b
Ts = 0.01;                  % Sampling time
dt = 0.01;		            % Sampling time still
J = [1.1463e-5, 0, 0;
     0, 1.6993e-5, 0;
     0, 0, 2.9944e-5];      % Inertia matrix (kg*m^2)
MF = 0.06*g/65536/4;        % Motor signal to force

%%% Init state
% init_pos = [0;0;0];         % Position
% init_v = [0;0;0];           % Velocity
% init_a = [0;0;0];           % Acceleration
% init_angle = [0;0;0];       % Angle
% init_w = [0;0;0];          % Angular velocity