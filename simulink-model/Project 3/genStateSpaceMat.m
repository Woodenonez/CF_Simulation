function [A,B,C,D] = genStateSpaceMat(nstates, is_discrete)

% Check inputs.
if nargin > 2
    error('Requires at most 2 optional inputs.');
end

switch nargin
    case 0
        nstates = 5;
        is_discrete = false;
    case 1
        is_discrete = false;
end

if ~ismember(nstates, [5,6,12])
    error('The number of states should be 5, 6 or 12.')
end

% Load parameters
try
    params
catch
    error('Cannot load parameters.')
end

% Operation point
init_pos    = [0;0;0];	% Position
init_v      = [0;0;0];	% Velocity
init_a      = [0;0;0];	% Acceleration
init_angle  = [0;0;0];	% Angle
init_w      = [0;0;0];	% Angular velocity
init_u      = m*g/4.*ones(4,1); % Motor force when balance

% Define 12 states and functions
%{ 
    x=[theta_roll, phi_pitch,   psi_yaw,   ...
       omega_roll, omega_pitch, omega_yaw, ...
       px, py, pz, vx, vy, vz]^T
%}
syms t dt real
x = sym('x',[12,1]); % states
u = sym('u',[4,1]); % inputs: F1~F4

Tau = [-1, -1,  1,  1;...
       -1,  1,  1, -1;...
       -1,  1, -1,  1] * u .* [d/sqrt(2); d/sqrt(2); k/b];
   
Rx = [1, 0, 0; 0, cos(x(1)), -sin(x(1)); 0, sin(x(1)), cos(x(1))];
Ry = [cos(x(2)), 0, sin(x(2)); 0, 1, 0; -sin(x(2)), 0, cos(x(2))];
Rz = [cos(x(3)), sin(x(3)), 0; -sin(x(3)), cos(x(3)), 0; 0, 0, 1];

f1 = x(4); % theta_roll
f2 = x(5); % phi_pitch
f3 = x(6); % psi_yaw
f4_6 = J \ (-cross(x(4:6),J*x(4:6)) + Tau);
f7 = x(10); % px
f8 = x(11); % py
f9 = x(12); % pz
f10_12 = [0; 0; -g] + Rx*Ry*Rz*[0;0;sum(u)]/m;

if nstates == 5
    % 5 states (angles without yaw and angular speeds)
    %{ 
        x=[theta, phi, omega_r, omega_p, omega_y]^T
        which is x1~x5.
    %}
    f = [f1; f2; f4_6];
    A = double(subs(jacobian(f,[x(1:2); x(4:6)]), ...
                    [x(1:2); x(4:6)], [init_angle(1:2); init_w]));
    B = double(subs(jacobian(f,u), u, init_u));
    C = 180/pi .* eye(5); % radian to degree
    D = zeros(5,4);
elseif nstates == 6
    % 6 states (angles and angular speeds)
    %{ 
        x=[theta, phi, psi, omega_r, omega_p, omega_y]^T
        which is x1~x6.
    %}
    f = [f1; f2; f3; f4_6];
    A = double(subs(jacobian(f,x(1:6)), x(1:6), [init_angle; init_w]));
    B = double(subs(jacobian(f,u), u, init_u));
    C = 180/pi .* eye(6); % radian to degree
    D = zeros(6,4);
else
    % 12 states (all states)
    %{ 
        x=[theta_roll, phi_pitch,   psi_yaw,   ...
           omega_roll, omega_pitch, omega_yaw, ...
           px, py, pz, vx, vy, vz]^T
        which is x1~12.
    %}
    f = [f1; f2; f3; f4_6; f7; f8; f9; f10_12];
    A = double(subs(jacobian(f,x), [x; u], [zeros(12,1); init_u]));
    B = double(subs(jacobian(f,u), [x; u], [zeros(12,1); init_u]));
    C = blkdiag(180/pi .* eye(6), eye(6)); % radian to degree for angles
    D = zeros(12,4);
end

if is_discrete
    [A,B,C,D] = ssc2d(A,B,C,D,Ts);
end

B = B.*MF.*180/pi; % motor signals to forces, rad to degree

end


% Helper function
function [Ad,Bd,Cd,Dd] = ssc2d(A,B,C,D,Ts)
%     Ad = eye(size(A,1)) + Ts.*A;
%     Bd = Ts.*B;
%     Cd = C;
%     Dd = D;
    sysc = ss(A,B,C,D);
    sysd = c2d(sysc,Ts,'zoh');
    Ad = sysd.A;
    Bd = sysd.B;
    Cd = sysd.C;
    Dd = sysd.D;
end