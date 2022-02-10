% constant
syms m g T real

% Body frame


% Earth frame
angle = sym('angle',[3,1],'real');

% Earth -> Body (XYZ)
R_e2b_x = [1,             0,              0;
           0, cos(angle(1)), -sin(angle(1));
           0, sin(angle(1)),  cos(angle(1))];
R_e2b_y = [ cos(angle(2)), 0, sin(angle(2)); 
                        0, 1,             0; 
           -sin(angle(2)), 0, cos(angle(2))];
R_e2b_z = [ cos(angle(3)), sin(angle(3)), 0;
           -sin(angle(3)), cos(angle(3)), 0; 
                        0,             0, 1];
R_e2b = R_e2b_x * R_e2b_y * R_e2b_z;
R_b2e = R_e2b';

%
a_e = R_b2e * [0, 0, T/m]' - [0, 0, g]';
a_b = R_e2b * ([0, 0, g]' + a_e);
simplify(a_e)
simplify(a_b)

