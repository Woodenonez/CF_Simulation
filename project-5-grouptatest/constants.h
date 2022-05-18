
#ifndef CONSTANTS_H
#define CONSTANTS_H

struct crazyflie_constants_t {
    double m; // Mass
    double g; // Gravitational acceleration
    double d; // Distance from center of mass to rotor axis
    double k; // Drag constant k
    double b; // Lift constant b
    double j_x; // Diagonal
    double j_y; // inertia
    double j_z; // matrix
    double pwm_n; // pwm to newton conversion rate
};

const struct crazyflie_constants_t crazyflie_constants;

#endif
