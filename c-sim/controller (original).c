#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "controller.h"
#include "constants.h"

void controlSystemTask(void *pvParameters) {
    struct ControlSystemParams *params =
        (struct ControlSystemParams*)pvParameters;

    // we keep local copies of the global state + semaphores
    unsigned short motors[4];
    double gyro_data[3];
    double acc_data[3];
    double r_rpdy[3];
    double estimate[3] = {0.0};

    // copy the semaphore handles for convenience
    SemaphoreHandle_t motors_sem = params->motors_sem;
    SemaphoreHandle_t references_sem = params->references_sem;
    SemaphoreHandle_t sensors_sem = params->sensors_sem;
    SemaphoreHandle_t estimate_sem = params->estimate_sem;

    while(1) {
        // read sensor data (gyro)
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));

        // read filter data (angle estimates)
        memcpy(estimate, params->estimate, sizeof(estimate));

        // read latest references
        memcpy(r_rpdy, params->r_rpdy, sizeof(r_rpdy));

        // compute error


        // example of how to log some intermediate calculation
        // and use the provided constants
        params->log_data[0] = crazyflie_constants.m * crazyflie_constants.g;
        params->log_data[2] = r_rpdy[0];


        // compute motor outputs


        // write motor output
        memcpy(params->motors, motors, sizeof(motors));

        // sleep 10ms to make this task run at 100Hz
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
