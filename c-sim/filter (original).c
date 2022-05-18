#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "filter.h"
#include "constants.h"

void filterTask(void *pvParameters) {
    struct FilterParams *params =
        (struct FilterParams*)pvParameters;

    // we keep local copies of the global state + semaphores
    double gyro_data[3];
    double acc_data[3];

    // copy the semaphore handles for convenience
    SemaphoreHandle_t sensors_sem = params->sensors_sem;
    SemaphoreHandle_t estimate_sem = params->estimate_sem;

    // local internal state.
    double estimate[3] = {0.0};

    while(1) {
        // read sensor data
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));
        memcpy(acc_data, params->acc_data, sizeof(acc_data));

        // apply filter

        // estimate of the yaw angle provided as an example
        estimate[2] += 0.01 * gyro_data[2];

        // example of how to log some intermediate calculation
        params->log_data[1] = 0.01 * gyro_data[2];

        // write estimates output
        memcpy(params->estimate, estimate, sizeof(estimate));

        // sleep 10ms to make this task run at 100Hz
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
