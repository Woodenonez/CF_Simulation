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

    /* Last tick */
    TickType_t lastWakeTime = xTaskGetTickCount(); 

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

    /* Define user variables */
    float ax, ay, az, gx, gy, gz, roll_acc, pitch_acc;
    float gamma = 0.98;
    float h = 0.01;

    while(1) {
        // read sensor data
        /* Use semaphore */
        xSemaphoreTake(sensors_sem, portMAX_DELAY);
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));
        memcpy(acc_data, params->acc_data, sizeof(acc_data));
        xSemaphoreGive(sensors_sem);

        /* Use user variables */
        ax = acc_data[0];
        ay = acc_data[1];
        az = acc_data[2];
        gx = gyro_data[0];
        gy = gyro_data[1];
        gz = gyro_data[2];

        /* Estimation from acc */
        roll_acc  = (180/M_PI) * atan2(ay, az);
        pitch_acc = (180/M_PI) * atan2(-ax, sqrt(pow(ay,2.0)+pow(az,2.0)));

        /* apply filter */
        estimate[0] = (1-gamma)*roll_acc + gamma*(estimate[0]+h*gx); // roll
        estimate[1] = (1-gamma)*pitch_acc + gamma*(estimate[1]+h*gy); // pitch

        // estimate of the yaw angle provided as an example
        estimate[2] += h * gyro_data[2];

        // example of how to log some intermediate calculation
        params->log_data[1] = h * gyro_data[2];

        // write estimates 
        /* Use semaphore */
        xSemaphoreTake(estimate_sem, portMAX_DELAY);
        memcpy(params->estimate, estimate, sizeof(estimate));
        xSemaphoreGive(estimate_sem);

        // sleep 10ms to make this task run at 100Hz
        vTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS);
    }
}
