#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "controller.h"
#include "constants.h"

static double gyro_data[3];
static double r_rpdy[3];
static double estimate[3] = {0.0};
static unsigned short motors[4];

static SemaphoreHandle_t ctrl_read_sem;
static SemaphoreHandle_t ctrl_write_sem;


void controlInputTask(void *pvParameters) {

    TickType_t lastWakeTime = xTaskGetTickCount(); 

    struct ControlSystemParams *params = (struct ControlSystemParams*)pvParameters;

    SemaphoreHandle_t sensors_sem = params->sensors_sem;
    SemaphoreHandle_t estimate_sem = params->estimate_sem;
    SemaphoreHandle_t references_sem = params->references_sem;

    while(1) {
        // read sensor data (gyro)
        xSemaphoreTake(ctrl_read_sem, portMAX_DELAY);
        xSemaphoreTake(sensors_sem, portMAX_DELAY);
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));
        xSemaphoreGive(sensors_sem);

        // read filter data (angle estimates)
        xSemaphoreTake(estimate_sem, portMAX_DELAY);
        memcpy(estimate, params->estimate, sizeof(estimate));
        xSemaphoreGive(estimate_sem);

        // read latest references 
        xSemaphoreTake(references_sem, portMAX_DELAY);
        memcpy(r_rpdy, params->r_rpdy, sizeof(r_rpdy));
        xSemaphoreGive(references_sem);
        xSemaphoreGive(ctrl_read_sem);

        vTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS);
    }
}


void controlOutputTask(void *pvParameters) {

    TickType_t lastWakeTime = xTaskGetTickCount(); 

    struct ControlSystemParams *params = (struct ControlSystemParams*)pvParameters;

    SemaphoreHandle_t motors_sem = params->motors_sem;

    while(1) {
        xSemaphoreTake(ctrl_write_sem, portMAX_DELAY);
        xSemaphoreTake(motors_sem, portMAX_DELAY);
        memcpy(params->motors, motors, sizeof(motors));
        xSemaphoreGive(motors_sem);
        xSemaphoreGive(ctrl_write_sem);

        vTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS);
    }
}


void controlSystemTask(void *pvParameters) {

    TickType_t lastWakeTime = xTaskGetTickCount(); 
    
    struct ControlSystemParams *params =
        (struct ControlSystemParams*)pvParameters;

    ctrl_read_sem = xSemaphoreCreateBinary(); xSemaphoreGive(ctrl_read_sem);
    ctrl_write_sem = xSemaphoreCreateBinary(); xSemaphoreGive(ctrl_write_sem);

    xTaskCreate(controlInputTask, "Control Input Task",
                configMINIMAL_STACK_SIZE, (void*)params, 5, NULL);
    xTaskCreate(controlOutputTask, "Control Output Task",
                configMINIMAL_STACK_SIZE, (void*)params, 3, NULL);

    /* Controller parameters */
    float k1 = 135.0867;
    float k2 = 140.5737;
    float k3 = 4.9062;
    float k4 = 19.1707;
    float k5 = 21.9781;
    float k6 = 16.8891;

    while(1) {

        /* compute error */
        xSemaphoreTake(ctrl_read_sem, portMAX_DELAY);
        float er  = r_rpdy[0] - estimate[0];
        float edr = 0         - gyro_data[0];
        float ep  = r_rpdy[1] - estimate[1];
        float edp = 0         - gyro_data[1];
        float ey  = 0         - estimate[2];
        float edy = r_rpdy[2] - gyro_data[2];
        xSemaphoreGive(ctrl_read_sem);

        /* compute motor outputs */
        float base = 29491.2;
        float u1 = -k1*er -k2*ep -k3*ey -k4*edr -k5*edp -k6*edy + base;
        float u2 = -k1*er +k2*ep +k3*ey -k4*edr +k5*edp +k6*edy + base;
        float u3 =  k1*er +k2*ep -k3*ey +k4*edr +k5*edp -k6*edy + base;
        float u4 =  k1*er -k2*ep +k3*ey +k4*edr -k5*edp +k6*edy + base;

        /* Clamp-func */
        if (u1<0) u1 = 0; else if (u1>65535) u1 = 65535;
        if (u2<0) u2 = 0; else if (u2>65535) u2 = 65535;
        if (u3<0) u3 = 0; else if (u3>65535) u3 = 65535;
        if (u4<0) u4 = 0; else if (u4>65535) u4 = 65535;

        xSemaphoreTake(ctrl_write_sem, portMAX_DELAY);
        motors[0] = (int)u1;
        motors[1] = (int)u2;
        motors[2] = (int)u3;
        motors[3] = (int)u4;
        xSemaphoreGive(ctrl_write_sem);

        vTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS);
    }
}
