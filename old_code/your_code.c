#include "your_code.h"
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
// #include "semphr.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator_kalman.h"
#include "estimator.h"

static bool isInit;

static setpoint_t setpoint;
static sensorData_t sensorData;
static mS state;

const TickType_t xDelay = 2 / portTICK_PERIOD_MS;

xTaskHandle  compTaskHandle;
xTaskHandle  genTaskHandle;
xTaskHandle  CtrlTaskHandle;

SemaphoreHandle_t ctrlGetSensor;
SemaphoreHandle_t ctrlGetSetpoint;

void complementaryFilter(void *pvParameters);
void generateControlInput(void *pvParameters);
void mystateController(void *pvParameters);

void yourCodeInit(void)
{ 
    if (isInit)
    return;

    sensorsInit();
    stateControllerInit();
    powerDistributionInit();
    
    // set up semaphores
    ctrlGetSensor = xSemaphoreCreateMutex();
    ctrlGetSetpoint = xSemaphoreCreateMutex();
    xSemaphoreGive(ctrlGetSensor);
    xSemaphoreGive(ctrlGetSetpoint);
    
    // Create the tasks. 
    xTaskCreate(complementaryFilter, "compfilter",
                configMINIMAL_STACK_SIZE, NULL, 3, &compTaskHandle);
    xTaskCreate(generateControlInput, "genctrlinput",
                configMINIMAL_STACK_SIZE, NULL, 1, &genTaskHandle);
    xTaskCreate(mystateController, "mystatectrl",
                configMINIMAL_STACK_SIZE, NULL, 2, &CtrlTaskHandle);
    isInit = true;
 }
 // COMPLEMENTARY FILTER TASK
void complementaryFilter(void *pvParameters)
{   
    TickType_t xLastWakeTime = xTaskGetTickCount();
	  TickType_t lastWakeTime = xTaskGetTickCount();
    
    // Wait for sensors to be calibrated
    while(!sensorsAreCalibrated()) {
            //vTaskDelay( xDelay );
            vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
        }
    //Wait for the system to be fully started to start stabilization loop
    //systemWaitStart(); // system.c
    while (1){
        sensorsAcquire(&sensorData);
        float ax, ay, az, gx, gy, gz;
        float gamma = 0.99;
        float h = 0.001;
        float square = 2;
        float roll_acc, pitch_acc;

        ax = sensorData.acc.x;
        ay = sensorData.acc.y;
        az = sensorData.acc.z;
        gx = sensorData.gyro.x;
        gy = sensorData.gyro.y;
        gz = sensorData.gyro.z;

        // Accelerometer estimation of angles
        roll_acc = (180 / M_PI) * atan2(ay, az);
        pitch_acc = (180 /M_PI) * atan2(-ax, sqrt(pow(ay, square) + pow(az, square)));

        xSemaphoreTake(ctrlGetSensor, portMAX_DELAY);

        // Write estimation to state after discretization with Euler Backwards 
        state.roll = (1 - gamma) * roll_acc + gamma*(state.roll + h * gx);
        state.pitch = (1 - gamma) * pitch_acc + gamma*(state.pitch + h * gy);
        state.rollRate = gx;
        state.pitchRate = gy;
        state.yawRate = gz;

        xSemaphoreGive(ctrlGetSensor);
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
    vTaskDelete(NULL);
  
}

// GENERATE CONTROL INPUT TASK
void generateControlInput(void *pvParameters)
{
    while (1){     
        xSemaphoreTake(ctrlGetSetpoint, portMAX_DELAY);
        commanderGetSetpoint(&setpoint);
        xSemaphoreGive(ctrlGetSetpoint);
        vTaskDelay(xDelay/2);
    }
    vTaskDelete(NULL);
}

// LQR CONTROLLER TASK
void mystateController(void *pvParameters)
{   
    // Values from LQR gain matrix K. Each column has same values with different signs, therefore only 6 values
    float k1 = 0.620775545289811; //0.007516895441346;
    float k2 = 0.629365966448514; //0.007517994487586;
    float k3 = 0.001026759552748; //0.000032287334864;
    float k4 = 0.088411714284182; //0.008849024514257;
    float k5 = 0.130920506412828; //0.013096728360578;
    float k6 = 0.000206377805846; //0.000020743492175;
    float kt = 7765.234070515980; // Motor thrust conversion
    while (1){
        
        xSemaphoreTake(ctrlGetSensor, portMAX_DELAY);
        xSemaphoreTake(ctrlGetSetpoint, portMAX_DELAY);
        // Compute error
        float yr = setpoint.attitude.roll - state.roll; //
        float yrR = setpoint.attitudeRate.roll - state.rollRate; // 
        float yp = setpoint.attitude.pitch - state.pitch; //
        float ypR = setpoint.attitudeRate.pitch - state.pitchRate; //
        float yy = 0; // Set to 0 as out of scope ourError->yaw;
        float yyR = setpoint.attitudeRate.yaw - state.yawRate;// 
        xSemaphoreGive(ctrlGetSensor);
        xSemaphoreGive(ctrlGetSetpoint); 

        uint16_t value1, value2, value3, value4;

        // Compute control values with LQR gain K
        float u1 = -k1*yr -k2*yp -k3*yy -k4*yrR -k5*ypR -k6*yyR;
        float u2 = -k1*yr + k2*yp + k3*yy -k4*yrR + k5*ypR + k6*yyR;
        float u3 = k1*yr + k2*yp - k3*yy + k4*yrR + k5*ypR -k6*yyR;
        float u4 = k1*yr -k2*yp + k3*yy + k4*yrR -k5*ypR + k6*yyR;
        value1 = kt * u1 +  setpoint.thrust;
        value2 = kt * u2 +  setpoint.thrust;
        value3 = kt * u3 +  setpoint.thrust;
        value4 = kt * u4 +  setpoint.thrust;

      // Clamp-function to make sure that all the values for the input is within 0-65535. 
        if(value1<0){
          value1=0;
        }
        if(value1>65535){
          value1=65535;
        }

        if(value2<0){
          value2=0;
        }
        if(value2>65535){
          value2=65535;
        }
        if(value3<0){
          value3=0;
        }
        if(value3>65535){
          value3=65535;
        }
        if(value4<0){
          value4=0;
        }
        if(value4>65535){
          value4=65535;
        }
        motorsSetRatio(MOTOR_M1, value1);
        motorsSetRatio(MOTOR_M2, value2);
        motorsSetRatio(MOTOR_M3, value3);
        motorsSetRatio(MOTOR_M4, value4);
        vTaskDelay(xDelay/2);
    }
    vTaskDelete(NULL);

}






