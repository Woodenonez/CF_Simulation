#include "your_code.h"

/***
 *
 * This file is where you should add you tasks. You already know the structure
 * Required to do so from the work with the simulator.
 *
 * The function yourCodeInit() is set to automatically execute when the
 * quadrotor is started. This is where you need to create your tasks. The
 * scheduler that runs the tasks is already up and running so you should
 * NOT make a call to vTaskStartScheduler();.
 *
 * Below that you can find a few examples of useful function calls and code snippets.
 *
 * For further reference on how this is done: Look into the file stabilizer.c
 * which usually handles the control of the crazyflie.
 *
 ***/

// NO NEED TO CHANGE THIS
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;
static StateEstimatorType estimatorType;
// END NO NEED TO CHANGE THIS

// *** Own implementation ***
static bool isInit;
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static mS mystate;

const TickType_t xDelay = 2 / portTICK_PERIOD_MS;

STATIC_MEM_TASK_ALLOC(complementaryFilter, configMINIMAL_STACK_SIZE);
STATIC_MEM_TASK_ALLOC(generateReference, configMINIMAL_STACK_SIZE);
STATIC_MEM_TASK_ALLOC(stateController, configMINIMAL_STACK_SIZE);

static void complementaryFilter(void* param);
static void generateReference(void* param);
static void stateController(void* param);

SemaphoreHandle_t ctrlGetSensor;
SemaphoreHandle_t ctrlGetSetpoint;
// *** Own implementation ***

void yourCodeInit(void)
{
    estimatorType = getStateEstimator();
	
	/*
   * CREATE AND EXECUTE YOUR TASKS FROM HERE
   */

  // *** Own implementation ***
  if (isInit)
    return;
  
  sensorsInit();
  controllerInit(ControllerTypeAny);
  powerDistributionInit();

  // set up semaphores
  ctrlGetSensor = xSemaphoreCreateMutex();
  ctrlGetSetpoint = xSemaphoreCreateMutex();
  xSemaphoreGive(ctrlGetSensor);
  xSemaphoreGive(ctrlGetSetpoint);

  STATIC_MEM_TASK_CREATE(complementaryFilter, complementaryFilter, "complementaryFilter", NULL, 3);
  STATIC_MEM_TASK_CREATE(generateReference, generateReference, "generateReference", NULL, 1);
  STATIC_MEM_TASK_CREATE(stateController, stateController, "stateController", NULL, 2);

  isInit = true;
  // *** Own implementation ***
}

/* 
* ADD TASKS HERE
*/
// *** Own implementation ***
static void complementaryFilter(void* param){
  uint32_t tick = 1;
  uint32_t lastWakeTime = xTaskGetTickCount ();

  while(!sensorsAreCalibrated()) {
      vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  while(1){
    sensorsAcquire(&sensorData, tick);
    float ax, ay, az, gx, gy, gz, roll_acc, pitch_acc;
    float gamma = 0.98;
    float h = 0.001;

    ax = sensorData.acc.x;
    ay = sensorData.acc.y;
    az = sensorData.acc.z;
    gx = sensorData.gyro.x;
    gy = sensorData.gyro.y;
    gz = sensorData.gyro.z;

    roll_acc = (180 / M_PI) * atan2(ay, az);
    pitch_acc = (180 /M_PI) * atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2)));
  
    xSemaphoreTake(ctrlGetSensor, portMAX_DELAY);

    // Write estimation to state after discretization with Euler Backwards 
    mystate.roll  = (1 - gamma) * roll_acc  + gamma*(mystate.roll  + h*gx);
    mystate.pitch = (1 - gamma) * pitch_acc + gamma*(mystate.pitch + h*gy);
    mystate.rollRate = gx;
    mystate.pitchRate = gy;
    mystate.yawRate = gz;

    xSemaphoreGive(ctrlGetSensor);
    vTaskDelayUntil(&lastWakeTime, xDelay);
  }
  vTaskDelete(NULL);
}

void generateReference(void *param)
{
    while (1){     
        xSemaphoreTake(ctrlGetSetpoint, portMAX_DELAY);
        commanderGetSetpoint(&setpoint, &state);
        xSemaphoreGive(ctrlGetSetpoint);
        vTaskDelay(xDelay/2);
    }
    vTaskDelete(NULL);
}

void stateController(void *param)
{   
    // Values from LQR gain matrix K. Each column has same values with different signs, therefore only 6 values
    float k1 = 200.77971421860937;
    float k2 = 263.151411793972;
    float k3 = 60.285823952791439;
    float k4 = 79.404284035754685;
    float k5 = 47.321399813063628;
    // float base = 29491.2;
    float base = 0.0;
    
    uint16_t m1, m2, m3, m4;
    while (1){
        
        xSemaphoreTake(ctrlGetSensor, portMAX_DELAY);
        xSemaphoreTake(ctrlGetSetpoint, portMAX_DELAY);
        // Compute error
        float yr = setpoint.attitude.roll - mystate.roll;
        float yrR = setpoint.attitudeRate.roll - mystate.rollRate;
        float yp = setpoint.attitude.pitch - mystate.pitch;
        float ypR = setpoint.attitudeRate.pitch - mystate.pitchRate;
        float yyR = setpoint.attitudeRate.yaw - mystate.yawRate;
        xSemaphoreGive(ctrlGetSensor);
        xSemaphoreGive(ctrlGetSetpoint); 

        // Compute control values with LQR gain K
        float u1 = -k1*yr - k2*yp - k3*yrR - k4*ypR - k5*yyR;
        float u2 = -k1*yr + k2*yp - k3*yrR + k4*ypR + k5*yyR;
        float u3 =  k1*yr + k2*yp + k3*yrR + k4*ypR - k5*yyR;
        float u4 =  k1*yr - k2*yp + k3*yrR - k4*ypR + k5*yyR;
        m1 = u1 + setpoint.thrust + base;
        m2 = u2 + setpoint.thrust + base;
        m3 = u3 + setpoint.thrust + base;
        m4 = u4 + setpoint.thrust + base;

        // Clamp-function to make sure that all the values for the input is within 0-65535. 
        if (m1<0) m1 = 0; else if (m1>65535) m1 = 65535;
        if (m2<0) m2 = 0; else if (m2>65535) m2 = 65535;
        if (m3<0) m3 = 0; else if (m3>65535) m3 = 65535;
        if (m4<0) m4 = 0; else if (m4>65535) m4 = 65535;

        motorsSetRatio(MOTOR_M1, m1);
        motorsSetRatio(MOTOR_M2, m2);
        motorsSetRatio(MOTOR_M3, m3);
        motorsSetRatio(MOTOR_M4, m4);
        vTaskDelay(xDelay/2);
    }
    vTaskDelete(NULL);

}
// *** Own implementation ***

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/*************************************************
 * CREATE A TASK (Example, a task with priority 5)
 ************************************************/
 //STATIC_MEM_TASK_ALLOC(yourTaskName, configMINIMAL_STACK_SIZE);
 //STATIC_MEM_TASK_CREATE(yourTaskName, yourTaskName,
 //   "TASK_IDENTIFIER_NAME", NULL, 5);

/*************************************************
 * WAIT FOR SENSORS TO BE CALIBRATED
 ************************************************/
// lastWakeTime = xTaskGetTickCount ();
// while(!sensorsAreCalibrated()) {
//     vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
// }



/*************************************************
 * RETRIEVE THE MOST RECENT SENSOR DATA
 *
 * The code creates a variable called sensorData and then calls a function
 * that fills this variable with the latest data from the sensors.
 *
 * sensorData_t sensorData = struct {
 *     Axis3f acc;
 *     Axis3f gyro;
 *     Axis3f mag;
 *     baro_t baro;
 *     zDistance_t zrange;
 *     point_t position;
 * }
 *
 * Before starting the loop, initialize tick:
 * uint32_t tick;
 * tick = 1;
 ************************************************/
// sensorData_t sensorData;
// sensorsAcquire(&sensorData, tick);



/*************************************************
 * RETRIEVE THE SET POINT FROM ANY EXTERNAL COMMAND INTERFACE
 *
 * The code creates a variable called setpoint and then calls a function
 * that fills this variable with the latest command input.
 *
 * setpoint_t setpoint = struct {
 *     uint32_t timestamp;
 *
 *     attitude_t attitude;      // deg
 *     attitude_t attitudeRate;  // deg/s
 *     quaternion_t attitudeQuaternion;
 *     float thrust;
 *     point_t position;         // m
 *     velocity_t velocity;      // m/s
 *     acc_t acceleration;       // m/s^2
 *     bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame
 *
 *     struct {
 *         stab_mode_t x;
 *         stab_mode_t y;
 *         stab_mode_t z;
 *         stab_mode_t roll;
 *         stab_mode_t pitch;
 *         stab_mode_t yaw;
 *         stab_mode_t quat;
 *     } mode;
 * }
 *
 ************************************************/
// state_t state;
// setpoint_t setpoint;
// stateEstimator(&state, tick);
// commanderGetSetpoint(&setpoint, &state);



/*************************************************
 * SENDING OUTPUT TO THE MOTORS
 *
 * The code sends an output to each motor. The output should have the be
 * of the typ unsigned 16-bit integer, i.e. use variables such as:
 * uint16_t value_i
 *
 ************************************************/
// motorsSetRatio(MOTOR_M1, value_1);
// motorsSetRatio(MOTOR_M2, value_2);
// motorsSetRatio(MOTOR_M3, value_3);
// motorsSetRatio(MOTOR_M4, value_4);


/*************************************************
 * LOGGING VALUES THAT CAN BE PLOTTEN IN PYTHON CLIENT
 *
 * We have already set up three log blocks to for the accelerometer data, the
 * gyro data and the setpoints, just uncomment the block to start logging. Use
 * them as reference if you want to add custom blocks.
 *
 ************************************************/

/*
LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)
*/

/*
LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)
*/

/*
LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)
*/
