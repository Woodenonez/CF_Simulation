#ifndef YOUR_CODE_H_
#define YOUR_CODE_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "sensors.h"
#include "commander.h"
#include "motors.h"

#include "stabilizer.h"

#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"


#define RATE_1000_HZ 1000
#define RATE_MAIN_LOOP RATE_1000_HZ

typedef struct myState {
  float roll;
  float pitch;
  float yaw;
  float rollRate;
  float pitchRate;
  float yawRate;
} mS;

typedef struct myControl {
  float control_1;
  float control_2;
  float control_3;
  float control_4;
} mC;

void yourCodeInit(void);

#endif /* YOUR_CODE_H_ */
