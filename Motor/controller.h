#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "main.h"

#define MOTOR_ADDR 1

enum ZG_RETVAL
{ /* zero_guideway 返回值 */
  ZG_OK,
  ZG_ERR_TIMEOUT,
  ZG_ERR_SENSOR
};

uint8_t zero_guideway(void);

#endif
