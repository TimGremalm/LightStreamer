#ifndef MQTTHANDLER_H
#define MQTTHANDLER_H

#include <stdint.h>
#include "tsl2591.h"

typedef struct {
	tsl2591_config_t *sensortsl2591;
} mqtthandler_config_t;

void mqtthandlertask(void *pvParameters);

#endif /* MQTTHANDLER_H */

