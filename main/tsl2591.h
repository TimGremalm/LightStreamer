#ifndef TSL2591_H
#define TSL2591_H

#include <stdint.h>

typedef struct {
	uint8_t state;
} tsl2591_config_t;

void tsl2591task(void *pvParameters);

#endif /* TSL2591_H */

