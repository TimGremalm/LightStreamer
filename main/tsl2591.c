#include "tsl2591.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "TSL2591";

tsl2591_config_t tsl2591_config;

void tsl2591task(void *pvParameters) {
	tsl2591_config = *(tsl2591_config_t *) pvParameters;
	ESP_LOGI(TAG, "Start");
	vTaskDelay(500 / portTICK_PERIOD_MS);
	while(1) {
		ESP_LOGI(TAG, "Loop");
		vTaskDelay(50000 / portTICK_PERIOD_MS);
	}
}

