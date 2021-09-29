#include "mqtthandler.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "MQTT";

mqtthandler_config_t mqtthandler_config;

void mqtthandlertask(void *pvParameters) {
	mqtthandler_config = *(mqtthandler_config_t *) pvParameters;
	ESP_LOGI(TAG, "Start");
	float buf_lux;

	while(1) {
		if (xQueueReceive(mqtthandler_config.sensortsl2591->queue_lux, &buf_lux, (TickType_t) 10)) {
			ESP_LOGI(TAG, "From queue %f", buf_lux);
		} else {
			// ESP_LOGI(TAG, "Error receiving from queue_lux");
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

