#include "mqtthandler.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <string.h>

#include "mqtt_client.h"

static const char *TAG = "MQTT";

#define NUM_RELAYS 2
#define GPIO_REALY1 13
#define GPIO_REALY2 15
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_REALY1) | (1ULL<<GPIO_REALY2))
uint8_t relays[NUM_RELAYS] = {GPIO_REALY1, GPIO_REALY2};

mqtthandler_config_t mqtthandler_config;
uint8_t mac[6];
char macstring[13];
char topic_lux[100];
char topic_relays_get[NUM_RELAYS][100];
char topic_relays_set[NUM_RELAYS][100];
bool timer_relay_enabled[NUM_RELAYS];
uint32_t timer_relay_start[NUM_RELAYS];
uint16_t timer_relay_goal[NUM_RELAYS];

static void log_error_if_nonzero(const char *message, int error_code) {
	if (error_code != 0) {
		ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
	}
}

int atoin(const char* str, int len) {
	// Like atoi, but will only parse up to length
	int i;
	int ret = 0;
	int ret_factor = 1;
	for(i = 0; i < len; ++i) {
		if (str[i] == '-' && i == 0) {
			ret_factor = -1;
		} else if (str[i] >= '0' && str[i] <= '9') {
			ret = ret * 10 + (str[i] - '0');
		} else {
			return ret * ret_factor;
		}
	}
	return ret * ret_factor;
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
	esp_mqtt_event_handle_t event = event_data;
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	int i = 0;
	switch ((esp_mqtt_event_id_t)event_id) {
		case MQTT_EVENT_CONNECTED:
			ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

			for (i=0; i < NUM_RELAYS; i++) {
				msg_id = esp_mqtt_client_subscribe(client, topic_relays_set[i], 0);
				ESP_LOGI(TAG, "sent subscribe to %s successful, msg_id=%d", topic_relays_set[i], msg_id);
			}
			break;
		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
			break;
		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_DATA:
			ESP_LOGI(TAG, "MQTT_EVENT_DATA");
			uint8_t relay_number = 0;
			for (i=0; i < NUM_RELAYS; i++) {
				if (strncmp(event->topic, topic_relays_set[i], event->topic_len) == 0) {
					relay_number = i;
				}
			}
			if (event->data_len > 0 && event->data_len <= 7) {
				// Only try to parse data up to 7 cahracters
				int parsed_num = atoin(event->data, event->data_len);
				if (parsed_num >= 0 && parsed_num <= 65535) {
					// Only accept a range of 0-65535
					if (parsed_num == 0) {
						gpio_set_level(relays[relay_number], 0);
						// Also disable any timer
						timer_relay_enabled[relay_number] = false;
					} else {
						gpio_set_level(relays[relay_number], 1);
						// Enable timeout in ms if num is more than 1
						if (parsed_num > 1) {
							timer_relay_enabled[relay_number] = true;
							timer_relay_start[relay_number] = xTaskGetTickCount() * portTICK_PERIOD_MS;
							timer_relay_goal[relay_number] = parsed_num;
							ESP_LOGI(TAG, "Start timer for relay: %d start: %d", relay_number, timer_relay_start[i]);
						}
					}
					ESP_LOGI(TAG, "Relay %u num %d", relay_number, parsed_num);
				} else {
					ESP_LOGE(TAG, "Out of range, allowed 0-65535");
					printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
					printf("DATA=%.*s\r\n", event->data_len, event->data);
				}
			} else {
				ESP_LOGE(TAG, "Invalid relay number or data");
				printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
				printf("DATA=%.*s\r\n", event->data_len, event->data);
			}
			break;
		case MQTT_EVENT_ERROR:
			ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
			if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
				log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
				log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
				log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
				ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
			}
			break;
		default:
			ESP_LOGI(TAG, "Other event id:%d", event->event_id);
			break;
	}
}

void mqtthandlertask(void *pvParameters) {
	mqtthandler_config = *(mqtthandler_config_t *) pvParameters;
	ESP_LOGI(TAG, "Start");
	int i = 0;

	// Configure GPIO for relays
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	for (i=0; i < NUM_RELAYS; i++) {
		timer_relay_enabled[i] = false;
	}

	// Configure topics based on MAC address
	esp_read_mac(mac, ESP_MAC_WIFI_STA);
	sprintf(macstring, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	ESP_LOGI(TAG, "Mac: %s", macstring);
	// Lux topic
	sprintf(topic_lux, "/%s/tsl2591/lux", macstring);
	ESP_LOGI(TAG, "Topic Lux: %s", topic_lux);
	// Relay topics
	for (i=0; i < NUM_RELAYS; i++) {
		sprintf(topic_relays_get[i], "/%s/relay/%d/get", macstring, i);
		ESP_LOGI(TAG, "Topic relay get %d: %s", i, topic_relays_get[i]);
		sprintf(topic_relays_set[i], "/%s/relay/%d/set", macstring, i);
		ESP_LOGI(TAG, "Topic relay set %d: %s", i, topic_relays_set[i]);
	}

	// Connect to MQTT broker
	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = CONFIG_BROKER_URL,
	};
	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
	esp_mqtt_client_start(client);

	float buf_lux;
	while(1) {
		if (xQueueReceive(mqtthandler_config.sensortsl2591->queue_lux, &buf_lux, (TickType_t) 10)) {
			// ESP_LOGI(TAG, "From queue %f", buf_lux);
			char buf_message[100];
			sprintf(buf_message, "%f", buf_lux);
			esp_mqtt_client_publish(client, topic_lux, buf_message, 0, 1, 0);
			for (i=0; i < NUM_RELAYS; i++) {
				sprintf(buf_message, "%d", gpio_get_level(relays[i]));
				esp_mqtt_client_publish(client, topic_relays_get[i], buf_message, 0, 1, 0);
			}
		} else {
			// ESP_LOGI(TAG, "Error receiving from queue_lux");
		}

		uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
		uint32_t delta = 0;
		for (i=0; i < NUM_RELAYS; i++) {
			if (timer_relay_enabled[i] == true) {
				delta = now - timer_relay_start[i];
				// ESP_LOGI(TAG, "Timer %d: %d %d", i, timer_relay_start[i], delta);
				if (delta > timer_relay_goal[i]) {
					ESP_LOGI(TAG, "Timer has passed, turn off relay: %d start: %d delta: %d", i, timer_relay_start[i], delta);
					gpio_set_level(relays[i], 0);
					timer_relay_enabled[i] = false;
				}
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

