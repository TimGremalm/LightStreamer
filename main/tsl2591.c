#include "tsl2591.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "TSL2591";

#define I2C_MASTER_SCL_IO			CONFIG_I2C_MASTER_SCL	/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO			CONFIG_I2C_MASTER_SDA	/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM				0						/*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ			400000					/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE	0						/*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE	0						/*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS		1000

#define TSL2591_ADDRESS				0x29					/*!< Sensor default address is 0x29 */

tsl2591_config_t tsl2591_config;

static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

uint8_t register_read8(uint8_t reg_addr) {
	uint8_t buf[1];
	//register_read(reg_addr, &buf, sizeof(buf));
	i2c_master_write_read_device(I2C_MASTER_NUM, TSL2591_ADDRESS, &reg_addr, 1, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
	return buf[1];
}

uint16_t register_read16(uint8_t reg_addr) {
	uint8_t buf[2];
	//register_read(reg_addr, &buf, sizeof(buf));
	i2c_master_write_read_device(I2C_MASTER_NUM, TSL2591_ADDRESS, &reg_addr, 1, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
	uint16_t out = (buf[1] << 8) + buf[0];
	return out;
}

static esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, TSL2591_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    return ret;
}

void enable_sensor() {
	/* Enables the chip, so it's ready to take readings.
	Enable the device by setting the control bit to 0x01 */
	register_write_byte(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
	TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN | TSL2591_ENABLE_NPIEN);
	tsl2591_config.state = TSL2591_Enabled;
}

void disable_sensor() {
	/* Disables the chip, so it's in power down mode.
	Disable the device by setting the control bit to 0x00 */
	register_write_byte(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
	TSL2591_ENABLE_POWEROFF);
	tsl2591_config.state = TSL2591_Disabled;
}

uint8_t getStatus(void) {
	/* Gets the most recent sensor event from the hardware status register.
	return Sensor status as a byte. Bit 0 is ALS Valid. Bit 4 is ALS Interrupt. */
	enable_sensor();
	uint8_t data = register_read8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_STATUS);
	disable_sensor();
	return data;
}

void setTiming() {
	register_write_byte(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL,
	tsl2591_config.integration | tsl2591_config.gain);
}

void setGain() {
	register_write_byte(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL,
	tsl2591_config.integration | tsl2591_config.gain);
}

float calculateLux(uint16_t ch0, uint16_t ch1) {
	float atime, again;
	float cpl, lux;

	// Check for overflow conditions first
	if ((ch0 == 0xFFFF) | (ch1 == 0xFFFF)) {
		// Signal an overflow
		return -1;
	}

	// Note: This algorithm is based on preliminary coefficients
	// provided by AMS and may need to be updated in the future

	switch (tsl2591_config.integration) {
		case TSL2591_INTEGRATIONTIME_100MS:
			atime = 100.0F;
			break;
		case TSL2591_INTEGRATIONTIME_200MS:
			atime = 200.0F;
			break;
		case TSL2591_INTEGRATIONTIME_300MS:
			atime = 300.0F;
			break;
		case TSL2591_INTEGRATIONTIME_400MS:
			atime = 400.0F;
			break;
		case TSL2591_INTEGRATIONTIME_500MS:
			atime = 500.0F;
			break;
		case TSL2591_INTEGRATIONTIME_600MS:
			atime = 600.0F;
			break;
		default: // 100ms
			atime = 100.0F;
			break;
	}

	switch (tsl2591_config.gain) {
		case TSL2591_GAIN_LOW:
			again = 1.0F;
			break;
		case TSL2591_GAIN_MED:
			again = 25.0F;
			break;
		case TSL2591_GAIN_HIGH:
			again = 428.0F;
			break;
		case TSL2591_GAIN_MAX:
			again = 9876.0F;
			break;
		default:
			again = 1.0F;
			break;
	}

	// cpl = (ATIME * AGAIN) / DF
	cpl = (atime * again) / TSL2591_LUX_DF;

	// Original lux calculation (for reference sake)
	// lux1 = ( (float)ch0 - (TSL2591_LUX_COEFB * (float)ch1) ) / cpl;
	// lux2 = ( ( TSL2591_LUX_COEFC * (float)ch0 ) - ( TSL2591_LUX_COEFD *
	// (float)ch1 ) ) / cpl; lux = lux1 > lux2 ? lux1 : lux2;

	// Alternate lux calculation 1
	// See: https://github.com/adafruit/Adafruit_TSL2591_Library/issues/14
	lux = (((float)ch0 - (float)ch1)) * (1.0F - ((float)ch1 / (float)ch0)) / cpl;

	// Alternate lux calculation 2
	// lux = ( (float)ch0 - ( 1.7F * (float)ch1 ) ) / cpl;

	// Signal I2C had no errors
	return lux;
}

void readLight(void) {
	enable_sensor();
	// Wait x ms for ADC to complete
	for (uint8_t d = 0; d <= tsl2591_config.integration; d++) {
		vTaskDelay(120 / portTICK_PERIOD_MS);
	}

	// CHAN0 must be read before CHAN1
	// See: https://forums.adafruit.com/viewtopic.php?f=19&t=124176
	tsl2591_config.light_broadband = register_read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);
	tsl2591_config.light_infrared = register_read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);
	tsl2591_config.light_visible = tsl2591_config.light_broadband - tsl2591_config.light_infrared;
	tsl2591_config.light_lux = calculateLux(tsl2591_config.light_broadband, tsl2591_config.light_infrared);
	disable_sensor();
}

void init_sensor() {
	enable_sensor();
	setTiming();
	setGain();
	disable_sensor();
}

void tsl2591task(void *pvParameters) {
	tsl2591_config = *(tsl2591_config_t *) pvParameters;
	ESP_LOGI(TAG, "Start");
	tsl2591_config.state = TSL2591_Starting;
	vTaskDelay(500 / portTICK_PERIOD_MS);

	tsl2591_config.state = TSL2591_InitI2c;
	ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

	tsl2591_config.state = TSL2591_InitSensor;
	init_sensor();
	
	while(1) {
		readLight();
		// Only print values every 20'th value to reduce print
		if ((esp_random() % 60) == 0) {
			ESP_LOGI(TAG, "Sampled raw values (Reduced print) Broadband: %d Visible: %d Infrared: %d, Calculated Lux %f",
					tsl2591_config.light_broadband, tsl2591_config.light_visible, tsl2591_config.light_infrared, tsl2591_config.light_lux);
		}
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
}

