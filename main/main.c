#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"


#include "driver/gpio.h"
#include "driver/uart.h"

#include "lib/espnow/espnow.h"

#include "driver/mcpwm.h"

//#define ENABLE_DEBUG

const char * version = "0.3.0";

#define GPIO_OUTPUT_LED 2  //LED
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_OUTPUT_LED)

#define ESC_TXD 3 //Send to ESC
#define ESC_RXD 1
#define ESC_BAUD 115200
#define ESC_UART_PORT_NUM 2
#define ESC_BUF_SIZE (1024)

#define XBEE_TXD 16
#define XBEE_RXD 17
#define XBEE_BAUD 115200
#define XBEE_UART_PORT_NUM 1
#define XBEE_BUF_SIZE (1024)

#define GPS_TXD 27 //Send to GPS
#define GPS_RXD 26
#define GPIO_PPM_TOGGLE (1ULL<<GPS_TXD)
#define GPIO_PPM_OUTPUT (1ULL<<GPS_RXD)

static bool receiver_in_ppm_mode = false;

static bool receiver_in_pairing_mode = true;
static bool xbee_in_configuration = false;

/* PWM */
TickType_t pwm_last_updated;
#define PWM_MIN_PULSEWIDTH_US (1000) // Minimum pulse width in microsecond
#define PWM_MAX_PULSEWIDTH_US (2000) // Maximum pulse width in microsecond
#define PWM_TIMEOUT_MS (900)
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	if (x < in_min) x = in_min;
	if (x > in_max) x = in_max;
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* PWM */

/* VESC */
#include "crc.h"
#include "packet.h"
#include "buffer.h"
#include "datatypes.h"

#define PACKET_VESC						0

static int packet_send_payload(uint8_t * payload, int lenPay) {

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = '\0';


	// Sending package
	uart_write_bytes(ESC_UART_PORT_NUM, messageSend, count);

	// Returns number of send bytes
	return count;
}

static void uart_send_buffer(unsigned char *data, unsigned int len) {
	uart_write_bytes(ESC_UART_PORT_NUM, data, len);
}

void process_packet_vesc(unsigned char *data, unsigned int len) {
    // Intercept CHUCK_DATA and generate PWM signal
	if (data[0] == COMM_SET_CHUCK_DATA)
	{
        pwm_last_updated = xTaskGetTickCount();
        uint8_t joystick_value = data[2];
        long chuck_joy_microseconds = map(joystick_value, 0, 255, PWM_MIN_PULSEWIDTH_US, PWM_MAX_PULSEWIDTH_US);
        // Set PPM output value
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, chuck_joy_microseconds));
#ifdef ENABLE_DEBUG
        ESP_LOGI(__FUNCTION__,"RECEIVED CHUCK DATA IN PPM MODE");
#endif
	} else {
		// Pass data to ESC
        packet_send_payload(data, len);
	}
}
/* VESC */

static void gpio_task(void *arg)
{
    uint16_t delay = 500;
    while(1)
    {
        if (receiver_in_pairing_mode) delay = 100;
        else delay = 1000;
        /// Blink LED
        gpio_set_level(GPIO_OUTPUT_LED, 1);
        vTaskDelay(delay/portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_LED, 0);
        vTaskDelay(delay/portTICK_PERIOD_MS);
    }
}

static void esc_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ESC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ESC_UART_PORT_NUM, ESC_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ESC_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ESC_UART_PORT_NUM, ESC_TXD, ESC_RXD, 0, 0));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(ESC_BUF_SIZE);

    TickType_t esc_last_responded = xTaskGetTickCount();
    bool esc_pins_swapped = false;
    while (1) {
        // Read data from the ESC
        int len = uart_read_bytes(ESC_UART_PORT_NUM, data, ESC_BUF_SIZE, 20 / portTICK_RATE_MS);

        if (len) {
            uart_write_bytes(XBEE_UART_PORT_NUM, data, len);
            esc_last_responded = xTaskGetTickCount();
        }

        if ((xTaskGetTickCount() - esc_last_responded) * portTICK_RATE_MS > 1000) {
#ifdef ENABLE_DEBUG
            ESP_LOGW(__FUNCTION__,"ESC has not responded for 1000ms. Swapping TX/RX (%d)", !esc_pins_swapped);
#endif
            esc_pins_swapped = !esc_pins_swapped;
            if (esc_pins_swapped) {
                // Swap ESC TX and RX
                ESP_ERROR_CHECK(uart_set_pin(ESC_UART_PORT_NUM, ESC_RXD, ESC_TXD, 0, 0));
            } else {
                // Set default ESC TX/RX configuration
                ESP_ERROR_CHECK(uart_set_pin(ESC_UART_PORT_NUM, ESC_TXD, ESC_RXD, 0, 0));
            }
            esc_last_responded = xTaskGetTickCount(); // Reset ESC reponded time
        }

		vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

static void xbee_send_string(unsigned char *data) {
	unsigned int len = strlen((char*)data);
	uart_write_bytes(XBEE_UART_PORT_NUM, data, len);
}
static bool xbee_wait_ok(uint8_t *data, bool is_fatal)
{
	TickType_t startTick = xTaskGetTickCount();
	TickType_t endTick, diffTick;
	bool receivedOK = false;
	bool receivedO = false;
	bool receivedK = false;
	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	while(!receivedOK && diffTick*portTICK_RATE_MS < 1500)
	{
		int length = uart_read_bytes(XBEE_UART_PORT_NUM, data, XBEE_BUF_SIZE, 20 / portTICK_RATE_MS);
		if (length) {
			for (int i = 0; i < length; i++) {
				if (data[i] == 0x4F) receivedO = true;
				if (data[i] == 0x4B) receivedK = true;
			}
		}
		if (receivedO && receivedK) receivedOK = true;

		endTick = xTaskGetTickCount();
		diffTick = endTick - startTick;
	}
	if (is_fatal && !receivedOK)
	{
		ESP_LOGE(__FUNCTION__,"XBEE did not OK! Receiver Haulting");
		while(1) {
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
	}
	return receivedOK;
}
bool xbee_configure(uint8_t p_xbee_ch, uint16_t p_xbee_id, uint16_t p_xbee_remote_address, uint16_t p_xbee_receiver_address)
{
#ifdef ENABLE_DEBUG
    ESP_LOGI(__FUNCTION__,"Configuring XBEE");
#endif
    uint8_t data[16] = {0};

    xbee_in_configuration = true;

    vTaskDelay(1000/portTICK_PERIOD_MS);
    xbee_send_string((unsigned char *)"+++");
    //TODO: Check for OK message from XBEE at all baud rates
    if (!xbee_wait_ok(data, false)) {
        //TODO: What do we do if the xbee doesn't respond??????
#ifdef ENABLE_DEBUG
        ESP_LOGE(__FUNCTION__, "XBEE Did not respond");
#endif
        while(1) {
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
    } else {
#ifdef ENABLE_DEBUG
        ESP_LOGI(__FUNCTION__, "XBEE OK");
#endif
    }
#ifdef ENABLE_DEBUG
    ESP_LOGI(__FUNCTION__,"XBEE READY");
#endif

    bool configuration_success = true;
    unsigned char write_data[10] = {0};
    sprintf((char*)write_data, "ATCH%02x\r", p_xbee_ch); // Network Channel
    xbee_send_string(write_data);
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    sprintf((char*)write_data, "ATID%04x\r", p_xbee_id); // Network ID
    xbee_send_string(write_data);
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    xbee_send_string((unsigned char*)"ATDH0\r"); // Destination High is 0
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    sprintf((char*)write_data, "ATDL%04x\r", p_xbee_remote_address); // Destination Low
	xbee_send_string(write_data);
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    sprintf((char*)write_data, "ATMY%04x\r", p_xbee_receiver_address); // My Address
	xbee_send_string(write_data);
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    xbee_send_string((unsigned char*)"ATBD7\r"); // Baud 115200
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    xbee_send_string((unsigned char*)"ATD70\r"); // Digital IO7 is Disabled
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    xbee_send_string((unsigned char*)"ATWR\r"); // Write configuration
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    xbee_send_string((unsigned char*)"ATCN\r"); // Exit Command mode
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    if (configuration_success) {
#ifdef ENABLE_DEBUG
        ESP_LOGI(__FUNCTION__, "XBEE Configuration Successful");
#endif
    } else {
#ifdef ENABLE_DEBUG
        ESP_LOGE(__FUNCTION__, "XBEE Configuration failed");
#endif
        while(1) {
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }
#ifdef ENABLE_DEBUG
    printf("XBEE Configured\n");
#endif
    xbee_in_configuration = false;

    return true;
}
static void xbee_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = XBEE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(XBEE_UART_PORT_NUM, XBEE_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(XBEE_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(XBEE_UART_PORT_NUM, XBEE_TXD, XBEE_RXD, 0, 0));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(XBEE_BUF_SIZE);

    if (receiver_in_ppm_mode) {
        packet_init(uart_send_buffer, process_packet_vesc, PACKET_VESC);
    }

    while (1) {
        if (xbee_in_configuration) {
            // Do not consume data here if the XBEE is being configured
            vTaskDelay(10/portTICK_PERIOD_MS);
            continue;
        }
        // Read data from the XBEE
        int len = uart_read_bytes(XBEE_UART_PORT_NUM, data, XBEE_BUF_SIZE, 20 / portTICK_RATE_MS);

        if (len) {
            // Cancel pairing mode if the XBEE is receiving from the remote
            if (receiver_in_pairing_mode) {
#ifdef ENABLE_DEBUG
                ESP_LOGI(__FUNCTION__, "Pairing mode deactivated by XBEE communication");
#endif
                receiver_in_pairing_mode = false;
                // Shut down ESPNOW and WiFi
                example_espnow_cancel();
                esp_wifi_stop();
            }

            gpio_set_level(GPIO_OUTPUT_LED, 1);
#ifdef ENABLE_DEBUG
            printf("XBEE Read %d bytes\n", len);
            printf("ESC unavailable while debugging\n");
#else
            if (receiver_in_ppm_mode) {
                // Process remote communication
                for (int i = 0;i < len;i++) {
					packet_process_byte(data[i], PACKET_VESC);
				}
            } else {
                // Pass data directly to ESC via UART
                uart_write_bytes(ESC_UART_PORT_NUM, data, len);
            }
#endif
        }

		vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void check_ppm_mode(void)
{
    gpio_config_t io_conf;

    // Initialize GPIO
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_PPM_TOGGLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    receiver_in_ppm_mode = !gpio_get_level(GPS_TXD);

    if (receiver_in_ppm_mode) {
#ifdef ENABLE_DEBUG
        ESP_LOGI(__FUNCTION__,"RECEIVER IN PPM MODE");
#endif
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = GPIO_PPM_OUTPUT;
        io_conf.pull_down_en = 1;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        // Init MCPWM generator
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPS_RXD);

        mcpwm_config_t pwm_config = {
            .frequency = 100, // Hz
            .cmpr_a = 0,     // duty cycle of PWMxA = 0
            .counter_mode = MCPWM_UP_COUNTER,
            .duty_mode = MCPWM_DUTY_MODE_0,
        };
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    }
}

void app_main(void)
{
#ifdef ENABLE_DEBUG
    ESP_LOGI(__FUNCTION__, "Starting FreeSK8 Receiver v%s", version);
#endif
    check_ppm_mode();

   	gpio_config_t io_conf;

	// Initialize GPIO
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Initialize NVS for ESPNOW
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // GPIO task
	xTaskCreate(gpio_task, "gpio_task", 1024, NULL, 10, NULL);

#ifdef ENABLE_DEBUG
    ESP_LOGI(__FUNCTION__, "Debugging is enabled. ESC comms are unavailable");
#else
    // ESC task
	xTaskCreate(esc_task, "esc_task", 1024 * 2, NULL, 10, NULL);
#endif

    // XBEE task
	xTaskCreate(xbee_task, "xbee_task", 1024 * 4, NULL, 10, NULL);

    vTaskDelay(1000 / portTICK_PERIOD_MS); //NOTE: Wait 1 second for paired device to communicate and cancel pairing
    if (receiver_in_pairing_mode) //NOTE: This is only true at boot
    {
        example_espnow_init(0x0, 0x0, 0x0, 0x0, &xbee_configure);
        receiver_in_pairing_mode = false; //NOTE: Only allowing the pairing process to take place once
        esp_wifi_stop(); // Turn off wifi to save power after pairing
    }

    while (1) {
        // Delay, but not too long. This adds to the potential PWM_TIMEOUT_MS
        vTaskDelay(100 / portTICK_PERIOD_MS);

        if (receiver_in_ppm_mode) {
            // Check if the remote signal has timed out
            if ((xTaskGetTickCount() - pwm_last_updated) * portTICK_RATE_MS > PWM_TIMEOUT_MS) {
                // Cancel PWM signal generation
                ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0));
#ifdef ENABLE_DEBUG
                ESP_LOGW(__FUNCTION__, "Signal lost while in PPM mode");
#endif
            }
        }
    }
}
