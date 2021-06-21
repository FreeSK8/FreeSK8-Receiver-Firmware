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

//#define ENABLE_DEBUG

static const char *TAG = "Receiver";

#define GPIO_OUTPUT_LED 2  //LED
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_OUTPUT_LED)

#define ESC_TXD 1
#define ESC_RXD 3
#define ESC_BAUD 115200
#define ESC_UART_PORT_NUM 2
#define ESC_BUF_SIZE (1024)

#define XBEE_TXD 16
#define XBEE_RXD 17
#define XBEE_BAUD 115200
#define XBEE_UART_PORT_NUM 1
#define XBEE_BUF_SIZE (1024)

static bool receiver_in_pairing_mode = true;
static bool xbee_in_configuration = false;

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

    while (1) {
        // Read data from the ESC
        int len = uart_read_bytes(ESC_UART_PORT_NUM, data, ESC_BUF_SIZE, 20 / portTICK_RATE_MS);

        if (len) {
            printf("ESC Read %d bytes\n", len);
            uart_write_bytes(XBEE_UART_PORT_NUM, data, len);
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
		ESP_LOGE(__FUNCTION__,"XBEE did not OK! Haulting");
		while(1) {
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
	}
	return receivedOK;
}
bool xbee_configure(uint8_t p_xbee_ch, uint16_t p_xbee_id)
{
    ESP_LOGI(__FUNCTION__,"Configuring XBEE");
    uint8_t data[16] = {0};

    xbee_in_configuration = true;

    vTaskDelay(1000/portTICK_PERIOD_MS);
    xbee_send_string((unsigned char *)"+++");
    //TODO: Check for OK message from XBEE at all baud rates
    if (!xbee_wait_ok(data, false)) {
        //TODO: What do we do if the xbee doesn't respond??????
        ESP_LOGE(__FUNCTION__, "XBEE Did not respond");
        while(1) {
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
    } else {
        ESP_LOGI(__FUNCTION__, "XBEE OK");
    }
    ESP_LOGI(__FUNCTION__,"XBEE READY");

    bool configuration_success = true;
    unsigned char write_data[10] = {0};
    sprintf((char*)write_data, "ATCH%02x\r", p_xbee_ch);
    xbee_send_string(write_data);
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    sprintf((char*)write_data, "ATID%04x\r", p_xbee_id);
    xbee_send_string(write_data);
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    xbee_send_string((unsigned char*)"ATDH0\r"); // Destination High is 0
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    xbee_send_string((unsigned char*)"ATDL1\r"); // Destination Low is #1
    if (configuration_success) configuration_success = xbee_wait_ok(data, false);

    xbee_send_string((unsigned char*)"ATMY2\r"); // My Address is #2
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
        ESP_LOGI(__FUNCTION__, "XBEE Configuration Successful");
    } else {
        ESP_LOGE(__FUNCTION__, "XBEE Configuration failed");
        while(1) {
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }

    printf("XBEE Configured\n");

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
                ESP_LOGI(__FUNCTION__, "Pairing mode deactivated by XBEE communication");
                receiver_in_pairing_mode = false;
                //TODO: example_espnow_cancel(); causes laggy response to input but requires debugger
            }
            printf("XBEE Read %d bytes\n", len);
            gpio_set_level(GPIO_OUTPUT_LED, 1);
#ifdef ENABLE_DEBUG
            printf("ESC unavailable while debugging");
#else
            uart_write_bytes(ESC_UART_PORT_NUM, data, len);
#endif
        }

		vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
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
    ESP_LOGI(TAG, "Debugging is enabled. ESC comms are unavailable");
#else
    // ESC task
	xTaskCreate(esc_task, "esc_task", 1024 * 2, NULL, 10, NULL);
#endif

    // XBEE task
	xTaskCreate(xbee_task, "xbee_task", 1024 * 4, NULL, 10, NULL);

    vTaskDelay(1000 / portTICK_PERIOD_MS); //NOTE: Wait 1 second for paired device to communicate and cancel pairing
    if (receiver_in_pairing_mode) //NOTE: This is only true at boot
    {
        example_espnow_init(0x0, 0x0, &xbee_configure);
        receiver_in_pairing_mode = false; //NOTE: Only allowing the pairing process to take place once
    }

    int i = 0;
    while (1) {
        printf("[%d] Stayin' Alive!\n", i);
        i++;
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
