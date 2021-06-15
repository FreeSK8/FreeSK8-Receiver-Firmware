#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/uart.h"

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

static void gpio_task(void *arg)
{
    while(1)
    {
        /// Blink LED
        gpio_set_level(GPIO_OUTPUT_LED, 1);
        vTaskDelay(500/portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_LED, 0);
        vTaskDelay(500/portTICK_PERIOD_MS);
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
            gpio_set_level(GPIO_OUTPUT_LED, 0);
            uart_write_bytes(XBEE_UART_PORT_NUM, data, len);
        }

		vTaskDelay(10/portTICK_PERIOD_MS);
    }
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
        // Read data from the XBEE
        int len = uart_read_bytes(XBEE_UART_PORT_NUM, data, XBEE_BUF_SIZE, 20 / portTICK_RATE_MS);

        if (len) {
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

	/// OUTPUTS
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    // GPIO task
	xTaskCreate(gpio_task, "gpio_task", 1024, NULL, 10, NULL);

#ifdef ENABLE_DEBUG
    ESP_LOGI(TAG, "Debugging is enabled. ESC comms are unavailable");
#else
    // ESC task
	xTaskCreate(esc_task, "esc_task", 1024 * 2, NULL, 10, NULL);
#endif

    // XBEE task
	xTaskCreate(xbee_task, "xbee_task", 1024 * 2, NULL, 10, NULL);

    int i = 0;
    while (1) {
        printf("[%d] Stayin' Alive!\n", i);
        i++;
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
