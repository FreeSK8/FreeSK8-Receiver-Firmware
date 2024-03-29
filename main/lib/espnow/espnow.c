/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow.h"

#define ESPNOW_MAGIC 42
#define ESPNOW_RECEIVER 1

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

static uint8_t xbee_ch;
static uint16_t xbee_id;
static uint16_t xbee_remote_address;
static uint16_t xbee_receiver_address;

static xQueueHandle s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic, uint8_t *xbee_ch, uint16_t *xbee_id, uint16_t *xbee_remote_address, uint16_t *xbee_receiver_address)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    *xbee_ch = buf->xbee_ch;
    *xbee_id = buf->xbee_id;
    *xbee_remote_address = buf->xbee_remote_address;
    *xbee_receiver_address = buf->xbee_receiver_address;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    buf->xbee_ch = xbee_ch;
    buf->xbee_id = xbee_id;
    buf->xbee_remote_address = xbee_remote_address;
    buf->xbee_receiver_address = xbee_receiver_address;

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}
static example_espnow_send_param_t *send_param;
static esp_err_t example_espnow_task(void *pvParameter, configure_xbee_func p_configure_xbee)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    uint8_t recv_xbee_ch = 0;
    uint16_t recv_xbee_id = 0;
    uint16_t recv_xbee_remote_address = 0;
    uint16_t recv_xbee_receiver_address = 0;
    bool is_broadcast = false;
    int ret;
#if ESPNOW_RECEIVER
    bool pairing_configuration_received = false;
#endif
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    send_param = (example_espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        return ESP_FAIL;
    }

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                // Limit number of ESPNOW messages
                send_param->count--;
                if (send_param->count == 0) {
                    ESP_LOGI(TAG, "Send attempts exhausted");
                    example_espnow_deinit(send_param);
                    return ESP_FAIL;
                }

                if (is_broadcast && (send_param->broadcast == false)) {
                    ESP_LOGW(TAG, "Nothing to send because send_param->broadcast is false");
                    break;
                }
#if ESPNOW_RECEIVER
                if (send_param->state > PAIRING_STATE_READY) {
                    ESP_LOGW(TAG, "send_param->state is > 3 (ready)");
                }
#else
                /* Remote will wait for state to be READY */
                if (recv_state == PAIRING_STATE_CONNECTED) {
                    ESP_LOGI(TAG, "Waiting for ready from the receiver");
                }
#endif
                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_RATE_MS);
                }

                ESP_LOGI(TAG, "send data (%d) to "MACSTR"", send_param->count, MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                example_espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic, &recv_xbee_ch, &recv_xbee_id, &recv_xbee_remote_address, &recv_xbee_receiver_address);
                free(recv_cb->data);
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Received %d broadcasted State: %d from: "MACSTR", len: %d", recv_seq, recv_state, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            return ESP_FAIL;
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                        ESP_LOGI(TAG, "Peer added from broadcasted data");
                    }

                    /* Change state to connected when we have a peer added */
                    if (send_param->state == PAIRING_STATE_INIT) {
                        ESP_LOGI(TAG, "Changing state to 1 (Connected)");
                        send_param->state = PAIRING_STATE_CONNECTED;
                    }

#if ESPNOW_RECEIVER
                    if (recv_state == PAIRING_STATE_CONNECTED) {
                        ESP_LOGI(TAG, "Start sending unicast data");
                        ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                        /* Start sending unicast ESPNOW data. */
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        example_espnow_data_prepare(send_param);
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            example_espnow_deinit(send_param);
                            return ESP_FAIL;
                        }
                        send_param->broadcast = false;
                        send_param->unicast = true;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "hey renee recv state %d has no handler, mystate %d", recv_state, send_param->state);
                        /* Change state back to init because the other side isn't ready */
                        send_param->state = PAIRING_STATE_INIT;
                        send_param->broadcast = true;
                        send_param->unicast = false;
                    }
#endif
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    ESP_LOGI(TAG, "Receive unicast State %d, XBEE CH 0x%02x ID 0x%04x TX 0x%04x RX 0x%04x, My State %d", recv_state, recv_xbee_ch, recv_xbee_id, recv_xbee_remote_address, recv_xbee_receiver_address, send_param->state);
#if ESPNOW_RECEIVER
                    if (recv_state == PAIRING_STATE_READY && !pairing_configuration_received)
                    {
                        ESP_LOGI(TAG, "Configuring XBEE because remote is ready");
                        pairing_configuration_received = true;

                        // Configure XBEE and respond with PAIRED | FAILED
                        if ((*p_configure_xbee)(recv_xbee_ch, recv_xbee_id, recv_xbee_remote_address, recv_xbee_receiver_address))
                        {
                            ESP_LOGI(TAG, "Changing state to 3 (Paired)");
                            send_param->state = PAIRING_STATE_PAIRED;
                        }
                        else {
                            ESP_LOGI(TAG, "Changing state to 4 (Failed)");
                            send_param->state = PAIRING_STATE_FAILED;
                        }

                        /* Send PAIRED or FAILED response */
                        example_espnow_data_prepare(send_param);
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            example_espnow_deinit(send_param);
                            return ESP_FAIL;
                        }
                    } else if (recv_state == PAIRING_STATE_PAIRED) {
                        //NOTE: The remote knows we are in a successful state
                        ESP_LOGI(TAG, "PAIRED message received, remote knows i'm done!");
                        example_espnow_deinit(send_param);
                        return ESP_OK;
                    }
#else

                    if (recv_state == PAIRING_STATE_CONNECTED && send_param->state == PAIRING_STATE_CONNECTED) {
                        ESP_LOGI(TAG, "Changing state to 2 (Ready)");
                        send_param->state = PAIRING_STATE_READY;
                        ESP_LOGI(TAG, "Start sending unicast data");
                        ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                        /* Start sending unicast ESPNOW data. */
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        example_espnow_data_prepare(send_param);
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            example_espnow_deinit(send_param);
                            return ESP_FAIL;
                        }
                        send_param->broadcast = false;
                        send_param->unicast = true;
                    }
                    else if (recv_state == PAIRING_STATE_PAIRED) {
                        ESP_LOGI(TAG, "Pairing SUCCESSFUL");
                        ESP_LOGI(TAG, "Changing state to 3 (Paired)");
                        send_param->state = PAIRING_STATE_PAIRED;
                        example_espnow_data_prepare(send_param);
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            example_espnow_deinit(send_param);
                            return ESP_FAIL;
                        }
                        vTaskDelay(1000/portTICK_RATE_MS); //NOTE: Waiting for send receive queue
                        example_espnow_cancel();
                        return ESP_OK;
                    }
                    else if (recv_state == PAIRING_STATE_FAILED) {
                        ESP_LOGE(TAG, "Pairing error on receiver side");
                        example_espnow_deinit(send_param);
                        return ESP_FAIL;
                    }
#endif
                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }

    return ESP_FAIL;
}

esp_err_t example_espnow_init(uint8_t p_xbee_ch, uint16_t p_xbee_id, uint16_t p_remote_address, uint16_t p_receiver_address, configure_xbee_func p_configure_xbee)
{

#if ESPNOW_RECEIVER
    //NOTE: This is already initialized on the remote to generate XBEE configuration
    example_wifi_init();
#endif

    xbee_ch = p_xbee_ch;
    xbee_id = p_xbee_id;
    xbee_remote_address = p_remote_address;
    xbee_receiver_address = p_receiver_address;
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = ESPNOW_MAGIC; // Higher number sends ESPNOW data
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    return example_espnow_task(send_param, p_configure_xbee);
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    esp_now_unregister_recv_cb();
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}

void example_espnow_cancel()
{
    // Exhaust all ESPNOW send attempts to cancel pairing
    if(send_param) send_param->count = 1;
}
