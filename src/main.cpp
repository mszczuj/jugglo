#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_eap_client.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"
#include <netdb.h>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

#include "imu_bmi160.hpp"

#if __has_include("wifi_secret.h")
#include "wifi_secret.h"
#else
#warning "wifi_secret.h not found - using default WiFi credentials & master host/port"

#define WIFI_SSID "juglo_ap"
#define WIFI_PASS "jugl0mast4r"

#define MASTER_HOST "hostname.local"
#define MASTER_PORT 50555
#endif

#define I2C_SCL GPIO_NUM_20
#define I2C_SDA GPIO_NUM_19

#define I2C_FREQ_HZ 400000 // 400 kHz; higher speeds were flaky when WiFi traffic preempts I2C ISR
static i2c_master_bus_handle_t i2c_bus = nullptr;

#define DEFAULT_SCAN_LIST_SIZE 5

#define LED_GPIO GPIO_NUM_15   // ESP32-C6 DevKit LED

#define TAG "app"

static void init_i2c(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));
    ESP_LOGI(IMUTAG, "I2C bus ready on SDA=%d SCL=%d @ %d Hz", I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
}


static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
#define WIFI_MAX_RETRY 10

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            s_retry_num++;
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = { 0 };
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    // WPA2-PSK: ustaw minimalny poziom auth; dla typowego WPA2/WPA3-mixed działa bez certyfikatów
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP: %s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Failed to connect to AP: %s", WIFI_SSID);
    }
}


int sock = -1;

static void udp_client_connect(void)
{
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_DGRAM,
    };
    struct addrinfo *res;

    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%d", TCP_PORT);

    int err = getaddrinfo(TCP_HOST, port_str, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGE(TAG, "DNS lookup failed: %d", err);
        return;
    }

    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create UDP socket: errno %d", errno);
        freeaddrinfo(res);
        return;
    }

    // For UDP, no connect() is strictly needed, but you can use connect() to set default peer address
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "UDP socket connect failed: errno %d", errno);
        close(sock);
        freeaddrinfo(res);
        return;
    }

    ESP_LOGI(TAG, "UDP socket ready to %s:%d", TCP_HOST, TCP_PORT);

    freeaddrinfo(res);
}

void send_datagram(const void* data, size_t len) {
    if (sock < 0) {
        ESP_LOGW(TAG, "Socket not connected, cannot send data");
        return;
    }

    ssize_t bytes_sent = send(sock, data, len, MSG_DONTWAIT);
    if (bytes_sent < 0) {
        ESP_LOGE(TAG, "Failed to send data: errno %d", errno);
    } else if ((size_t)bytes_sent < len) {
        ESP_LOGW(TAG, "Partial data sent: %d of %d bytes", bytes_sent, len);
    }
}

struct __attribute__((packed)) ImuDatagram {
    char magic[4]; // "IMUD"
    uint32_t timestamp_ms;     //  
    uint16_t sequence_number;  // 
    uint8_t  sensor_id;        // 
    int16_t  accel_gyro[6];    // ax,ay,az, gx,gy,gz (12 bytes)
    uint16_t checksum;         // 16-bit ones' complement over all preceding bytes
};


static inline uint16_t imu_checksum16(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t sum = 0;

    while (len >= 2) {
        uint16_t word = (uint16_t)((p[0] << 8) | p[1]);
        sum += word;
        p += 2;
        len -= 2;
    }

    if (len == 1) {
        uint16_t word = (uint16_t)(p[0] << 8);
        sum += word;
    }

    while (sum >> 16) {
        sum = (sum & 0xFFFFu) + (sum >> 16);
    }

    return (uint16_t)(~sum & 0xFFFFu);
}

static inline void imu_datagram_update_checksum(ImuDatagram& d) {
    d.checksum = 0;
    d.checksum = imu_checksum16(&(d.timestamp_ms), sizeof(ImuDatagram) - sizeof(d.magic) - sizeof(d.checksum));
}

static inline bool imu_datagram_verify(const ImuDatagram& d) {
    return d.checksum == imu_checksum16(&(d.timestamp_ms), sizeof(ImuDatagram) - sizeof(d.magic) - sizeof(d.checksum));
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Serial connection established");

    init_i2c();
    IMUBMI160 imu;

    ESP_ERROR_CHECK( imu.initialize(i2c_bus) );

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);  // LED ON

    wifi_init_sta();    
    udp_client_connect();

    ImuDatagram datagram = {
        {'I', 'M', 'U', 'D'}, // magic 
        0,                    // timestamp_ms
        0,                    // sequence_number
        2,                    // sensor_id
        {0, 0, 0, 0, 0, 0},   // accel_gyro
        0                     // checksum
    }; // size = 24 bytes

    int16_t accel_gyro_buf[6];

    size_t batch_size = 10;
    ImuDatagram dgram_batch[batch_size];

    for (size_t i = 0; i < batch_size; ++i)
        memcpy(&dgram_batch[i], &datagram, sizeof(ImuDatagram));
    
    uint16_t seq_num = 0;
    while (1) {

        vTaskDelay(pdMS_TO_TICKS(20)); // that gives imu time to fill fifo (with 8-9 samples), while we  wifi driver is sending data
        int samples_count = imu.update();
        // ESP_LOGI(TAG, "Read %d samples", samples_count);
        for (uint8_t sample_no=0; sample_no < samples_count && sample_no < batch_size; sample_no++) {            
            // copy imu data to datagram
            memcpy((void*)dgram_batch[sample_no].accel_gyro, (const void*)&imu.imu_buffer[sample_no * 6], sizeof(accel_gyro_buf));

            // that will be timestamp of the end of the batch
            dgram_batch[sample_no].timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
            dgram_batch[sample_no].sequence_number = seq_num++;        
            imu_datagram_update_checksum(dgram_batch[sample_no]);            
        }
        send_datagram(&dgram_batch, sizeof(datagram) * samples_count);

        // vTaskDelay(pdMS_TO_TICKS(10));

        
        // fwrite(&dgram_batch, sizeof(dgram_batch), 1, stdout);
        // fflush(stdout);
        // for (size_t i = 0; i < batch_size; i++) {
        //     // imu.waitAndUpdate();
        //     // if (i!= 0) 
        //     vTaskDelay(pdMS_TO_TICKS(10));
        //     int values_count = imu.update();
        //     // imu.getAccel(accel_gyro_buf);
        //     // imu.getGyro(accel_gyro_buf + 3);

        //     // memcpy((void*)dgram_batch[i].accel_gyro, (const void*)accel_gyro_buf, sizeof(accel_gyro_buf));

        //     dgram_batch[i].timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
        //     dgram_batch[i].sequence_number = seq_num++;
            
        //     imu_datagram_update_checksum(dgram_batch[i]);

        //     ESP_LOGI(TAG, "Read %d samples", values_count);

        // }

        // vTaskDelay(pdMS_TO_TICKS(10));
        // int samples_count = imu.update();
        // for (uint8_t sample_no=0; sample_no < samples_count && sample_no < batch_size; sample_no++) {            
        //     // copy imu data to datagram

        //     memcpy((void*)dgram_batch[sample_no].accel_gyro, (const void*)&imu.imu_buffer[sample_no * 6], sizeof(accel_gyro_buf));

        //     // that will be timestamp of the end of the batch
        //     dgram_batch[sample_no].timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
        //     dgram_batch[sample_no].sequence_number = seq_num++;
            
        //     imu_datagram_update_checksum(dgram_batch[sample_no]);            
        // }
        
        // fwrite(&dgram_batch, sizeof(dgram_batch), 1, stdout);
        // fflush(stdout);
    
    
    }
}
