#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_private/wifi.h"

// --- إعدادات شبكة Wi-Fi ---
#define WIFI_SSID      "YOUR_WIFI_SSID"      // اسم شبكة الراوتر الرئيسي
#define WIFI_PASS      "YOUR_WIFI_PASSWORD"  // كلمة مرور الراوتر

// --- إعدادات منافذ SPI و GPIO ---
#define PIN_NUM_MISO   19
#define PIN_NUM_MOSI   23
#define PIN_NUM_CLK    18
#define PIN_NUM_CS     5
#define PIN_NUM_HANDSHAKE 4 // خط التنبيه للبوردة الثانية

#define SPI_HOST       VSPI_HOST
#define MAX_ETH_FRAME_SIZE 1536 // الحد الأقصى لحجم إطار الإيثرنت

static const char *TAG = "BRIDGE_MASTER";

// هيكل الحزمة داخل طابور المعالجة
typedef struct {
    uint16_t length;
    uint8_t payload[MAX_ETH_FRAME_SIZE];
} packet_t;

static QueueHandle_t tx_queue = NULL;
static spi_device_handle_t spi_dev;

// --- تهيئة منافذ SPI كـ Master ---
static void spi_master_init(void) {
    esp_err_t ret;

    // تهيئة خط الـ Handshake كـ Output
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_HANDSHAKE),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_NUM_HANDSHAKE, 0);

    // إعدادات ناقل SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_ETH_FRAME_SIZE + 4,
    };

    // إعدادات الجهاز على الناقل
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // سرعة الناقل 10MHz
        .mode = 0,                           // SPI Mode 0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };

    // تفعيل الناقل مع محرك DMA
    ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SPI_HOST, &devcfg, &spi_dev);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI Master initialized successfully.");
}

// --- دالة استدعاء خاطف لاستلام الحزم الخام من شريحة Wi-Fi ---
static esp_err_t wifi_rx_callback(void *buffer, uint16_t len, void *eb) {
    if (len > 0 && len <= MAX_ETH_FRAME_SIZE) {
        packet_t pkt;
        pkt.length = len;
        memcpy(pkt.payload, buffer, len);

        // إرسال الحزمة للطابور دون إعاقة المعالجة المباشرة (Non-blocking)
        if (xQueueSendFromISR(tx_queue, &pkt, NULL) != pdTRUE) {
            ESP_LOGW(TAG, "TX Queue Full! Packet dropped.");
        }
    }

    // تحرير ذاكرة الحزمة في شريحة الواي فاي (إجباري لمنع Memory Leak)
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
}

// --- مهمة إرسال الحزم عبر SPI (SPI Transmission Task) ---
static void spi_tx_task(void *pvParameters) {
    packet_t pkt;
    spi_transaction_t t;

    // ذاكرة مؤقتة مخصصة لـ DMA (DMA Capable RAM)
    uint8_t *dma_tx_buf = heap_caps_malloc(MAX_ETH_FRAME_SIZE + 2, MALLOC_CAP_DMA);

    while (1) {
        // انتظار وجود حزمة في الطابور
        if (xQueueReceive(tx_queue, &pkt, portMAX_DELAY) == pdTRUE) {

            // كتابة الطول في أول 2 بايت ثم الحمولة
            dma_tx_buf[0] = (pkt.length >> 8) & 0xFF;
            dma_tx_buf[1] = pkt.length & 0xFF;
            memcpy(dma_tx_buf + 2, pkt.payload, pkt.length);

            memset(&t, 0, sizeof(t));
            t.length = (pkt.length + 2) * 8; // الطول بالبتات (Bits)
            t.tx_buffer = dma_tx_buf;

            // 1. رفع خط التنبيه لإعلام البوردة الثانية بتجهيز الـ Slave DMA
            gpio_set_level(PIN_NUM_HANDSHAKE, 1);
            vTaskDelay(pdMS_TO_TICKS(1)); // تأخير ميكروي لتأكيد الاستجابة

            // 2. إرسال البيانات عبر SPI
            spi_device_transmit(spi_dev, &t);

            // 3. خفض خط التنبيه بعد إتمام الإرسال
            gpio_set_level(PIN_NUM_HANDSHAKE, 0);
        }
    }

    free(dma_tx_buf);
    vTaskDelete(NULL);
}

// --- إدارة أحداث Wi-Fi ---
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Reconnecting to AP...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Connected to Router successfully!");

        // تفعيل وضع 4-Address (WDS) للشفافية إذا كان الراوتر يدعمه
        esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
        
        // تسجيل دالة الخاطف للالتقاط الخام فور الاتصال
        esp_wifi_internal_reg_rxcb(WIFI_IF_STA, wifi_rx_callback);
        ESP_LOGI(TAG, "Raw RX Callback registered.");
    }
}

// --- تهيئة شبكة Wi-Fi ---
static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // تفعيل وضع Promiscuous الخفيف لالتقاط جميع الحزم الموجهة لغير العنوان الخاص
    esp_wifi_set_promiscuous(true);
}

void app_main(void) {
    // 1. تهيئة الذاكرة غير المتطايرة NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. إنشاء طابور الحزم (يتسع لـ 20 حزمة)
    tx_queue = xQueueCreate(20, sizeof(packet_t));

    // 3. تهيئة SPI Master
    spi_master_init();

    // 4. إنشاء مهمة إرسال SPI على النواة الأولى (Core 1)
    xTaskCreatePinnedToCore(spi_tx_task, "spi_tx_task", 4096, NULL, 5, NULL, 1);

    // 5. تشغيل الواي فاي والاتصال
    wifi_init_sta();
}
