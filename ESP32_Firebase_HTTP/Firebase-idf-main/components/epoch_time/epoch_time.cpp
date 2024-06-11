#include "epoch_time.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sntp.h"

static const char *TAG = "epoch_time";

void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

void obtain_time(void)
{
    initialize_sntp();

    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry == retry_count) {
        ESP_LOGE(TAG, "Failed to get time from SNTP server");
    }
}

time_t get_epoch_time()
{
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Getting time over NTP.");
        obtain_time();
        time(&now);
    }

    return now;
}

void format_time(time_t epoch_time, char *buffer, size_t buffer_size)
{
    struct tm timeinfo;
    epoch_time += 7 * 3600;
    gmtime_r(&epoch_time, &timeinfo);
    strftime(buffer, buffer_size, "%H:%M:%S UTC %d/%m/%Y", &timeinfo);
}
