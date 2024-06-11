/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "epoch_time.h"
#include "fan.h"

static const char *TAG = "mqtt_example";

#define WIFI_SSID "d"
#define WIFI_PASSWORD "21521929"

#define GPIO_PWM0A_OUT 26   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 27   //Set GPIO 16 as PWM0B

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_PULSE_GPIO             GPIO_NUM_16        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms


mcpwm_cmpr_handle_t comparator;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

int retry_num=0;
char data[127];
char door_control_topic[50] = "esp32/doorCtrl";
char fan_control_topic[50] = "esp32/fanCtrl";
char light_control_topic[50] = "esp32/lightCtrl";
char deviceState_topic[50] = "esp32/StateDv232";
char light_topic[50] = "esp32/light";
char deviceName[20] = "DevicesNode-232";
int lightState = -1;
int doorState = -1;
int fanState = -1;
int qos = 1;
char timeTurnOn[6];
int timerFlag = 0;
int isTurnedOn = 0;


static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data){
if(event_id == WIFI_EVENT_STA_START)
{
  printf("WIFI CONNECTING....\n");
}
else if (event_id == WIFI_EVENT_STA_CONNECTED)
{
  printf("WiFi CONNECTED\n");
}
else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
{
  printf("WiFi lost connection\n");
  if(retry_num<5){esp_wifi_connect();retry_num++;printf("Retrying to Connect...\n");}
  else
  {

  }
}
else if (event_id == IP_EVENT_STA_GOT_IP)
{
  printf("Wifi got IP...\n\n");
}
}

void wifi_connection()
{
     //                          s1.4
    // 2 - Wi-Fi Configuration Phase
    esp_netif_init();
    esp_event_loop_create_default();     // event loop                    s1.2
    esp_netif_create_default_wifi_sta(); // WiFi station                      s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); //     
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    //esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD
            }
        };
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
    printf( "wifi_init_softap finished. SSID:%s  password:%s",WIFI_SSID,WIFI_PASSWORD);
    
}

void servo_ctrl_init()
{
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

esp_mqtt_event_handle_t event;
esp_mqtt_client_handle_t client;
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    event = event_data;
    client = event->client;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, door_control_topic, qos);
        esp_mqtt_client_subscribe(client, fan_control_topic, qos);
        esp_mqtt_client_subscribe(client, light_control_topic, qos);
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
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        cJSON *root = cJSON_Parse(event->data);
        if (root == NULL) {
            printf("Can not parse JSON.\n");
        }
        esp_err_t ert;
        memset(data, '\0', sizeof(data));
        if(strcmp(event->topic, door_control_topic) == 0)
        {
            cJSON *dstate = cJSON_GetObjectItemCaseSensitive(root, "state");
            if (cJSON_IsBool(dstate)) {
                if(cJSON_IsTrue(dstate))
                {
                    doorState = 1;
                    ert = mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(90));
                }
                else
                {
                    doorState = 0;
                    ert = mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
                }
                if(ert != ESP_OK)
                {
                    doorState = -1;
                }
            }
            printf("Door state: %d\n", doorState);
            char temp[3];
            itoa(doorState, temp, 10);
            sprintf(data, "device_name: %s, door:%s", deviceName, temp);
            
        }
        else if(strcmp(event->topic, light_control_topic) == 0)
        {
            cJSON *timerState = cJSON_GetObjectItemCaseSensitive(root, "isSetTime");
            cJSON *timerValue = cJSON_GetObjectItemCaseSensitive(root, "timeTurnOn");
            sprintf(timeTurnOn, "%s", timerValue->valuestring);
            printf("timeturnon: %s\n", timeTurnOn);
            if (cJSON_IsBool(timerState)) {
                if(cJSON_IsTrue(timerState))
                {
                    timerFlag = 1;
                }
                else
                {
                    timerFlag = 0;
                }
            }
            cJSON *lstate = cJSON_GetObjectItemCaseSensitive(root, "state");
            if (cJSON_IsBool(lstate)) {
                if(cJSON_IsTrue(lstate))
                {
                    lightState = 1;
                    ert = gpio_set_level(GPIO_NUM_2, 1);
                }
                else
                {
                    lightState = 0;
                    ert = gpio_set_level(GPIO_NUM_2, 0);
                }
                if(ert != ESP_OK)
                {
                    lightState = -1;
                }
            }
            printf("Led state: %d\n", lightState);
            char temp[3];
            itoa(lightState, temp, 10);
            sprintf(data, "device_name: %s, light: %s", deviceName, temp);
        }
        else if(strcmp(event->topic, fan_control_topic) == 0)
        {
            cJSON *fstate = cJSON_GetObjectItemCaseSensitive(root, "level");
            fanState = atoi(fstate->valuestring);
            switch (fanState)
            {
            case FAN_LEVEL_0:
                motor_set_speed(0);
                break;
            case FAN_LEVEL_1:
                motor_set_speed(3000);
                break;
            case FAN_LEVEL_2:
                motor_set_speed(4000);
                break;
            case FAN_LEVEL_3:
                motor_set_speed(6000);
                break;
            default:
                break;
            }
            printf("Fan state: %d\n", fanState);
            char temp[3];
            itoa(fanState, temp, 10);
            sprintf(data, "device_name: %s, fan: %s", deviceName, temp);
        }
        esp_mqtt_client_publish(client, deviceState_topic, data, sizeof(data), qos, 0);
        cJSON_Delete(root);
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

static void mqtt_app_start(void)
{
    // esp_mqtt_client_config_t mqtt_cfg = {
    //     .broker.address.uri = "mqtt://mqtt.flespi.io",
    //     .credentials.username = "oWLsmINUN4kOw7FTJ8DDzuu24lS5aYUvsxqbJu6A8VgG3aoJ6OZC8GmTQw3LG4At",
    //     .credentials.authentication.password = "",
    //     .credentials.client_id = "esp3201",
    //     .broker.address.port = 1883,
    // };

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.emqx.io",
        .credentials.username = "esp32",
        .credentials.authentication.password = "d123456",
        .credentials.client_id = "esp3202",

    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void timer_app(void *pvParameter)
{
    esp_err_t err;
    while (1)
    {
        if(timerFlag == 1)
        {
            char time_str[64];
            format_time(get_epoch_time(), time_str, sizeof(time_str));
            char timer[6];
            for (int i = 0; i < 5; i++)
                timer[i] = time_str[i];
            timer[5] = '\0';
            if(strcmp(timer, timeTurnOn) == 0)
            {
                if(isTurnedOn == 0)
                {
                    char temp[8] = "state: true";
                    lightState = 1;
                    isTurnedOn = 1;
                    err = gpio_set_level(GPIO_NUM_2, 1);
                    if(err != ESP_OK)
                    {
                        lightState = -1;
                    }
                    printf("Led state: %d\n", lightState);
                    esp_mqtt_client_publish(client, light_topic, temp, sizeof(temp), qos, 0);
                    memset(data, '\0', sizeof(data));
                    sprintf(data, "device_name: %s, light: %d", deviceName, lightState);
                    esp_mqtt_client_publish(client, light_control_topic, data, sizeof(data), qos, 0);
                } 
            }
            else
            {
                isTurnedOn = 0;
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    motor_init(); // Khởi tạo driver điều khiển động cơ
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    wifi_connection();
    mqtt_app_start();
    servo_ctrl_init();
    
    //xTaskCreate( &timer_app, "timer_task", 2048, NULL, 5, NULL );
}
