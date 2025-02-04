#include "FreeRTOS.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wifihelper.h"
#include "lwip/dns.h"
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include "pico/stdlib.h"

#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "task.h"

#include "mqttthing.h"
#include "rf_pico.h"
#include "protocol.h"

#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

const char *WIFISSID = WIFI_SSID;
const char *WIFIPASSWORD = WIFI_PASSWORD;
const char *MQTTHOST = MQTT_HOST;
const int MQTTPORT = MQTT_PORT;
const char *MQTTUSER = MQTT_USER;
const char *MQTTPASSWD = MQTT_PASSWD;
const char *TOPICROOT = "SENSOR";
const char *TOPICTEMPERATURE = "TEMPERATURE";

#define MAX_SENSORS 10
#define UPDATE_INTERVAL 5000 //ms

uint32_t sensors[MAX_SENSORS];

typedef struct
{
    uint8_t sensorId;
    float value;
} queue_entry_t;

queue_t message_queue;

bool needs_update(uint8_t sensorId)
{
    if (sensorId > MAX_SENSORS)
    {
        return false;
    }
    uint32_t current_timestamp = to_ms_since_boot(get_absolute_time());
    if (current_timestamp - sensors[sensorId-1] > UPDATE_INTERVAL)
    {
        sensors[sensorId-1] = current_timestamp;
        return true;
    }
    else
    {
        return false;
    }
}

void report_result(RF_Message* message)
{
    uint8_t sensorId = get_device_address(&message->message);
    if (needs_update(sensorId))
    {
        queue_entry_t entry;
        entry.sensorId = sensorId;
        entry.value = get_temperature(&message->message);
        queue_try_add(&message_queue, &entry);
    }
}

void main_task(void *params)
{
    MQTTThing thing;

     // Setup for MQTT Connection
    char mqttTarget[] = MQTT_HOST;
    int mqttPort = MQTT_PORT;
    char mqttClient[] = MQTT_CLIENT;
    char mqttUser[] = MQTT_USER;
    char mqttPwd[] = MQTT_PASSWD;

    if (!mqttthing_init(&thing, WIFISSID, WIFIPASSWORD, MQTTHOST, MQTTPORT, MQTTUSER, MQTTPASSWD))
    {
        LogError(("Failed to initialize MQTTThing\n"));
        return;
    }

    mqttthing_connectLoop(&thing);

    char topicBuffer[40];
    char payloadBuffer[10];
    queue_entry_t entry = {0};
    while (true)
    {
   
        if (queue_try_remove(&message_queue, &entry))
        {
            sprintf(topicBuffer, "%s/%s/%d", TOPICROOT, TOPICTEMPERATURE, entry.sensorId);
            sprintf(payloadBuffer, "%3.2f", entry.value);
            printf("Publishing to %s, payload %s\n", topicBuffer, payloadBuffer);
            mqttthing_publish(&thing, topicBuffer, payloadBuffer);
        }
        vTaskDelay(pdTICKS_TO_MS(1000U));
        
    }
}

void vLaunch(void)
{
    TaskHandle_t task;

    xTaskCreate(main_task, "MainThread", 512, NULL, TASK_PRIORITY, &task);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

void core1_main(void)
{
    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;
    rtos_name = "FreeRTOS";
    printf("Starting %s on core 1:\n", rtos_name);
    sleep_ms(1000);

    vLaunch();
}

int main(void)
{
    timer_hw->dbgpause = 0; // hack!
    stdio_init_all();

    sleep_ms(1000);

    printf("Init...\n");
    
    queue_init(&message_queue, sizeof(queue_entry_t), 20);
    memset(sensors, 0, sizeof(sensors));    

    multicore_launch_core1(core1_main);

    printf("Waiting...");
    sleep_ms(20000);

    rf_pico_receiver rec;
    pico_init_receiver(&rec, report_result);
    sleep_ms(1000);
    pico_rx_start_receiving(&rec);


    while (1)
    {
       sleep_ms(1000);
    }
    return 0;
}