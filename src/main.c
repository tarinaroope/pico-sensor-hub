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

#define POWER_LED_INDICATOR_GPIO 20

#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define MAX_SENSORS 10
#define UPDATE_INTERVAL 5000 //ms
#define ROOT_TOPIC "SENSOR"
#define TEMPERATURE_TOPIC "TEMPERATURE"

uint32_t sensors[MAX_SENSORS];

typedef struct
{
    uint8_t sensorId;
    float value;
} queue_entry_t;

queue_t message_queue;
queue_t status_queue;

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

// pmicro-rf callback in case of incoming sensor information
void report_result(RF_Message* message)
{
    // Get the sensor where the message is from
    uint8_t sensorId = get_device_address(&message->message);
    if (needs_update(sensorId))
    {
        queue_entry_t entry;
        entry.sensorId = sensorId;
        // Now we only get temperature
        entry.value = get_temperature(&message->message);
        queue_try_add(&message_queue, &entry);
    }
}

// Mqttthing callback for online status
void connectCallback(bool online)
{
    if (queue_is_full(&status_queue))
    {
        queue_try_remove(&status_queue, NULL);
    }
    queue_try_add(&status_queue, &online);
}

void main_task(void *params)
{
    MQTTThing thing;

    // Create buffers
    char* topicBuffer = pvPortMalloc(strlen(TEMPERATURE_TOPIC) + 5);
    char* payloadBuffer = pvPortMalloc(8);
    if (topicBuffer == NULL || payloadBuffer == NULL ||
        (!mqttthing_init(&thing, WIFI_SSID, WIFI_PASSWORD, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASSWD, ROOT_TOPIC)) )
    {
        vPortFree(topicBuffer);
        vPortFree(payloadBuffer);
        LogError(("Failed to initialize/allocate memory!\n"));
        return;
    }

    mqttthing_connectLoop(&thing, connectCallback);

    queue_entry_t entry = {0};

    while (true)
    {
        while(queue_try_remove(&message_queue, &entry))
        {
            sprintf(topicBuffer, "%d/%s", entry.sensorId, TEMPERATURE_TOPIC );
            sprintf(payloadBuffer, "%3.2f", entry.value);
            printf("Publishing to %s, payload %s\n", topicBuffer, payloadBuffer);
            mqttthing_publish(&thing, topicBuffer, payloadBuffer);
        }
        vTaskDelay(pdTICKS_TO_MS(2000U));
    }

    vPortFree(topicBuffer);
    vPortFree(payloadBuffer);
}

void vLaunch(void)
{
    xTaskCreate(main_task, "MainThread", 512, NULL, TASK_PRIORITY, NULL);
    vTaskStartScheduler();
}

void core1_main(void)
{
    printf("Starting FreeRTOS on core %d...\n", get_core_num());
    sleep_ms(1000);
    vLaunch();
}

int main(void)
{
    timer_hw->dbgpause = 0; // hack!
    stdio_init_all();

    sleep_ms(1000);

    printf("SensorHub init...\n");

    // Setup power led indicator
    gpio_init(POWER_LED_INDICATOR_GPIO);
    gpio_set_dir(POWER_LED_INDICATOR_GPIO, GPIO_OUT);
    
    queue_init(&message_queue, sizeof(queue_entry_t), 20);
    queue_init(&status_queue, sizeof(bool), 1);
    memset(sensors, 0, sizeof(sensors));    
    multicore_launch_core1(core1_main);

    printf("Initializing RF receiver system on core %d. Waiting for MQTT system to start.\n", get_core_num());
    bool mqttOnline = false;
    bool ledState = true;
    while (!queue_try_peek(&status_queue, &mqttOnline) || !mqttOnline)
    {
        // Indicate the initialization process with flashing power led
        gpio_put(POWER_LED_INDICATOR_GPIO, ledState);
        sleep_ms(1000);    
        ledState = !ledState;
    }
  
    printf("MQTT system is online. Starting RF receiver system.\n");

    rf_pico_receiver receiver;
    pico_init_receiver(&receiver, report_result);
    pico_rx_start_receiving(&receiver);
    ledState = true;
    while (true)
    {
        gpio_put(POWER_LED_INDICATOR_GPIO, ledState);
        sleep_ms(3000); 
        ledState = !ledState;
    }
    return 0;
}