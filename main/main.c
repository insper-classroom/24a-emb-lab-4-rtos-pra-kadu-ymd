/*
 * LED blink with FreeRTOS
 */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "ssd1306.h"
#include "gfx.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <timers.h>

QueueHandle_t xQueueTime;
QueueHandle_t xQueueDistance;
SemaphoreHandle_t xSemaphoreTrigger;

const int TRIGGER_PIN = 2;
const int ECHO_PIN = 3;
const int V_SOM = 343;
char dist[20];

void pin_init(void) {
    gpio_init(TRIGGER_PIN);
    gpio_init(ECHO_PIN);

    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

void pin_callback(uint gpio, uint32_t events) {
    double time = 0.0;
    if(gpio == ECHO_PIN) {
        if(events == 0x4) {
            time = to_us_since_boot(get_absolute_time());
        } else if(events == 0x8) {
            time = to_us_since_boot(get_absolute_time());
        }
    }
    xQueueSendFromISR(xQueueTime, &time, 0);
}

void trigger_task(void *pvParameters) {
    while(1) {
        gpio_put(TRIGGER_PIN, 1);
        vTaskDelay(1);
        gpio_put(TRIGGER_PIN, 0);
        vTaskDelay(1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void echo_task(void *pvParameters) {
    double time_start = 0.0;
    double time_end = 0.0;

    gpio_set_irq_enabled_with_callback(ECHO_PIN,
                                       GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                                       true,
                                       &pin_callback);

    while(1) {
        double distance = 0.0;
        if(xQueueReceive(xQueueTime, &time_start, pdMS_TO_TICKS(100))) {
            if(xQueueReceive(xQueueTime, &time_end, pdMS_TO_TICKS(100))) {
                distance = V_SOM * (time_end - time_start) * 1e-6 * 1e2/2;
                xQueueSend(xQueueDistance, &distance, 0);
                xSemaphoreGive(xSemaphoreTrigger);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void oled_task(void *pvParameters) {
    printf("Inicializando Driver\n");
    ssd1306_init();

    printf("Inicializando GLX\n");
    ssd1306_t disp;
    gfx_init(&disp, 128, 32);

    while(1) {
        double distance = 0.0;

        if(xSemaphoreTake(xSemaphoreTrigger, pdMS_TO_TICKS(100))) {
            if(xQueueReceive(xQueueDistance, &distance, pdMS_TO_TICKS(100))) {
                int line = 14;
                sprintf(dist, "Dist: %f cm", distance);
                gfx_clear_buffer(&disp);
                gfx_draw_string(&disp, 0, 0, 1, dist);
                line = (int) distance + 14;
                if(distance > 100.00001) {
                    line = 114;
                }
                gfx_draw_line(&disp, 14, 27, line, 27);
                vTaskDelay(pdMS_TO_TICKS(50));
                gfx_show(&disp);
            } else {
                gfx_clear_buffer(&disp);
                gfx_draw_string(&disp, 0, 0, 1, "Dist: FALHA!");
                vTaskDelay(pdMS_TO_TICKS(50));
                gfx_show(&disp);
            }
        }
    }
}

int main() {
    stdio_init_all();
    pin_init();

    xSemaphoreTrigger = xSemaphoreCreateBinary();
    xQueueTime = xQueueCreate(32, sizeof(double));
    xQueueDistance = xQueueCreate(32, sizeof(double));

    xTaskCreate(trigger_task, "Trigger Task", 256, NULL, 1, NULL);
    xTaskCreate(echo_task, "Echo Task", 256, NULL, 1, NULL);
    xTaskCreate(oled_task, "OLED Task", 4095, NULL, 1, NULL);
    vTaskStartScheduler();

    while(true);

}
