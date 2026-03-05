#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

void http_mutex_init();
bool http_mutex_acquire(TickType_t timeout = portMAX_DELAY);
void http_mutex_release();
