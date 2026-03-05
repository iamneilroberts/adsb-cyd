#include "http_mutex.h"

static SemaphoreHandle_t _mutex = nullptr;

void http_mutex_init() {
    _mutex = xSemaphoreCreateMutex();
}

bool http_mutex_acquire(TickType_t timeout) {
    return xSemaphoreTake(_mutex, timeout) == pdTRUE;
}

void http_mutex_release() {
    xSemaphoreGive(_mutex);
}
