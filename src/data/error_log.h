#pragma once
#include <stdint.h>

#define ERROR_LOG_MAX 10
#define ERROR_LOG_MSG_LEN 48

struct ErrorEntry {
    char msg[ERROR_LOG_MSG_LEN];
    uint32_t timestamp;  // millis()
};

struct ErrorSnapshot {
    ErrorEntry entries[ERROR_LOG_MAX];
    int count;           // number of valid entries (0..ERROR_LOG_MAX)
};

void error_log_init();

// Printf-style error logging (thread-safe)
void error_log_add(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Returns chronological copy of the ring buffer (oldest first)
ErrorSnapshot error_log_snapshot();

// Clear the ring buffer (total count persists)
void error_log_clear();

// Lifetime error count (survives clear)
uint32_t error_log_total_count();
