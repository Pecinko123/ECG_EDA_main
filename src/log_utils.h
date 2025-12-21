#ifndef LOG_UTILS_H
#define LOG_UTILS_H

#include <SPIFFS.h>
#include <sys/time.h>
#include <cstdarg>

#define LOG_FILE "/log.txt"

/**
 * @brief Writes a formatted log message to a file on SPIFFS.
 *
 * This function works like printf, appending a timestamped log entry to the log file.
 *
 * @param format The format string.
 * @param ... The variable arguments.
 */
void logToFile(const char *format, ...)
{
    File file = SPIFFS.open(LOG_FILE, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open log file for appending");
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long time_ms = tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL);

    char log_buf[256];
    int initial_len = snprintf(log_buf, sizeof(log_buf), "[%llu] ", time_ms);

    va_list args;
    va_start(args, format);
    vsnprintf(log_buf + initial_len, sizeof(log_buf) - initial_len, format, args);
    va_end(args);

    file.println(log_buf);
    file.close();
}

/**
 * @brief Prints the contents of the log file to Serial and then deletes the file.
 */
void printAndClearLogs()
{
    File file = SPIFFS.open(LOG_FILE, FILE_READ);
    if (!file || file.size() == 0)
    {
        if (file)
            file.close();
        return;
    }

    Serial.println("\n--- PREVIOUS LOGS ---");
    while (file.available())
    {
        Serial.write(file.read());
    }
    file.close();
    Serial.println("--- END OF LOGS ---\n");

    SPIFFS.remove(LOG_FILE);
}

#endif // LOG_UTILS_H
