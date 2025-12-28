#ifndef WORK_INTERFACE_H
#define WORK_INTERFACE_H

#include <cstdarg>
#include <cstdio>
#include <cstring>

class WorkInterface {
protected:
    char logMsg[256];  // Buffer for JSON error messages

public:
    WorkInterface() {
        // Initialize with empty JSON object
        strcpy(logMsg, "{}");
    }
    
    virtual ~WorkInterface() = default;
    virtual void init_work() = 0;
    virtual void do_work() = 0;
    virtual void end_work() = 0;
    virtual void deinit_work() = 0;
    virtual const char* getMessage() = 0;
    
    // Provide default implementation that can be overridden if needed
    virtual void logE(const char* format, ...) {
        va_list args;
        va_start(args, format);
        
        // First log to ESP logger (you'll need to define TAG in derived classes)
        // esp_log_writev(ESP_LOG_ERROR, TAG, format, args);
        
        // For demonstration, using printf
        printf("[ERROR] ");
        vprintf(format, args);
        printf("\n");
        
        // Create formatted error message for JSON
        char errorMsg[256];
        va_list args_copy;
        va_copy(args_copy, args);
        vsnprintf(errorMsg, sizeof(errorMsg), format, args_copy);
        va_end(args_copy);
        
        // Create JSON error message and store in logMsg
        snprintf(logMsg, sizeof(logMsg), "{\"error\":\"%s\"}", errorMsg);
        
        va_end(args);
    }
    
    // Function to read the log message
    virtual const char* getLogMessage() const {
        return logMsg;
    }
};

#endif // WORK_INTERFACE_H