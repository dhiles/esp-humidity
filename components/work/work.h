#ifndef WORK_H
#define WORK_H

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "work_interface.h"
#include "myntp.h"
#include "mymqtt.h"
#include "ble_provisioning.h"

#define WORK_SAMPLE_PERIOD_MS 30000

class WorkImplementation : public WorkInterface {
private:
    static WorkImplementation* instance;
    char msg[512];
    float humidity;
    int sampleCount = 0;

    // Private constructor
    WorkImplementation();

public:
    // Delete copy constructor and assignment operator
    WorkImplementation(const WorkImplementation&) = delete;
    WorkImplementation& operator=(const WorkImplementation&) = delete;

    // Static method to get the singleton instance
    static WorkImplementation& getInstance();

    virtual ~WorkImplementation() = default;

    void init_work() override;
    void do_work() override;
    void end_work() override;
    void deinit_work() override;
    const char* getMessage() override;
    void logE(const char* format, ...) override;
};

// Function declarations
void start_work_task();
const char* get_current_work_message();

#endif // WORK_H