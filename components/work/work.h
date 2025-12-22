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
#include "sensor_data_manager.h"

#define WORK_SAMPLE_PERIOD_MS 30000

class WorkImplementation : public WorkInterface
{
private:
    static WorkImplementation* instance;
    static SensorDataManager& sensorDataManager;
    char     msg[512]{};
    float    humidity{0.0f};
    float    temperature{0.0f};
    uint32_t sampleCount{0};

    WorkImplementation();

public:
    WorkImplementation(const WorkImplementation&) = delete;
    WorkImplementation& operator=(const WorkImplementation&) = delete;

    static WorkImplementation& getInstance();
    virtual ~WorkImplementation() = default;

    void init_work()    override;
    void do_work()      override;
    void end_work()     override;
    void deinit_work()  override;
    const char* getMessage() override;
    void logE(const char* format, ...) override;

    // Optional convenience getters — declared only, defined in .cpp if you want them
    float getHumidity() const;
    float getTemperature() const;
    uint32_t getSampleCount() const;
};

void start_work_task();
const char* get_current_work_message();

#endif // WORK_H