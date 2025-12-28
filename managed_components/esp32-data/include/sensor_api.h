// sensor_api.h
#ifndef SENSOR_API_H
#define SENSOR_API_H

#include <cstdint>
#include <string>
#include <vector>
#include <variant>
#include <memory>
#include <utility>
#include <type_traits>
#include "esp_err.h"
#include "sensor_store.h"   // must define SensorData, SensorType, etc.
#include "sensor_data.h"    // if SensorType is here
#include "esp_log.h"

#define TAG "SENSOR_API"

// IMPORTANT: Define TypeStats here (before it's used)
struct TypeStats {
    SensorType type = SensorType::UNKNOWN;
    size_t count = 0;
    float min_value = 0.0f;
    float max_value = 0.0f;
    float avg_value = 0.0f;
    float std_dev = 0.0f;
    uint32_t last_timestamp = 0;
};

namespace SensorApi {

enum class RequestType : uint8_t {
    GET_LATEST,
    GET_RANGE,
    GET_LAST_N,
    ADD_DATA,
    GET_STATS,
    GET_COUNT,
    CLEAR_DATA,
    EXPORT_DATA,
    REMOVE_OLD,
    GET_BY_TYPE,
    GET_BY_TYPES,
    GET_SENSOR_IDS,
    GET_TYPES,
    FILTER_DATA,
    GET_STATS_BY_TYPE
};

enum class ExportFormat : uint8_t {
    JSON,
    CSV,
    BINARY
};

// Parameter structs
struct RangeParams { uint32_t start_time = 0; uint32_t end_time = 0; uint8_t sensor_id = 0; };
struct LastNParams { size_t count = 0; };
struct AddDataParams { SensorData data; };
struct ExportParams { ExportFormat format = ExportFormat::JSON; };
struct RemoveOldParams { uint32_t timestamp = 0; };
struct ByTypeParams { SensorType type = SensorType::UNKNOWN; uint8_t sensor_id = 0; };
struct ByTypesParams { std::vector<SensorType> types; uint8_t sensor_id = 0; };
struct GetTypesParams { uint8_t sensor_id = 0; };
struct FilterParams { uint32_t start_time = 0; uint32_t end_time = 0; uint8_t sensor_id = 0; std::vector<SensorType> types; };
struct StatsByTypeParams { SensorType type = SensorType::UNKNOWN; uint8_t sensor_id = 0; };

// Request class
class Request {
public:
    RequestType type;
    std::variant<
        std::monostate,
        RangeParams,
        LastNParams,
        AddDataParams,
        ExportParams,
        RemoveOldParams,
        ByTypeParams,
        ByTypesParams,
        GetTypesParams,
        FilterParams,
        StatsByTypeParams
    > params;

    Request() : type(RequestType::GET_LATEST), params(std::monostate{}) {}
    explicit Request(RequestType t) : type(t), params(std::monostate{}) {}

    Request(const Request&) = delete;
    Request& operator=(const Request&) = delete;

    Request(Request&& other) noexcept
        : type(other.type), params(std::move(other.params)) {
        other.type = RequestType::GET_LATEST;
        other.params = std::monostate{};
    }

    Request& operator=(Request&& other) noexcept {
        if (this != &other) {
            type = other.type;
            params = std::move(other.params);
            other.type = RequestType::GET_LATEST;
            other.params = std::monostate{};
        }
        return *this;
    }

    // Factory methods (declarations only)
    static Request createGetLatest();
    static Request createGetRange(uint32_t start, uint32_t end, uint8_t id = 0);
    static Request createGetLastN(size_t count);
    static Request createAddData(const SensorData& data);
    static Request createGetStats();
    static Request createGetCount();
    static Request createClearData();
    static Request createExportData(ExportFormat fmt);
    static Request createRemoveOld(uint32_t timestamp);
    static Request createGetByType(SensorType t, uint8_t id = 0);
    static Request createGetByTypes(const std::vector<SensorType>& types, uint8_t id = 0);
    static Request createGetSensorIds();
    static Request createGetTypes(uint8_t id = 0);
    static Request createFilterData(uint32_t start, uint32_t end, uint8_t id = 0,
                                    const std::vector<SensorType>& types = {});
    static Request createGetStatsByType(SensorType t, uint8_t id = 0);

    std::string toJson() const;
    static Request fromJson(const std::string& json);
};

// Response class
class Response {
public:
    esp_err_t error_code = ESP_OK;
    std::string error_message;
    std::shared_ptr<void> data;
    size_t data_size = 0;
    size_t data_count = 0;

    static Response createSuccess();
    static Response createError(esp_err_t code, const std::string& msg = "");
    static Response createDataResponse(const void* ptr, size_t size, size_t count = 1);
    static Response createSensorDataResponse(const SensorData& data);
    static Response createSensorDataListResponse(const std::vector<SensorData>& data);
    static Response createStatsResponse(const SensorStore::Stats& stats);
    static Response createTypeStatsResponse(const std::vector<TypeStats>& stats);

    std::string toJson() const;
};

class SensorApi {
public:
    static SensorApi& getInstance();

    esp_err_t init(size_t max_buffer_size = 8192);
    void deinit();

    Response processRequest(const Request& request);
    Response processJsonRequest(const std::string& json);

private:
    SensorApi();
    ~SensorApi();

    bool initialized_ = false;
    std::unique_ptr<char[]> buffer_;
    size_t buffer_size_ = 0;

    Response processGetLatest(const Request& req);
    Response processGetRange(const Request& req);
    Response processGetLastN(const Request& req);
    Response processAddData(const Request& req);
    Response processGetStats(const Request& req);
    Response processGetCount(const Request& req);
    Response processClearData(const Request& req);
    Response processExportData(const Request& req);
    Response processRemoveOld(const Request& req);
    Response processGetByType(const Request& req);
    Response processGetByTypes(const Request& req);
    Response processGetSensorIds(const Request& req);
    Response processGetTypes(const Request& req);
    Response processFilterData(const Request& req);
    Response processGetStatsByType(const Request& req);

    std::vector<TypeStats> calculateTypeStats(const std::vector<SensorData>& data,
                                              const std::vector<SensorType>& types);
};

} // namespace SensorApi

#endif // SENSOR_API_H