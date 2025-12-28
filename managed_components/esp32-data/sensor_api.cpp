// sensor_api.cpp (implementation)

#include "sensor_api.h"
#include <cstring>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include "esp_timer.h"
#include "cJSON.h"
#include "sensor_type_utils.h"  // assuming this exists for toString/fromString

namespace SensorApi {

// ─────────────────────────────────────────────────────────────────────────────
// Request Factory Methods
// ─────────────────────────────────────────────────────────────────────────────

Request Request::createGetLatest() {
    return Request(RequestType::GET_LATEST);
}

Request Request::createGetRange(uint32_t start, uint32_t end, uint8_t id) {
    Request r(RequestType::GET_RANGE);
    r.params = RangeParams{start, end, id};
    return r;
}

Request Request::createGetLastN(size_t count) {
    Request r(RequestType::GET_LAST_N);
    r.params = LastNParams{count};
    return r;
}

Request Request::createAddData(const SensorData& data) {
    Request r(RequestType::ADD_DATA);
    r.params = AddDataParams{data};
    return r;
}

Request Request::createGetStats() {
    return Request(RequestType::GET_STATS);
}

Request Request::createGetCount() {
    return Request(RequestType::GET_COUNT);
}

Request Request::createClearData() {
    return Request(RequestType::CLEAR_DATA);
}

Request Request::createExportData(ExportFormat fmt) {
    Request r(RequestType::EXPORT_DATA);
    r.params = ExportParams{fmt};
    return r;
}

Request Request::createRemoveOld(uint32_t ts) {
    Request r(RequestType::REMOVE_OLD);
    r.params = RemoveOldParams{ts};
    return r;
}

Request Request::createGetByType(SensorType t, uint8_t id) {
    Request r(RequestType::GET_BY_TYPE);
    r.params = ByTypeParams{t, id};
    return r;
}

Request Request::createGetByTypes(const std::vector<SensorType>& types, uint8_t id) {
    Request r(RequestType::GET_BY_TYPES);
    r.params = ByTypesParams{types, id};
    return r;
}

Request Request::createGetSensorIds() {
    return Request(RequestType::GET_SENSOR_IDS);
}

Request Request::createGetTypes(uint8_t id) {
    Request r(RequestType::GET_TYPES);
    r.params = GetTypesParams{id};
    return r;
}

Request Request::createFilterData(uint32_t start, uint32_t end, uint8_t id,
                                  const std::vector<SensorType>& types) {
    Request r(RequestType::FILTER_DATA);
    r.params = FilterParams{start, end, id, types};
    return r;
}

Request Request::createGetStatsByType(SensorType t, uint8_t id) {
    Request r(RequestType::GET_STATS_BY_TYPE);
    r.params = StatsByTypeParams{t, id};
    return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Request::toJson()
// ─────────────────────────────────────────────────────────────────────────────

std::string Request::toJson() const {
    cJSON* root = cJSON_CreateObject();
    if (!root) return "{}";

    const char* type_str = "unknown";
    switch (type) {
        case RequestType::GET_LATEST:       type_str = "get_latest"; break;
        case RequestType::GET_RANGE:        type_str = "get_range"; break;
        case RequestType::GET_LAST_N:       type_str = "get_last_n"; break;
        case RequestType::ADD_DATA:         type_str = "add_data"; break;
        case RequestType::GET_STATS:        type_str = "get_stats"; break;
        case RequestType::GET_COUNT:        type_str = "get_count"; break;
        case RequestType::CLEAR_DATA:       type_str = "clear_data"; break;
        case RequestType::EXPORT_DATA:      type_str = "export_data"; break;
        case RequestType::REMOVE_OLD:       type_str = "remove_old"; break;
        case RequestType::GET_BY_TYPE:      type_str = "get_by_type"; break;
        case RequestType::GET_BY_TYPES:     type_str = "get_by_types"; break;
        case RequestType::GET_SENSOR_IDS:   type_str = "get_sensor_ids"; break;
        case RequestType::GET_TYPES:        type_str = "get_types"; break;
        case RequestType::FILTER_DATA:      type_str = "filter_data"; break;
        case RequestType::GET_STATS_BY_TYPE:type_str = "get_stats_by_type"; break;
    }
    cJSON_AddStringToObject(root, "type", type_str);

    std::visit([&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, RangeParams>) {
            cJSON_AddNumberToObject(root, "start_time", arg.start_time);
            cJSON_AddNumberToObject(root, "end_time", arg.end_time);
            cJSON_AddNumberToObject(root, "sensor_id", arg.sensor_id);
        }
        else if constexpr (std::is_same_v<T, LastNParams>) {
            cJSON_AddNumberToObject(root, "count", arg.count);
        }
        else if constexpr (std::is_same_v<T, AddDataParams>) {
            std::string data_json = arg.data.toJson();
            cJSON* data_obj = cJSON_Parse(data_json.c_str());
            if (data_obj) cJSON_AddItemToObject(root, "data", data_obj);
        }
        else if constexpr (std::is_same_v<T, ExportParams>) {
            const char* fmt = (arg.format == ExportFormat::JSON) ? "json" :
                              (arg.format == ExportFormat::CSV)  ? "csv"  : "binary";
            cJSON_AddStringToObject(root, "format", fmt);
        }
        else if constexpr (std::is_same_v<T, RemoveOldParams>) {
            cJSON_AddNumberToObject(root, "timestamp", arg.timestamp);
        }
        else if constexpr (std::is_same_v<T, ByTypeParams>) {
            cJSON_AddStringToObject(root, "sensor_type", SensorTypeUtils::toString(arg.type).c_str());
            cJSON_AddNumberToObject(root, "sensor_id", arg.sensor_id);
        }
        else if constexpr (std::is_same_v<T, ByTypesParams>) {
            cJSON* arr = cJSON_CreateArray();
            for (auto t : arg.types) {
                cJSON_AddItemToArray(arr, cJSON_CreateString(SensorTypeUtils::toString(t).c_str()));
            }
            cJSON_AddItemToObject(root, "types", arr);
            cJSON_AddNumberToObject(root, "sensor_id", arg.sensor_id);
        }
        else if constexpr (std::is_same_v<T, FilterParams>) {
            cJSON_AddNumberToObject(root, "start_time", arg.start_time);
            cJSON_AddNumberToObject(root, "end_time", arg.end_time);
            cJSON_AddNumberToObject(root, "sensor_id", arg.sensor_id);
            if (!arg.types.empty()) {
                cJSON* arr = cJSON_CreateArray();
                for (auto t : arg.types) {
                    cJSON_AddItemToArray(arr, cJSON_CreateString(SensorTypeUtils::toString(t).c_str()));
                }
                cJSON_AddItemToObject(root, "types", arr);
            }
        }
        else if constexpr (std::is_same_v<T, GetTypesParams>) {
            cJSON_AddNumberToObject(root, "sensor_id", arg.sensor_id);
        }
        else if constexpr (std::is_same_v<T, StatsByTypeParams>) {
            cJSON_AddStringToObject(root, "sensor_type", SensorTypeUtils::toString(arg.type).c_str());
            cJSON_AddNumberToObject(root, "sensor_id", arg.sensor_id);
        }
    }, params);

    char* json = cJSON_PrintUnformatted(root);
    std::string result(json ? json : "{}");
    free(json);
    cJSON_Delete(root);
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Request::fromJson()
// ─────────────────────────────────────────────────────────────────────────────

Request Request::fromJson(const std::string& json_str) {
    cJSON* root = cJSON_Parse(json_str.c_str());
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return Request();
    }

    Request req;
    cJSON* type_item = cJSON_GetObjectItem(root, "type");
    if (!type_item || !cJSON_IsString(type_item)) {
        cJSON_Delete(root);
        return req;
    }

    const char* type_str = type_item->valuestring;
    if (strcmp(type_str, "get_latest") == 0) req.type = RequestType::GET_LATEST;
    else if (strcmp(type_str, "get_range") == 0) req.type = RequestType::GET_RANGE;
    else if (strcmp(type_str, "get_last_n") == 0) req.type = RequestType::GET_LAST_N;
    else if (strcmp(type_str, "add_data") == 0) req.type = RequestType::ADD_DATA;
    else if (strcmp(type_str, "get_stats") == 0) req.type = RequestType::GET_STATS;
    else if (strcmp(type_str, "get_count") == 0) req.type = RequestType::GET_COUNT;
    else if (strcmp(type_str, "clear_data") == 0) req.type = RequestType::CLEAR_DATA;
    else if (strcmp(type_str, "export_data") == 0) req.type = RequestType::EXPORT_DATA;
    else if (strcmp(type_str, "remove_old") == 0) req.type = RequestType::REMOVE_OLD;
    else if (strcmp(type_str, "get_by_type") == 0) req.type = RequestType::GET_BY_TYPE;
    else if (strcmp(type_str, "get_by_types") == 0) req.type = RequestType::GET_BY_TYPES;
    else if (strcmp(type_str, "get_sensor_ids") == 0) req.type = RequestType::GET_SENSOR_IDS;
    else if (strcmp(type_str, "get_types") == 0) req.type = RequestType::GET_TYPES;
    else if (strcmp(type_str, "filter_data") == 0) req.type = RequestType::FILTER_DATA;
    else if (strcmp(type_str, "get_stats_by_type") == 0) req.type = RequestType::GET_STATS_BY_TYPE;
    else {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Unknown request type: %s", type_str);
        return req;
    }

    std::visit([&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, RangeParams>) {
            cJSON* s = cJSON_GetObjectItem(root, "start_time"); if (s) arg.start_time = s->valueint;
            cJSON* e = cJSON_GetObjectItem(root, "end_time");   if (e) arg.end_time = e->valueint;
            cJSON* id = cJSON_GetObjectItem(root, "sensor_id"); if (id) arg.sensor_id = id->valueint;
        }
        else if constexpr (std::is_same_v<T, LastNParams>) {
            cJSON* c = cJSON_GetObjectItem(root, "count"); if (c) arg.count = c->valueint;
        }
        else if constexpr (std::is_same_v<T, AddDataParams>) {
            cJSON* d = cJSON_GetObjectItem(root, "data");
            if (d) {
                char* js = cJSON_PrintUnformatted(d);
                if (js) {
                    arg.data = SensorData::fromJson(js);
                    free(js);
                }
            }
        }
        else if constexpr (std::is_same_v<T, ExportParams>) {
            cJSON* f = cJSON_GetObjectItem(root, "format");
            if (f && cJSON_IsString(f)) {
                if (strcmp(f->valuestring, "json") == 0) arg.format = ExportFormat::JSON;
                else if (strcmp(f->valuestring, "csv") == 0) arg.format = ExportFormat::CSV;
                else if (strcmp(f->valuestring, "binary") == 0) arg.format = ExportFormat::BINARY;
            }
        }
        else if constexpr (std::is_same_v<T, RemoveOldParams>) {
            cJSON* t = cJSON_GetObjectItem(root, "timestamp");
            if (t) arg.timestamp = t->valueint;
        }
        else if constexpr (std::is_same_v<T, ByTypeParams>) {
            cJSON* t = cJSON_GetObjectItem(root, "sensor_type");
            if (t) arg.type = SensorTypeUtils::fromString(t->valuestring);
            cJSON* id = cJSON_GetObjectItem(root, "sensor_id");
            if (id) arg.sensor_id = id->valueint;
        }
        else if constexpr (std::is_same_v<T, ByTypesParams>) {
            cJSON* arr = cJSON_GetObjectItem(root, "types");
            if (arr && cJSON_IsArray(arr)) {
                for (int i = 0; i < cJSON_GetArraySize(arr); ++i) {
                    cJSON* item = cJSON_GetArrayItem(arr, i);
                    if (item && cJSON_IsString(item)) {
                        arg.types.push_back(SensorTypeUtils::fromString(item->valuestring));
                    }
                }
            }
            cJSON* id = cJSON_GetObjectItem(root, "sensor_id");
            if (id) arg.sensor_id = id->valueint;
        }
        else if constexpr (std::is_same_v<T, FilterParams>) {
            cJSON* s = cJSON_GetObjectItem(root, "start_time"); if (s) arg.start_time = s->valueint;
            cJSON* e = cJSON_GetObjectItem(root, "end_time");   if (e) arg.end_time = e->valueint;
            cJSON* id = cJSON_GetObjectItem(root, "sensor_id"); if (id) arg.sensor_id = id->valueint;
            cJSON* arr = cJSON_GetObjectItem(root, "types");
            if (arr && cJSON_IsArray(arr)) {
                for (int i = 0; i < cJSON_GetArraySize(arr); ++i) {
                    cJSON* item = cJSON_GetArrayItem(arr, i);
                    if (item && cJSON_IsString(item)) {
                        arg.types.push_back(SensorTypeUtils::fromString(item->valuestring));
                    }
                }
            }
        }
        else if constexpr (std::is_same_v<T, GetTypesParams>) {
            cJSON* id = cJSON_GetObjectItem(root, "sensor_id");
            if (id) arg.sensor_id = id->valueint;
        }
        else if constexpr (std::is_same_v<T, StatsByTypeParams>) {
            cJSON* t = cJSON_GetObjectItem(root, "sensor_type");
            if (t) arg.type = SensorTypeUtils::fromString(t->valuestring);
            cJSON* id = cJSON_GetObjectItem(root, "sensor_id");
            if (id) arg.sensor_id = id->valueint;
        }
    }, req.params);

    cJSON_Delete(root);
    return req;
}

// ─────────────────────────────────────────────────────────────────────────────
// Response implementations
// ─────────────────────────────────────────────────────────────────────────────

Response Response::createSuccess() {
    return Response{ESP_OK, ""};
}

Response Response::createError(esp_err_t code, const std::string& msg) {
    return Response{code, msg.empty() ? esp_err_to_name(code) : msg};
}

Response Response::createDataResponse(const void* ptr, size_t size, size_t count) {
    Response r;
    if (ptr && size > 0) {
        r.data = std::shared_ptr<void>(malloc(size), free);
        if (r.data) {
            memcpy(r.data.get(), ptr, size);
            r.data_size = size;
            r.data_count = count;
        } else {
            r.error_code = ESP_ERR_NO_MEM;
            r.error_message = "Memory allocation failed";
        }
    }
    return r;
}

Response Response::createSensorDataResponse(const SensorData& data) {
    return createDataResponse(&data, sizeof(SensorData));
}

Response Response::createSensorDataListResponse(const std::vector<SensorData>& data) {
    if (data.empty()) return createDataResponse(nullptr, 0, 0);
    return createDataResponse(data.data(), data.size() * sizeof(SensorData), data.size());
}

Response Response::createStatsResponse(const SensorStore::Stats& stats) {
    return createDataResponse(&stats, sizeof(SensorStore::Stats));
}

Response Response::createTypeStatsResponse(const std::vector<TypeStats>& stats) {
    if (stats.empty()) return createDataResponse(nullptr, 0, 0);
    return createDataResponse(stats.data(), stats.size() * sizeof(TypeStats), stats.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// SensorApi Singleton & Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

SensorApi& SensorApi::getInstance() {
    static SensorApi instance;
    return instance;
}

SensorApi::SensorApi() = default;

SensorApi::~SensorApi() {
    deinit();
}

esp_err_t SensorApi::init(size_t max_buffer_size) {
    if (initialized_) return ESP_OK;
    if (max_buffer_size == 0) max_buffer_size = 8192;

    buffer_ = std::make_unique<char[]>(max_buffer_size);
    if (!buffer_) return ESP_ERR_NO_MEM;

    buffer_size_ = max_buffer_size;
    initialized_ = true;

    ESP_LOGI(TAG, "Initialized with buffer size: %zu", max_buffer_size);
    return ESP_OK;
}

void SensorApi::deinit() {
    if (!initialized_) return;
    buffer_.reset();
    buffer_size_ = 0;
    initialized_ = false;
    ESP_LOGI(TAG, "Deinitialized");
}

// ─────────────────────────────────────────────────────────────────────────────
// Process Request
// ─────────────────────────────────────────────────────────────────────────────

Response SensorApi::processRequest(const Request& request) {
    if (!initialized_) return Response::createError(ESP_ERR_INVALID_STATE, "API not initialized");

    switch (request.type) {
        case RequestType::GET_LATEST:        return processGetLatest(request);
        case RequestType::GET_RANGE:         return processGetRange(request);
        case RequestType::GET_LAST_N:        return processGetLastN(request);
        case RequestType::ADD_DATA:          return processAddData(request);
        case RequestType::GET_STATS:         return processGetStats(request);
        case RequestType::GET_COUNT:         return processGetCount(request);
        case RequestType::CLEAR_DATA:        return processClearData(request);
        case RequestType::EXPORT_DATA:       return processExportData(request);
        case RequestType::REMOVE_OLD:        return processRemoveOld(request);
        case RequestType::GET_BY_TYPE:       return processGetByType(request);
        case RequestType::GET_BY_TYPES:      return processGetByTypes(request);
        case RequestType::GET_SENSOR_IDS:    return processGetSensorIds(request);
        case RequestType::GET_TYPES:         return processGetTypes(request);
        case RequestType::FILTER_DATA:       return processFilterData(request);
        case RequestType::GET_STATS_BY_TYPE: return processGetStatsByType(request);
        default: return Response::createError(ESP_ERR_NOT_SUPPORTED, "Unsupported request type");
    }
}

Response SensorApi::processJsonRequest(const std::string& json) {
    Request req = Request::fromJson(json);
    return processRequest(req);
}

// ─────────────────────────────────────────────────────────────────────────────
// Individual request handlers
// ─────────────────────────────────────────────────────────────────────────────

Response SensorApi::processGetLatest(const Request&) {
    SensorData data;
    esp_err_t ret = SensorStore::getInstance().getLatest(data);
    if (ret == ESP_OK) return Response::createSensorDataResponse(data);
    if (ret == ESP_ERR_NOT_FOUND) return Response::createError(ret, "No data available");
    return Response::createError(ret);
}

Response SensorApi::processGetRange(const Request& req) {
    auto& p = std::get<RangeParams>(req.params);
    DataRange range(p.start_time, p.end_time, p.sensor_id, p.sensor_id == 0);
    auto data = SensorStore::getInstance().getRange(range);
    return Response::createSensorDataListResponse(data);
}

Response SensorApi::processGetLastN(const Request& req) {
    auto& p = std::get<LastNParams>(req.params);
    size_t n = p.count ? p.count : 100;
    if (n > 10000) n = 10000;
    auto data = SensorStore::getInstance().getLastN(n);
    return Response::createSensorDataListResponse(data);
}

Response SensorApi::processAddData(const Request& req) {
    auto& p = std::get<AddDataParams>(req.params);
    esp_err_t ret = SensorStore::getInstance().addData(p.data);
    return (ret == ESP_OK) ? Response::createSuccess() : Response::createError(ret);
}

Response SensorApi::processGetStats(const Request&) {
    auto stats = SensorStore::getInstance().getStats();
    return Response::createStatsResponse(stats);
}

Response SensorApi::processGetCount(const Request&) {
    size_t count = SensorStore::getInstance().getCount();
    return Response::createDataResponse(&count, sizeof(size_t));
}

Response SensorApi::processClearData(const Request&) {
    esp_err_t ret = SensorStore::getInstance().clear();
    return (ret == ESP_OK) ? Response::createSuccess() : Response::createError(ret);
}

Response SensorApi::processExportData(const Request& req) {
    auto& p = std::get<ExportParams>(req.params);
    std::string data = SensorStore::getInstance().exportData(p.format);
    return data.empty() ? Response::createError(ESP_ERR_NOT_FOUND, "No data to export")
                        : Response::createDataResponse(data.data(), data.size());
}

Response SensorApi::processRemoveOld(const Request& req) {
    auto& p = std::get<RemoveOldParams>(req.params);
    size_t count = 0;
    esp_err_t ret = SensorStore::getInstance().removeOlderThan(p.timestamp, &count);
    return (ret == ESP_OK) ? Response::createDataResponse(&count, sizeof(size_t))
                           : Response::createError(ret);
}

Response SensorApi::processGetByType(const Request& req) {
    auto& p = std::get<ByTypeParams>(req.params);
    DataRange range(0, UINT32_MAX, p.sensor_id, p.sensor_id == 0);
    range.addType(p.type);
    auto data = SensorStore::getInstance().getRange(range);
    return Response::createSensorDataListResponse(data);
}

Response SensorApi::processGetByTypes(const Request& req) {
    auto& p = std::get<ByTypesParams>(req.params);
    if (p.types.empty()) return Response::createError(ESP_ERR_INVALID_ARG, "No types specified");
    DataRange range(0, UINT32_MAX, p.sensor_id, p.sensor_id == 0);
    for (auto t : p.types) range.addType(t);
    auto data = SensorStore::getInstance().getRange(range);
    return Response::createSensorDataListResponse(data);
}

Response SensorApi::processGetSensorIds(const Request&) {
    auto all = SensorStore::getInstance().getLastN(SensorStore::getInstance().getCount());
    std::vector<uint8_t> ids;
    for (const auto& d : all) {
        uint8_t id = d.getSensorId();
        if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
            ids.push_back(id);
        }
    }
    return ids.empty() ? Response::createSuccess()
                       : Response::createDataResponse(ids.data(), ids.size() * sizeof(uint8_t), ids.size());
}

Response SensorApi::processGetTypes(const Request& req) {
    auto& p = std::get<GetTypesParams>(req.params);
    auto all = SensorStore::getInstance().getLastN(SensorStore::getInstance().getCount());
    std::vector<SensorType> types;
    for (const auto& d : all) {
        if (p.sensor_id != 0 && d.getSensorId() != p.sensor_id) continue;
        for (uint8_t i = 0; i < d.getValueCount(); ++i) {
            const SensorValue* v = d.getValueByIndex(i);
            if (v && std::find(types.begin(), types.end(), v->type) == types.end()) {
                types.push_back(v->type);
            }
        }
    }
    if (types.empty()) return Response::createSuccess();
    std::vector<uint8_t> type_bytes;
    for (auto t : types) type_bytes.push_back(static_cast<uint8_t>(t));
    return Response::createDataResponse(type_bytes.data(), type_bytes.size() * sizeof(uint8_t), type_bytes.size());
}

Response SensorApi::processFilterData(const Request& req) {
    auto& p = std::get<FilterParams>(req.params);
    DataRange range(p.start_time, p.end_time, p.sensor_id, p.sensor_id == 0);
    for (auto t : p.types) range.addType(t);
    auto data = SensorStore::getInstance().getRange(range);
    return Response::createSensorDataListResponse(data);
}

Response SensorApi::processGetStatsByType(const Request& req) {
    auto& p = std::get<StatsByTypeParams>(req.params);
    DataRange range(0, UINT32_MAX, p.sensor_id, p.sensor_id == 0);
    range.addType(p.type);
    auto data = SensorStore::getInstance().getRange(range);
    if (data.empty()) return Response::createSuccess();

    TypeStats stats{};
    stats.type = p.type;
    stats.min_value = FLT_MAX;
    stats.max_value = -FLT_MAX;
    float sum = 0.0f;
    size_t valid = 0;

    for (const auto& entry : data) {
        float val = 0.0f;
        if (entry.getValue(p.type, val)) {
            stats.min_value = std::min(stats.min_value, val);
            stats.max_value = std::max(stats.max_value, val);
            sum += val;
            valid++;
            stats.last_timestamp = std::max(stats.last_timestamp, entry.getTimestamp());
        }
    }

    if (valid > 0) {
        stats.count = valid;
        stats.avg_value = sum / valid;

        float var_sum = 0.0f;
        for (const auto& entry : data) {
            float val = 0.0f;
            if (entry.getValue(p.type, val)) {
                float diff = val - stats.avg_value;
                var_sum += diff * diff;
            }
        }
        stats.std_dev = sqrtf(var_sum / valid);
    }

    return Response::createDataResponse(&stats, sizeof(TypeStats));
}

// ─────────────────────────────────────────────────────────────────────────────
// TypeStats calculation helper (if needed for multi-type stats in future)
// ─────────────────────────────────────────────────────────────────────────────

std::vector<SensorApi::TypeStats> SensorApi::calculateTypeStats(
    const std::vector<SensorData>& data,
    const std::vector<SensorType>& types) {

    std::vector<TypeStats> results;
    for (auto t : types) {
        TypeStats s{};
        s.type = t;
        s.min_value = FLT_MAX;
        s.max_value = -FLT_MAX;
        float sum = 0.0f;
        size_t count = 0;

        for (const auto& d : data) {
            float val = 0.0f;
            if (d.getValue(t, val)) {
                s.min_value = std::min(s.min_value, val);
                s.max_value = std::max(s.max_value, val);
                sum += val;
                count++;
                s.last_timestamp = std::max(s.last_timestamp, d.getTimestamp());
            }
        }

        if (count > 0) {
            s.count = count;
            s.avg_value = sum / count;
            float var_sum = 0.0f;
            for (const auto& d : data) {
                float val = 0.0f;
                if (d.getValue(t, val)) {
                    float diff = val - s.avg_value;
                    var_sum += diff * diff;
                }
            }
            s.std_dev = sqrtf(var_sum / count);
        }
        results.push_back(s);
    }
    return results;
}

}  // namespace SensorApi