#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sqlite3.h>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace {

/** Format current system time as "YYYY-MM-DD HH:MM:SS" (local time) */
std::string now_local_str() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%F %T", &tm);
    return std::string(buf);
}

static double estimateDuration(const std::string& start, const std::string& end) {
    auto parse = [](const std::string& t) -> std::tm {
        std::tm tm{};
        if (t.size() >= 19) { // "YYYY-MM-DD HH:MM:SS"
            std::istringstream ss(t);
            ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
        }
        return tm;
    };
    std::tm tm1 = parse(start);
    std::tm tm2 = parse(end);
    auto t1 = std::mktime(&tm1);
    auto t2 = std::mktime(&tm2);
    if (t1 == -1 || t2 == -1) return 0.0;
    return std::difftime(t2, t1); // Return the difference in seconds
}

} // namespace

struct CycleData {
    std::string model;
    std::string color;
    std::string size;
    std::string detection_ts;
    std::string pick_ts;
    std::string place_ts;
    double cycle_duration_sec = 0.0;
    bool complete = false;
};

class AnalyticsLoggerNode : public rclcpp::Node {
public:
    AnalyticsLoggerNode()
    : Node("analytics_logger_node")
    {
        // Parameters
        db_path_ = this->declare_parameter<std::string>(
            "db_path", std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/parol6/parol6_pipeline/pipeline_analytics.db"
        );
        csv_path_ = this->declare_parameter<std::string>(
            "csv_path", std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/parol6/parol6_pipeline/pipeline_analytics.csv"
        );
        sim_start_ts_ = now_local_str();

        // Ensure parent dir exists
        try {
            std::filesystem::create_directories(std::filesystem::path(db_path_).parent_path());
            std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());
        } catch (...) {}

        openDb();
        createTableIfNeeded();

        // Subscriptions
        sub_events_ = this->create_subscription<std_msgs::msg::String>(
            "/analytics/event", rclcpp::QoS(100), 
            std::bind(&AnalyticsLoggerNode::onEvent, this, std::placeholders::_1)
        );


        // Live feed publisher
        pub_live_ = this->create_publisher<std_msgs::msg::String>("/analytics/live", 10);

        // Export service -> writes CSV to csv_path_
        srv_export_ = this->create_service<std_srvs::srv::Trigger>(
            "/analytics/export_csv",
            std::bind(&AnalyticsLoggerNode::onExportCsv, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // Clear table service
        srv_clear_ = this->create_service<std_srvs::srv::Trigger>(
            "/analytics/clear",
            std::bind(&AnalyticsLoggerNode::onClear, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(get_logger(), "AnalyticsLoggerNode ready. DB: %s", db_path_.c_str());
    }

    ~AnalyticsLoggerNode() override {
        if (db_) sqlite3_close(db_);
    }

private:
    // ---------- DB ----------
    void openDb() {
        if (sqlite3_open(db_path_.c_str(), &db_) != SQLITE_OK) {
            auto err = sqlite3_errmsg(db_);
            throw std::runtime_error(std::string("Failed to open DB: ") + (err ? err : "unknown"));
        }
    }

    void execSql(const char* sql) {
        char* errMsg = nullptr;
        if (sqlite3_exec(db_, sql, nullptr, nullptr, &errMsg) != SQLITE_OK) {
            std::string err = errMsg ? errMsg : "unknown";
            sqlite3_free(errMsg);
            throw std::runtime_error("SQLite error: " + err);
        }
    }

    void createTableIfNeeded() {
        const char* sql = R"SQL(
            CREATE TABLE IF NOT EXISTS pick_place_analytics (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                sim_start_ts TEXT NOT NULL,
                model TEXT NOT NULL,
                detected_color TEXT,
                size TEXT,
                detection_ts TEXT,
                pick_ts TEXT,
                place_ts TEXT,
                cycle_duration REAL
            );
            CREATE INDEX IF NOT EXISTS idx_model ON pick_place_analytics(model);
        )SQL";
        execSql(sql);
    }

    void insertDetection(const CycleData& d) {
        const char* sql = R"SQL(
            INSERT INTO pick_place_analytics
            (sim_start_ts, model, detected_color, size, detection_ts)
            VALUES (?, ?, ?, ?, ?);
        )SQL";
        sqlite3_stmt* stmt = nullptr;
        if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
            throw std::runtime_error("Prepare insertDetection failed");
        }
        sqlite3_bind_text(stmt, 1, sim_start_ts_.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, d.model.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 3, d.color.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 4, d.size.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 5, d.detection_ts.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
    }

    void updatePick(const CycleData& d) {
        const char* sql = R"SQL(
            UPDATE pick_place_analytics
            SET pick_ts = ?
            WHERE sim_start_ts = ? AND model = ?;
        )SQL";
        sqlite3_stmt* stmt = nullptr;
        sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
        sqlite3_bind_text(stmt, 1, d.pick_ts.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, sim_start_ts_.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 3, d.model.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
    }

    void updatePlaceAndDuration(const CycleData& d) {
        const char* sql = R"SQL(
            UPDATE pick_place_analytics
            SET place_ts = ?, cycle_duration = ?
            WHERE sim_start_ts = ? AND model = ?;
        )SQL";
        sqlite3_stmt* stmt = nullptr;
        sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
        sqlite3_bind_text(stmt, 1, d.place_ts.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_double(stmt, 2, d.cycle_duration_sec);
        sqlite3_bind_text(stmt, 3, sim_start_ts_.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 4, d.model.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
    }

    // ---------- ROS Callbacks ----------
    void onEvent(const std_msgs::msg::String::SharedPtr msg) {
        // Handle event and update DB
        json j;
        try {
            j = json::parse(msg->data);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Invalid JSON on /analytics/event: %s", e.what());
            return;
        }

        const std::string event   = j.value("event", "");
        const std::string model   = j.value("model", "");
        const std::string color   = j.value("color", "");
        const std::string size    = j.value("size", "");
        const std::string ts_in   = j.value("timestamp", "");
        const std::string ts      = ts_in.empty() ? now_local_str() : ts_in;

        if (event.empty() || model.empty()) {
            RCLCPP_WARN(get_logger(), "Missing required fields: event/model");
            return;
        }

        std::lock_guard<std::mutex> lk(mutex_);
        CycleData& d = cycles_[model];
        d.model = model;

        if (event == "detection") {
            d.color = color;
            d.size = size;
            d.detection_ts = ts;
            try {
                insertDetection(d);
                liveEmit("detection", d);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "DB insertDetection failed: %s", e.what());
            }
        } else if (event == "pick") {
            d.pick_ts = ts;
            try {
                updatePick(d);
                liveEmit("pick", d);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "DB updatePick failed: %s", e.what());
            }
        } else if (event == "place") {
            d.place_ts = ts;
            if (!d.detection_ts.empty()) {
                d.cycle_duration_sec = estimateDuration(d.detection_ts, d.place_ts);
            }
            d.complete = true;
            try {
                updatePlaceAndDuration(d);
                liveEmit("place", d);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "DB updatePlace failed: %s", e.what());
            }
        } else {
            RCLCPP_WARN(get_logger(), "Unknown event type: %s", event.c_str());
        }
    }

    // Live data to console and topic
    void liveEmit(const std::string& stage, const CycleData& d) {
        json out = {
            {"stage", stage},
            {"sim_start_ts", sim_start_ts_},
            {"model", d.model},
            {"color", d.color},
            {"size", d.size},
            {"detection_ts", d.detection_ts},
            {"pick_ts", d.pick_ts},
            {"place_ts", d.place_ts},
            {"cycle_duration_sec", d.cycle_duration_sec},
            {"complete", d.complete}
        };
        auto s = out.dump();
        RCLCPP_INFO(get_logger(), "[LIVE] %s", s.c_str());
        std_msgs::msg::String m; m.data = s;
        pub_live_->publish(m);
    }

    // Export data to CSV
    void onExportCsv(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        try {
            std::FILE* f = std::fopen(csv_path_.c_str(), "w");
            if (!f) throw std::runtime_error("Could not open CSV path for write");

            std::fprintf(f, "id,sim_start_ts,model,detected_color,size,detection_ts,pick_ts,place_ts,cycle_duration\n");

            const char* sql = R"SQL(
                SELECT id, sim_start_ts, model, detected_color, size,
                detection_ts, pick_ts, place_ts, cycle_duration
                FROM pick_place_analytics
                ORDER BY id ASC;
            )SQL";
            sqlite3_stmt* stmt = nullptr;
            if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
                std::fclose(f);
                throw std::runtime_error("Prepare export query failed");
            }
            while (sqlite3_step(stmt) == SQLITE_ROW) {
                auto col = [&](int i)->const unsigned char*{ return sqlite3_column_text(stmt, i); };
                auto col_d = [&](int i)->double{ return sqlite3_column_double(stmt, i); };
                std::fprintf(
                    f, "%d,%s,%s,%s,%s,%s,%s,%s,%.3f\n",
                    sqlite3_column_int(stmt, 0),
                    col(1) ? (const char*)col(1) : "",
                    col(2) ? (const char*)col(2) : "",
                    col(3) ? (const char*)col(3) : "",
                    col(4) ? (const char*)col(4) : "",
                    col(5) ? (const char*)col(5) : "",
                    col(6) ? (const char*)col(6) : "",
                    col(7) ? (const char*)col(7) : "",
                    sqlite3_column_type(stmt, 8) == SQLITE_NULL ? 0.0 : col_d(8)
                );
            }
            sqlite3_finalize(stmt);
            std::fclose(f);

            res->success = true;
            res->message = "CSV exported to: " + csv_path_;
            RCLCPP_INFO(get_logger(), "Exported CSV to %s", csv_path_.c_str());
        } catch (const std::exception& e) {
            res->success = false;
            res->message = std::string("Export failed: ") + e.what();
        }
    }

    // Clear data from the database
    void onClear(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        try {
            execSql("DELETE FROM pick_place_analytics;");
            std::lock_guard<std::mutex> lk(mutex_);
            cycles_.clear();
            res->success = true;
            res->message = "Analytics table cleared.";
            RCLCPP_WARN(get_logger(), "Analytics table cleared.");
        } catch (const std::exception& e) {
            res->success = false;
            res->message = std::string("Clear failed: ") + e.what();
        }
    }

private:
    std::string db_path_;
    std::string csv_path_;
    std::string sim_start_ts_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_events_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_live_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_export_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_;

    sqlite3* db_ = nullptr;

    std::unordered_map<std::string, CycleData> cycles_;
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnalyticsLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
