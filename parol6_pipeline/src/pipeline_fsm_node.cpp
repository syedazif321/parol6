#include "parol6_pipeline/pipeline_subsystems.hpp"
#include <unordered_map>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <sqlite3.h>
#include <atomic>
#include <thread>
#include <mutex>

// Mock classes were removed to resolve redefinition errors.
// The code now uses the definitions from the included header.

// --- AnalyticsDB: Modified to store session start/end times and L x W size ---
class AnalyticsDB {
public:
    explicit AnalyticsDB(rclcpp::Logger logger, const std::string &db_path = "analytics.db")
    : logger_(logger), db_path_(db_path), db_(nullptr) {
        open();
    }

    ~AnalyticsDB() {
        close();
    }

    bool open() {
        std::lock_guard<std::mutex> lk(m_);
        if (db_) return true;
        if (sqlite3_open(db_path_.c_str(), &db_) != SQLITE_OK) {
            RCLCPP_ERROR(logger_, "AnalyticsDB: cannot open DB %s: %s", db_path_.c_str(), sqlite3_errmsg(db_));
            sqlite3_close(db_);
            db_ = nullptr;
            return false;
        }

        exec("PRAGMA foreign_keys = ON;");

        exec("CREATE TABLE IF NOT EXISTS simulation_sessions ("
             "session_id INTEGER PRIMARY KEY AUTOINCREMENT,"
             "session_name TEXT NOT NULL,"
             "start_time TEXT NOT NULL,"
             "end_time TEXT);");


        exec("CREATE TABLE IF NOT EXISTS box_analytics ("
             "id INTEGER PRIMARY KEY AUTOINCREMENT,"
             "session_id INTEGER NOT NULL,"
             "box_id TEXT NOT NULL,"
             "color TEXT,"
             "size TEXT," 
             "detection_time TEXT,"
             "pick_time TEXT,"
             "place_time TEXT,"
             "cycle_duration REAL,"
             "FOREIGN KEY(session_id) REFERENCES simulation_sessions(session_id));");

        RCLCPP_INFO(logger_, "AnalyticsDB opened: %s", db_path_.c_str());
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lk(m_);
        if (db_) {
            sqlite3_close(db_);
            db_ = nullptr;
            RCLCPP_INFO(logger_, "AnalyticsDB closed");
        }
    }


    int startSession(const std::string &session_name, const std::string &start_time) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return -1;
        std::string sql = "INSERT INTO simulation_sessions (session_name, start_time) VALUES ('" + escape(session_name) + "', '" + escape(start_time) + "');";
        if (!exec(sql)) return -1;
        int id = static_cast<int>(sqlite3_last_insert_rowid(db_));
        RCLCPP_INFO(logger_, "AnalyticsDB: started session %d (%s) @ %s", id, session_name.c_str(), start_time.c_str());
        return id;
    }

    bool endSession(int session_id, const std::string &end_time) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return false;
        std::string sql = "UPDATE simulation_sessions SET end_time='" + escape(end_time) + "' WHERE session_id=" + std::to_string(session_id) + ";";
        return exec(sql);
    }

  
    bool logDetection(int session_id, const std::string &box_id,
                      const std::string &color, const std::string &size, const std::string &timestamp) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return false;
        std::string sql = "INSERT INTO box_analytics (session_id, box_id, color, size, detection_time) VALUES ("
                          + std::to_string(session_id) + ", '" + escape(box_id) + "', '" + escape(color) + "', '" + escape(size) + "', '" + escape(timestamp) + "');";
        return exec(sql);
    }

    bool logPick(const std::string &box_id, const std::string &timestamp) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return false;
        std::string sql = "UPDATE box_analytics SET pick_time='" + escape(timestamp) + "' WHERE box_id='" + escape(box_id) + "';";
        return exec(sql);
    }

    // When logging place, also compute cycle_duration in seconds using detection_time
    bool logPlace(const std::string &box_id, const std::string &timestamp) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return false;
        // Update place_time
        std::string sql_upd = "UPDATE box_analytics SET place_time='" + escape(timestamp) + "' WHERE box_id='" + escape(box_id) + "';";
        if (!exec(sql_upd)) return false;

        // Compute cycle_duration using julianday difference
        std::string sql_calc = "UPDATE box_analytics SET cycle_duration=(julianday(place_time)-julianday(detection_time))*86400.0 WHERE box_id='" + escape(box_id) + "' AND detection_time IS NOT NULL AND place_time IS NOT NULL;";
        return exec(sql_calc);
    }

private:
    bool exec(const std::string &sql) {
        if (!db_) return false;
        char *errmsg = nullptr;
        if (sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &errmsg) != SQLITE_OK) {
            RCLCPP_ERROR(logger_, "AnalyticsDB SQL error: %s", errmsg ? errmsg : "(unknown)");
            if (errmsg) sqlite3_free(errmsg);
            return false;
        }
        return true;
    }

    std::string escape(const std::string &s) const {
        std::string out; out.reserve(s.size()*2);
        for (char c : s) {
            if (c == '\'') out.push_back('\''), out.push_back('\'');
            else out.push_back(c);
        }
        return out;
    }

    rclcpp::Logger logger_;
    std::string db_path_;
    sqlite3 *db_;
    std::mutex m_;
};

// ----------------- PipelineFSM (modified) -----------------

class PipelineFSM : public rclcpp::Node {
public:
    PipelineFSM()
    : rclcpp::Node("pipeline_fsm_node"),
      target_db_(this->get_logger()),
      motion_(this, "arm"),
      gripper_(this),
      conveyor_(this),
      vision_(this),
      analytics_(this->get_logger(), "analytics.db") {

        // Parameters (targets_json_path now has a relative default)
        targets_json_path_ = this->declare_parameter<std::string>(
          "targets_json_path", "robot_data/Targets.json");
        pub_event_ = this->create_publisher<std_msgs::msg::String>("/analytics/event", 10);

        kVisionWindowSec_      = this->declare_parameter<double>("vision_window_sec", 7.0);
        kVisionWarmupSec_      = this->declare_parameter<double>("vision_warmup_sec", 5.0);
        kStablePosTol_         = this->declare_parameter<double>("vision_stable_tol_m", 0.01);
        kStableMinCount_       = this->declare_parameter<int>("vision_stable_count", 10);
        kColorGraceSec_        = this->declare_parameter<double>("vision_color_grace_sec", 12.0);

        kConveyor2Step_        = this->declare_parameter<double>("conveyor2_step", 0.265);
        kConveyor1Step_        = this->declare_parameter<double>("conveyor1_step", 0.26);
        kConveyor3Step_        = this->declare_parameter<double>("conveyor3_step", 0.26);

        init_feed_times_       = this->declare_parameter<int>("initial_feed_times", 3);

        initial_feed_gap_sec_ = this->declare_parameter<double>("initial_feed_gap_sec", 3.0);
        tail_feed_gap_sec_     = this->declare_parameter<double>("tail_feed_gap_sec", 2.0);
        cycle_gap_sec_         = this->declare_parameter<double>("cycle_gap_sec", 3.0);

        if (!target_db_.load(targets_json_path_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load Targets.json from '%s'", targets_json_path_.c_str());
        }

        speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/pipeline/speed", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg){
                current_speed_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "Pipeline speed set to %.2fx", current_speed_);
            }
        );
        
        // Subscriber for box size info
        box_info_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/detected_box_info", 10,
            std::bind(&PipelineFSM::onBoxInfo, this, std::placeholders::_1));

        const double default_speed = this->declare_parameter<double>("default_speed_scale", 1.0);
        motion_.setSpeedScale(std::clamp(default_speed, 0.1, 1.0));

        box_count_pub_ = this->create_publisher<std_msgs::msg::Int32>("/pipeline/box_count", 10);
        box_count_ = 0;
        publishBoxCount();  

        start_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "start_pipeline",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                RCLCPP_INFO(this->get_logger(), "[SERVICE] start_pipeline requested");
                if (!target_db_.loaded()) {
                    if (!target_db_.load(targets_json_path_)) {
                        res->success = false; res->message = "Targets.json load failed";
                        return;
                    }
                }
                stop_requested_.store(false);
                motion_.clearStop();
                vision_.clearStop();
                running_.store(true);

                // Start new analytics session
                std::string t0 = now_local_str();
                ++pipeline_session_counter_;
                std::string session_name = "pipeline_" + std::to_string(pipeline_session_counter_);
                analytics_session_id_ = analytics_.startSession(session_name, t0);
                if (analytics_session_id_ < 0) RCLCPP_WARN(this->get_logger(), "Analytics session failed to start");
                RCLCPP_INFO(this->get_logger(), "Starting new pipeline session: %s", session_name.c_str());

                res->success = true;
                res->message = "Pipeline running";
                RCLCPP_INFO(this->get_logger(), "[SERVICE] start_pipeline: RUNNING (analytics session=%d)", analytics_session_id_);
            });

        stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_pipeline",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                RCLCPP_WARN(this->get_logger(), "[SERVICE] stop_pipeline requested");
                stop_requested_.store(true);
                running_.store(false);
                motion_.requestStop();
                vision_.requestStop();

                // end analytics session
                std::string tend = now_local_str();
                if (analytics_session_id_ >= 0) analytics_.endSession(analytics_session_id_, tend);

                res->success = true;
                res->message = "Pipeline stopped";
                RCLCPP_WARN(this->get_logger(), "[SERVICE] stop_pipeline: STOPPED (analytics session ended)");
            });

        // FSM init
        state_ = State::INIT_FEED;
        cycles_done_ = 0;
        init_feed_remaining_ = init_feed_times_;
        pipeline_session_counter_ = 0;

        RCLCPP_INFO(get_logger(), "PipelineFSM ready. Targets path: %s", targets_json_path_.c_str());
    }

    void spinLoop() {
        rclcpp::Rate r(50);
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            if (running_.load()) tick();
            r.sleep();
        }
    }

private:
    enum class State {
        INIT_FEED, VISION_FIXED, CYCLE_VISION, START_PICK, ATTACH, MOVE_UP, FEED_TAIL,
        ROUTE_PRE, ROUTE_FINAL, DETACH, NUDGE_OUT, RETURN_BACK, RETURN_HOME, NEXT_CYCLE,
        WAIT
    };

    void onBoxInfo(const std_msgs::msg::String::SharedPtr msg) {
        // Parse the box size (L x W) from the detected_box_info message.
        std::string s = msg->data;
        std::string search_str = "Size (L x W ): ";
        size_t start_pos = s.find(search_str);
        if (start_pos != std::string::npos) {
            start_pos += search_str.length();
            size_t end_pos = s.find("\n", start_pos);
            if (end_pos != std::string::npos) {
                detected_box_size_ = s.substr(start_pos, end_pos - start_pos);
            }
        }
    }

    std::string now_local_str() const {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
    #if defined(_WIN32)
        localtime_s(&tm, &t);
    #else
        localtime_r(&t, &tm);
    #endif
        char buf[32];
        std::strftime(buf, sizeof(buf), "%F %T", &tm);
        return std::string(buf);
    }

    void beginWait(double seconds, State after) {
        wait_until_ = this->now() + rclcpp::Duration::from_seconds(seconds);
        after_wait_ = after;
        waiting_ = true;
        state_ = State::WAIT;
        RCLCPP_INFO(this->get_logger(), "STATE -> WAIT (%.2fs) then %s", seconds, stateName(after));
    }

    void stepWait() {
        if (!waiting_) { state_ = after_wait_; return; }
        if (this->now() >= wait_until_) {
            waiting_ = false;
            RCLCPP_INFO(this->get_logger(), "WAIT done -> %s", stateName(after_wait_));
            state_ = after_wait_;
        }
    }

    const char* stateName(State s) const {
        switch (s) {
            case State::INIT_FEED: return "INIT_FEED";
            case State::VISION_FIXED: return "VISION_FIXED";
            case State::CYCLE_VISION: return "CYCLE_VISION";
            case State::START_PICK: return "START_PICK";
            case State::ATTACH: return "ATTACH";
            case State::MOVE_UP: return "MOVE_UP";
            case State::FEED_TAIL: return "FEED_TAIL";
            case State::ROUTE_PRE: return "ROUTE_PRE";
            case State::ROUTE_FINAL: return "ROUTE_FINAL";
            case State::DETACH: return "DETACH";
            case State::NUDGE_OUT: return "NUDGE_OUT";
            case State::RETURN_BACK: return "RETURN_BACK";
            case State::RETURN_HOME: return "RETURN_HOME";
            case State::NEXT_CYCLE: return "NEXT_CYCLE";
            case State::WAIT: return "WAIT";
        }
        return "UNKNOWN";
    }

    void tick() {
        if (stop_requested_.load()) {
            motion_.requestStop();
            vision_.requestStop();
            return;
        }
        switch (state_) {
            case State::INIT_FEED:      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "STATE: INIT_FEED");    stepInitFeed();    break;
            case State::VISION_FIXED: RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "STATE: VISION_FIXED"); stepVisionFixed(); break;
            case State::CYCLE_VISION: RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "STATE: CYCLE_VISION"); stepCycleVision(); break;
            case State::START_PICK:     RCLCPP_INFO(get_logger(), "STATE: START_PICK");    stepStartPick();      break;
            case State::ATTACH:         RCLCPP_INFO(get_logger(), "STATE: ATTACH");        stepAttach();        break;
            case State::MOVE_UP:        RCLCPP_INFO(get_logger(), "STATE: MOVE_UP");      stepMoveUp();        break;
            case State::FEED_TAIL:      RCLCPP_INFO(get_logger(), "STATE: FEED_TAIL");    stepFeedTail();    break;
            case State::ROUTE_PRE:      RCLCPP_INFO(get_logger(), "STATE: ROUTE_PRE");    stepRoutePre();    break;
            case State::ROUTE_FINAL:    RCLCPP_INFO(get_logger(), "STATE: ROUTE_FINAL");  stepRouteFinal();  break;
            case State::DETACH:         RCLCPP_INFO(get_logger(), "STATE: DETACH");        stepDetach();        break;
            case State::NUDGE_OUT:      RCLCPP_INFO(get_logger(), "STATE: NUDGE_OUT");    stepNudgeOut();    break;
            case State::RETURN_BACK:    RCLCPP_INFO(get_logger(), "STATE: RETURN_BACK");  stepReturnBack();  break;
            case State::RETURN_HOME:    RCLCPP_INFO(get_logger(), "STATE: RETURN_HOME");  stepReturnHome();  break;
            case State::NEXT_CYCLE:     RCLCPP_INFO(get_logger(), "STATE: NEXT_CYCLE");    stepNextCycle();    break;
            case State::WAIT:           stepWait();             break;
        }
    }

    void stepCycleVision() {
        auto vd = vision_.runWindow(kVisionWarmupSec_, kVisionWindowSec_, kStablePosTol_, kStableMinCount_, kColorGraceSec_);
        if (!vd) {
            RCLCPP_WARN(get_logger(), "CYCLE_VISION: no detection; retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return;
        }
        det_pose_ = vd->first;
        det_color_ = vd->second;
        model_name_ = nextModelName(det_color_);
        RCLCPP_INFO(get_logger(), "CYCLE_VISION: color=%s model=%s, size=%s", det_color_.c_str(), model_name_.c_str(), detected_box_size_.c_str());

        // Log detection in DB with dynamic size
        if (analytics_session_id_ >= 0) {
            analytics_.logDetection(analytics_session_id_, model_name_, det_color_, detected_box_size_, now_local_str());
        }

        state_ = State::START_PICK;
    }

    void stepInitFeed() {
        if (init_feed_remaining_ <= 0) {
            RCLCPP_INFO(get_logger(), "INIT_FEED done -> VISION_FIXED");
            state_ = State::VISION_FIXED; return;
        }

        if (vision_.spawnBox()) {
            ++box_count_;
            publishBoxCount();
        } else {
            RCLCPP_ERROR(get_logger(), "INIT_FEED: spawn failed");
            running_.store(false);
            return;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!conveyor_.move("/conveyor2/MoveDistance", kConveyor2Step_)) { running_.store(false); return; }

        --init_feed_remaining_;
        if (init_feed_remaining_ > 0) {
            beginWait(initial_feed_gap_sec_, State::INIT_FEED);
        } else {
            beginWait(initial_feed_gap_sec_, State::VISION_FIXED);
        }
    }

    void stepVisionFixed() {
        auto vd = vision_.runFixed(4.0, 2.0);
        if (!vd) { 
            RCLCPP_ERROR(get_logger(), "VISION_FIXED: no detection"); 
            running_.store(false); 
            return; 
        }
        det_pose_ = vd->first;
        det_color_ = vd->second;
        model_name_ = nextModelName(det_color_);
        RCLCPP_INFO(get_logger(), "VISION_FIXED: color=%s model=%s, size=%s", det_color_.c_str(), model_name_.c_str(), detected_box_size_.c_str());

        // Publish detection event
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = R"({"event": "detection", "model": ")" + model_name_ + R"(", "color": ")" + det_color_ + R"(", "size": ")" + detected_box_size_ + R"(", "timestamp": ")" + now_local_str() + R"("})";
        pub_event_->publish(*msg);

        // Log detection in DB with dynamic size
        if (analytics_session_id_ >= 0) {
            analytics_.logDetection(analytics_session_id_, model_name_, det_color_, detected_box_size_, now_local_str());
        }

        state_ = State::START_PICK;
    }

    void stepStartPick() {
        RCLCPP_INFO(get_logger(), "START_PICK: triggering /start_picking");
        (void)gripper_.startPicking(30);

        // Publish pick event
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = R"({"event": "pick", "model": ")" + model_name_ + R"(", "color": ")" + det_color_ + R"(", "size": ")" + detected_box_size_ + R"(", "timestamp": ")" + now_local_str() + R"("})";
        pub_event_->publish(*msg);

        // Log pick time
        if (analytics_session_id_ >= 0) {
            analytics_.logPick(model_name_, now_local_str());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        state_ = State::ATTACH;
    }

    void stepAttach() {
        RCLCPP_INFO(get_logger(), "ATTACH: model=%s", model_name_.c_str());
        if (!gripper_.attach(model_name_)) {
            RCLCPP_WARN(get_logger(), "ATTACH failed (continuing anyway)");
        }
        state_ = State::MOVE_UP;
    }

    bool waitControllerReady(double timeout_sec = 2.0) {
        auto t0 = now();
        while ((now() - t0).seconds() < timeout_sec) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (stop_requested_.load()) return false;
        }
        return true;
    }

    void stepMoveUp() {
        RCLCPP_INFO(get_logger(), "MOVE_UP: checking targets at '%s'", targets_json_path_.c_str());
        if (!target_db_.loaded()) {
            RCLCPP_ERROR(get_logger(), "MOVE_UP: Targets.json not loaded!");
            running_.store(false);
            return;
        }
        std::vector<double> j;
        if (!target_db_.getJointTarget("up", j)) {
            RCLCPP_ERROR(get_logger(), "MOVE_UP: target 'up' not found in Targets.json!");
            running_.store(false);
            return;
        }

        (void)waitControllerReady(1.0);

        RCLCPP_INFO(get_logger(), "MOVE_UP: executing 6-joint target (speed %.2fx)", speed_scale_.load());
        if (!motion_.planAndExecuteJoints(j, "move_up")) {
            RCLCPP_ERROR(get_logger(), "MOVE_UP: plan/execute failed (controller busy or planning error)");
            running_.store(false);
            return;
        }
        RCLCPP_INFO(get_logger(), "MOVE_UP: success");
        state_ = State::FEED_TAIL;
    }

    void stepFeedTail() {
        RCLCPP_INFO(get_logger(), "FEED_TAIL: spawn+nudge conveyor2");
        if (vision_.spawnBox()) {
            ++box_count_;
            publishBoxCount();
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        (void)conveyor_.move("/conveyor2/MoveDistance", kConveyor2Step_);
        beginWait(tail_feed_gap_sec_, State::ROUTE_PRE);
    }

    void stepRoutePre() {
        const std::string pre = (det_color_=="red") ? "conveyor_1_pre" : "conveyor_3_pre";
        RCLCPP_INFO(get_logger(), "ROUTE_PRE: %s", pre.c_str());
        std::vector<double> j;
        if (!target_db_.getJointTarget(pre, j)) { RCLCPP_ERROR(get_logger(), "ROUTE_PRE: '%s' missing", pre.c_str()); running_.store(false); return; }
        if (!motion_.planAndExecuteJoints(j, "route_pre")) { running_.store(false); return; }
        state_ = State::ROUTE_FINAL;
    }

    void stepRouteFinal() {
        const std::string fin = (det_color_=="red") ? "conveyor_1" : "conveyor_3";
        RCLCPP_INFO(get_logger(), "ROUTE_FINAL: %s", fin.c_str());
        std::vector<double> j;
        if (!target_db_.getJointTarget(fin, j)) { RCLCPP_ERROR(get_logger(), "ROUTE_FINAL: '%s' missing", fin.c_str()); running_.store(false); return; }
        if (!motion_.planAndExecuteJoints(j, "route_final")) { running_.store(false); return; }
        state_ = State::DETACH;
    }

    void stepDetach() {
        RCLCPP_INFO(get_logger(), "DETACH: model=%s", model_name_.c_str());
        if (!gripper_.detach(model_name_)) { RCLCPP_WARN(get_logger(), "DETACH failed (continuing)"); }
        state_ = State::NUDGE_OUT;
    }

    void stepNudgeOut() {
        if (det_color_=="red") { (void)conveyor_.move("/conveyor1/MoveDistance", kConveyor1Step_); }
        else                   { (void)conveyor_.move("/conveyor3/MoveDistance", kConveyor3Step_); }
        state_ = State::RETURN_BACK;
    }

    void stepReturnBack() {
        const std::string fin = (det_color_=="red") ? "conveyor_1" : "conveyor_3";
        const std::string pre = (det_color_=="red") ? "conveyor_1_pre" : "conveyor_3_pre";
        std::vector<double> jf, jp;
        if (!target_db_.getJointTarget(fin, jf) || !target_db_.getJointTarget(pre, jp)) {
            RCLCPP_ERROR(get_logger(), "RETURN_BACK: targets missing"); running_.store(false); return;
        }
        if (!motion_.planAndExecuteJoints(jf, "return_fin")) { running_.store(false); return; }
        if (!motion_.planAndExecuteJoints(jp, "return_pre")) { running_.store(false); return; }

        // Publish place event
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = R"({"event": "place", "model": ")" + model_name_ + R"(", "color": ")" + det_color_ + R"(", "size": ")" + detected_box_size_ + R"(", "timestamp": ")" + now_local_str() + R"("})";
        pub_event_->publish(*msg);

        // Log place & compute duration in DB
        if (analytics_session_id_ >= 0) {
            analytics_.logPlace(model_name_, now_local_str());
        }

        state_ = State::RETURN_HOME;
    }

    void stepReturnHome() {
        std::vector<double> j;
        if (!target_db_.getJointTarget("home", j)) {
            RCLCPP_WARN(get_logger(), "RETURN_HOME: 'home' missing; keeping last pose.");
            state_ = State::NEXT_CYCLE; return;
        }
        if (!motion_.planAndExecuteJoints(j, "return_home")) { running_.store(false); return; }
        state_ = State::NEXT_CYCLE;
    }

    void stepNextCycle() {
        cycles_done_ += 1;
        RCLCPP_INFO(get_logger(), "Cycle %d complete.", cycles_done_);
        beginWait(cycle_gap_sec_, State::CYCLE_VISION);
    }

    std::string nextModelName(const std::string &color) {
        auto it = color_counts_.find(color);
        if (it == color_counts_.end()) color_counts_[color] = 0;
        color_counts_[color] += 1;
        std::string cap = color;
        cap[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(cap[0])));
        return cap + "_" + std::to_string(color_counts_[color]);
    }

    void publishBoxCount() {
        auto m = std_msgs::msg::Int32();
        m.data = box_count_;
        box_count_pub_->publish(m);
        RCLCPP_INFO(get_logger(), "Published box count: %d", box_count_);
    }

    // members (same as before) + analytics
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_event_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr box_info_sub_;
    double current_speed_ = 1.0;  // default speed
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr    box_count_pub_;
    std::atomic<double> speed_scale_{1.0};
    int box_count_{0};

    TargetDB             target_db_;
    MotionController motion_;
    Gripper              gripper_;
    Conveyor             conveyor_;
    VisionManager      vision_;
    AnalyticsDB        analytics_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    State state_;
    int cycles_done_{0};
    int pipeline_session_counter_;
    std::string detected_box_size_;

    std::string targets_json_path_;
    int init_feed_times_{3};
    int init_feed_remaining_{3};

    double kVisionWindowSec_;
    double kVisionWarmupSec_;
    double kStablePosTol_;
    int    kStableMinCount_;
    double kColorGraceSec_;
    double kConveyor2Step_;
    double kConveyor1Step_;
    double kConveyor3Step_;

    double initial_feed_gap_sec_{3.0};
    double tail_feed_gap_sec_{2.0};
    double cycle_gap_sec_{3.0};

    rclcpp::Time wait_until_;
    State        after_wait_{State::CYCLE_VISION};
    bool         waiting_{false};

    geometry_msgs::msg::PoseStamped det_pose_;
    std::string det_color_;
    std::string model_name_;
    std::unordered_map<std::string,int> color_counts_{{"red",0},{"blue",0}};

    int analytics_session_id_{-1};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PipelineFSM>();
    node->spinLoop();
    rclcpp::shutdown();
    return 0;
}
