#include "parol6_pipeline/pipeline_subsystems.hpp"
#include <unordered_map>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

class PipelineFSM : public rclcpp::Node {
public:
  PipelineFSM()
  : rclcpp::Node("pipeline_fsm_node"),
    target_db_(this->get_logger()),
    motion_(this, "arm"),
    gripper_(this),
    conveyor_(this),
    vision_(this) {

    // Parameters
    targets_json_path_ = this->declare_parameter<std::string>(
      "targets_json_path", "/home/azif/projetcs/parol6/robot_data/Targets.json");
    pub_event_ = this->create_publisher<std_msgs::msg::String>("/analytics/event", 10);

    kVisionWindowSec_   = this->declare_parameter<double>("vision_window_sec", 7.0);
    kVisionWarmupSec_   = this->declare_parameter<double>("vision_warmup_sec", 5.0);
    kStablePosTol_      = this->declare_parameter<double>("vision_stable_tol_m", 0.01);
    kStableMinCount_    = this->declare_parameter<int>("vision_stable_count", 10);
    kColorGraceSec_     = this->declare_parameter<double>("vision_color_grace_sec", 12.0);

    kConveyor2Step_     = this->declare_parameter<double>("conveyor2_step", 0.265);
    kConveyor1Step_     = this->declare_parameter<double>("conveyor1_step", 0.26);
    kConveyor3Step_     = this->declare_parameter<double>("conveyor3_step", 0.26);

    init_feed_times_    = this->declare_parameter<int>("initial_feed_times", 3);

    // Tunable gaps (optional; keep defaults if you don't need spacing)
    initial_feed_gap_sec_ = this->declare_parameter<double>("initial_feed_gap_sec", 3.0);
    tail_feed_gap_sec_    = this->declare_parameter<double>("tail_feed_gap_sec", 2.0);
    cycle_gap_sec_        = this->declare_parameter<double>("cycle_gap_sec", 3.0);

    // Load targets early (and loudly)
    if (!target_db_.load(targets_json_path_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load Targets.json from '%s'", targets_json_path_.c_str());
    }

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
        res->success = true;
        res->message = "Pipeline running";
        RCLCPP_INFO(this->get_logger(), "[SERVICE] start_pipeline: RUNNING");
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
        res->success = true;
        res->message = "Pipeline stopped";
        RCLCPP_WARN(this->get_logger(), "[SERVICE] stop_pipeline: STOPPED");
      });

    // FSM init
    state_ = State::INIT_FEED;
    cycles_done_ = 0;
    init_feed_remaining_ = init_feed_times_;

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

  // Helper function to get the current local time
  std::string now_local_str() const {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm); // for Linux and MacOS, use `localtime_r`
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
      case State::INIT_FEED:    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "STATE: INIT_FEED");    stepInitFeed();    break;
      case State::VISION_FIXED: RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "STATE: VISION_FIXED"); stepVisionFixed(); break;
      case State::CYCLE_VISION: RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "STATE: CYCLE_VISION"); stepCycleVision(); break;
      case State::START_PICK:   RCLCPP_INFO(get_logger(), "STATE: START_PICK");   stepStartPick();   break;
      case State::ATTACH:       RCLCPP_INFO(get_logger(), "STATE: ATTACH");       stepAttach();      break;
      case State::MOVE_UP:      RCLCPP_INFO(get_logger(), "STATE: MOVE_UP");      stepMoveUp();      break;
      case State::FEED_TAIL:    RCLCPP_INFO(get_logger(), "STATE: FEED_TAIL");    stepFeedTail();    break;
      case State::ROUTE_PRE:    RCLCPP_INFO(get_logger(), "STATE: ROUTE_PRE");    stepRoutePre();    break;
      case State::ROUTE_FINAL:  RCLCPP_INFO(get_logger(), "STATE: ROUTE_FINAL");  stepRouteFinal();  break;
      case State::DETACH:       RCLCPP_INFO(get_logger(), "STATE: DETACH");       stepDetach();      break;
      case State::NUDGE_OUT:    RCLCPP_INFO(get_logger(), "STATE: NUDGE_OUT");    stepNudgeOut();    break;
      case State::RETURN_BACK:  RCLCPP_INFO(get_logger(), "STATE: RETURN_BACK");  stepReturnBack();  break;
      case State::RETURN_HOME:  RCLCPP_INFO(get_logger(), "STATE: RETURN_HOME");  stepReturnHome();  break;
      case State::NEXT_CYCLE:   RCLCPP_INFO(get_logger(), "STATE: NEXT_CYCLE");   stepNextCycle();   break;
      case State::WAIT:         stepWait();        break;
    }
  }

  void stepCycleVision() {
    auto vd = vision_.runWindow(kVisionWarmupSec_, kVisionWindowSec_, kStablePosTol_, kStableMinCount_, kColorGraceSec_);
    if (!vd) {
      RCLCPP_WARN(get_logger(), "CYCLE_VISION: no detection; retrying...");
      std::this_thread::sleep_for(500ms);
      return;
    }
    det_pose_  = vd->first;
    det_color_ = vd->second;
    model_name_ = nextModelName(det_color_);
    RCLCPP_INFO(get_logger(), "CYCLE_VISION: color=%s model=%s", det_color_.c_str(), model_name_.c_str());
    state_ = State::START_PICK;
  }


  void stepInitFeed() {
    if (init_feed_remaining_ <= 0) {
      RCLCPP_INFO(get_logger(), "INIT_FEED done -> VISION_FIXED");
      state_ = State::VISION_FIXED; return;
    }
    if (!vision_.spawnBox()) { RCLCPP_ERROR(get_logger(), "INIT_FEED: spawn failed"); running_.store(false); return; }
    std::this_thread::sleep_for(1s);
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
    det_pose_  = vd->first;
    det_color_ = vd->second;
    model_name_ = nextModelName(det_color_);
    RCLCPP_INFO(get_logger(), "VISION_FIXED: color=%s model=%s", det_color_.c_str(), model_name_.c_str());

    // Publish detection event
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = R"({"event": "detection", "model": ")" + model_name_ + R"(", "color": ")" + det_color_ + R"(", "size": "small", "timestamp": ")" + now_local_str() + R"("})";
    pub_event_->publish(*msg); // Assuming pub_event_ is properly initialized

    state_ = State::START_PICK;
  }

  void stepStartPick() {
    RCLCPP_INFO(get_logger(), "START_PICK: triggering /start_picking");
    (void)gripper_.startPicking(30);

    // Publish pick event
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = R"({"event": "pick", "model": ")" + model_name_ + R"(", "color": ")" + det_color_ + R"(", "size": "small", "timestamp": ")" + now_local_str() + R"("})";
    pub_event_->publish(*msg); // Publish pick event

    std::this_thread::sleep_for(400ms);
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
      std::this_thread::sleep_for(50ms);
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

    RCLCPP_INFO(get_logger(), "MOVE_UP: executing 6-joint target");
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
    (void)vision_.spawnBox();
    std::this_thread::sleep_for(1s);
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
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = R"({"event": "place", "model": ")" + model_name_ + R"(", "color": ")" + det_color_ + R"(", "size": "small", "timestamp": ")" + now_local_str() + R"("})";
    pub_event_->publish(*msg); // Publish place event

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

  // services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_event_;

  // subsystems
  TargetDB         target_db_;
  MotionController motion_;
  Gripper          gripper_;
  Conveyor         conveyor_;
  VisionManager    vision_;

  // fsm & control
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  State state_;
  int cycles_done_{0};

  // params
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

  // timing params
  double initial_feed_gap_sec_{3.0};
  double tail_feed_gap_sec_{2.0};
  double cycle_gap_sec_{3.0};

  // wait machinery
  rclcpp::Time wait_until_;
  State        after_wait_{State::CYCLE_VISION};
  bool         waiting_{false};

  // cycle context
  geometry_msgs::msg::PoseStamped det_pose_;
  std::string det_color_;
  std::string model_name_;
  std::unordered_map<std::string,int> color_counts_{{"red",0},{"blue",0}};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PipelineFSM>();
  node->spinLoop();
  rclcpp::shutdown();
  return 0;
}
