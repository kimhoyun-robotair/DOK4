// ============================================================================
// MultiLifecycleSupervisor (Refactored)
// ----------------------------------------------------------------------------
// 요구 흐름(사용자 지정 시나리오):
//  1. 초기 Phase에서 등록된 모든 Lifecycle 노드들을 CONFIGURE 전환 (병렬 가능)
//  2. 모든 노드가 Configured 되면 (조건 없이) offboard_control_node ACTIVATE
//  3. /aruco_tracker_node/ready (std_msgs/Bool) 가 true 로 관측되면 aruco_tracker_node ACTIVATE
//  4. aruco_tracker_node 가 Active 된 뒤 /precision_land_node/ready (std_msgs/Bool) 가 true 로 관측되면 precision_land_node ACTIVATE
//  5. precision_land_node 가 Active 되면 offboard_control_node DEACTIVATE
//  6. /precision_land_node/end (std_msgs/Bool) 가 true 가 되면 모든 노드 DEACTIVATE 후 CLEANUP
//
//  * enum Phase 는 기존 정의를 유지 (요구사항: enum class 고정) 하고, 의미를 본 시나리오에 맞추어 내부 처리만 수정.
//  * Ready / End 토픽은 아래 규칙 사용:
//      - 각 노드는 <real_node_name>/ready 를 latched(transient_local) Bool로 발행 (이미 존재)
//      - precision_land_node 는 landing 종료 시 <real_node_name>/end 를 Bool true 발행
//  * 신뢰성 향상을 위해:
//      - Transition 비동기 요청 중복 방지
//      - Pending transition polling
//      - Ready 값의 staleness 검출 (옵션: 사용자가 필요시 활성화)
//      - Phase 변화시 명확한 로그
//
// 주의: 본 예시는 기본 구조. 실제 환경에서 mission timeout, safety abort, 재시도(backoff) 등
//       추가 정책은 프로젝트 요구에 따라 확장.
// ============================================================================

#include <chrono>
#include <unordered_map>
#include <vector>
#include <future>
#include <optional>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState    = lifecycle_msgs::srv::GetState;

// ----------------------------------------------------------------------------
// Data structures
// ----------------------------------------------------------------------------
struct PendingTransition {
  std::string key;          // Managed node key
  uint8_t transition_id;    // Transition id
  rclcpp::Time start_time;  // Dispatch time
  rclcpp::Client<ChangeState>::SharedFuture future; // Future for async response
};

struct ManagedNode {
  std::string key;        // Internal key
  std::string name;       // Actual ROS node base name
  std::string change_srv; // change_state service name
  std::string get_srv;    // get_state service name
  rclcpp::Client<ChangeState>::SharedPtr change_cli;
  rclcpp::Client<GetState>::SharedPtr get_cli;

  // State flags (lightweight mirror of lifecycle state)
  bool configured = false;
  bool active = false;
  bool ready = false; // Updated when /<name>/ready true observed
  bool end_flag = false; // Only used for precision_land_node's /end topic

  rclcpp::Time last_ready_stamp; // For optional staleness checks
};

// ----------------------------------------------------------------------------
// Phase enum (Fixed: do not change names / order; only reinterpret meanings)
// ----------------------------------------------------------------------------
enum class Phase {
  INIT,                     // Wait minimal time then configure all
  CONFIG_ALL,               // Wait until all nodes configured
  ACTIVATE_FLIGHT_CONTROL,  // Activate offboard_control_node
  WAIT_MISSION_END_READY,   // Wait for /aruco_tracker_node/ready true
  ACTIVATE_ARUCO_TRACKER,   // Activate aruco_tracker_node
  ACTIVATE_PRECISION_LAND,  // Wait for /precision_land_node/ready true then activate
  WAIT_PRECISION_LAND_READY,// After precision_land_node active -> (reuse as synchronization gap)
  LANDING,                  // precision land active; deactivate offboard, wait landing end
  SHUTDOWN_CLEANUP,         // Deactivate + cleanup all
  ABORT                     // Abort (not elaborated here)
};

// ----------------------------------------------------------------------------
// Supervisor Node
// ----------------------------------------------------------------------------
class MultiLifecycleSupervisor : public rclcpp::Node {
public:
  MultiLifecycleSupervisor()
  : Node("multi_lc_supervisor")
  {
    // Register nodes (key, actual node name).
    add_node(OFFBOARD_KEY, "offboard_control", true);
    add_node(ARUCO_KEY,    "lifecycle_aruco_tracker", true);
    add_node(PLAND_KEY,    "precision_land_offboard_lifecycle", true);

    // Subscribe ready topics of all nodes.
    for (auto & kv : nodes_) {
      const auto topic = "/" + kv.second.name + "/ready";
      ready_subs_.push_back(
        create_subscription<std_msgs::msg::Bool>(
          topic, rclcpp::QoS(1).transient_local(),
          [this, key = kv.first](std_msgs::msg::Bool::SharedPtr msg){
            auto & n = nodes_[key];
            // We accept ready=true only; false can be ignored or used to reset state.
            if (msg->data) {
              n.ready = true;
              n.last_ready_stamp = now();
              RCLCPP_INFO(get_logger(), "[%s] READY received", key.c_str());
            }
          })
      );
    }

    // Subscribe end topic ONLY for precision landing node
    end_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/" + nodes_[PLAND_KEY].name + "/end", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr msg){
        if (msg->data) {
          nodes_[PLAND_KEY].end_flag = true;
          RCLCPP_INFO(get_logger(), "[%s] END flag observed", PLAND_KEY);
        }
      }
    );

    phase_ = Phase::INIT;
    phase_enter_time_ = now();

    tick_timer_ = create_wall_timer(100ms, std::bind(&MultiLifecycleSupervisor::tick, this));
    RCLCPP_INFO(get_logger(), "Supervisor initialized.");
  }

private:
  // --------------------------------------------------------------------------
  // Constants / Keys
  // --------------------------------------------------------------------------
  static constexpr const char * OFFBOARD_KEY = "offboard_control_node";
  static constexpr const char * ARUCO_KEY    = "aruco_tracker_node";
  static constexpr const char * PLAND_KEY    = "precision_land_node";

  // Timeouts (tunable)
  static constexpr std::chrono::seconds CONFIG_TIMEOUT{20};
  static constexpr std::chrono::seconds ACTIVATE_TIMEOUT{15};

  // --------------------------------------------------------------------------
  // Node registration
  // --------------------------------------------------------------------------
  void add_node(const std::string & key, const std::string & name, bool /*required*/) {
    ManagedNode mn;
    mn.key = key;
    mn.name = name;
    mn.change_srv = name + "/change_state";
    mn.get_srv    = name + "/get_state";
    mn.change_cli = create_client<ChangeState>(mn.change_srv);
    mn.get_cli    = create_client<GetState>(mn.get_srv);
    nodes_[key] = std::move(mn);
  }

  // --------------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------------
  rclcpp::Time now() { return get_clock()->now(); }
  rclcpp::Duration in_phase() { return now() - phase_enter_time_; }

  void change_phase(Phase next) {
    if (phase_ == next) return;
    RCLCPP_INFO(get_logger(), "PHASE %d -> %d", static_cast<int>(phase_), static_cast<int>(next));
    phase_ = next;
    phase_enter_time_ = now();
  }

  bool all_configured() const {
    for (auto & kv : nodes_) if (!kv.second.configured) return false; return true;
  }

  bool all_deactivated() const {
    for (auto & kv: nodes_) if (kv.second.active) return false; return true;
  }

  bool is_transition_pending(const std::string & key, uint8_t id) const {
    return std::any_of(pending_.begin(), pending_.end(), [&](auto & p){ return p.key==key && p.transition_id==id; });
  }

  void dispatch_transition(const std::string & key, uint8_t transition_id) {
    auto & mn = nodes_.at(key);
    if (is_transition_pending(key, transition_id)) return; // prevent duplicate
    if (!mn.change_cli->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "[%s] change_state service unavailable", key.c_str());
      return;
    }
    auto req = std::make_shared<ChangeState::Request>();
    req->transition.id = transition_id;
    auto future = mn.change_cli->async_send_request(req);
    pending_.push_back(PendingTransition{ key, transition_id, now(), future });
    RCLCPP_INFO(get_logger(), "[%s] Dispatch transition %u", key.c_str(), transition_id);
  }

  void poll_transitions() {
    auto it = pending_.begin();
    while (it != pending_.end()) {
      if (it->future.wait_for(0ms) == std::future_status::ready) {
        auto resp = it->future.get();
        auto & mn = nodes_.at(it->key);
        bool ok = resp->success;
        RCLCPP_INFO(get_logger(), "[%s] Transition %u result: %s", it->key.c_str(), it->transition_id, ok?"SUCCESS":"FAIL");
        if (ok) {
          switch (it->transition_id) {
            case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
              mn.configured = true; break;
            case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
              mn.configured = false; mn.active = false; mn.ready = false; break;
            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
              mn.active = true; break;
            case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
              mn.active = false; break;
            default: break;
          }
        } else {
          // 단순: 실패 시 즉시 ABORT (요구 사항에 재시도 없음)
          RCLCPP_ERROR(get_logger(), "[%s] Transition %u failed -> ABORT", it->key.c_str(), it->transition_id);
          change_phase(Phase::ABORT);
        }
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void abort_all_safe() {
    for (auto & kv : nodes_) {
      if (kv.second.active) {
        dispatch_transition(kv.first, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }
    }
  }

  // --------------------------------------------------------------------------
  // Main FSM tick
  // --------------------------------------------------------------------------
  void tick() {
    poll_transitions();

    switch (phase_) {
    case Phase::INIT: {
      // 바로 CONFIGURE 전송 (원한다면 소량 대기)
      if (in_phase() > 1s) {
        for (auto & kv : nodes_) {
          if (!kv.second.configured)
            dispatch_transition(kv.first, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        }
        change_phase(Phase::CONFIG_ALL);
      }
      break; }

    case Phase::CONFIG_ALL: {
      if (all_configured()) {
        // 조건 없이 offboard 활성화
        dispatch_transition(OFFBOARD_KEY, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        change_phase(Phase::ACTIVATE_FLIGHT_CONTROL);
      } else if (in_phase() > CONFIG_TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "CONFIG timeout -> ABORT");
        change_phase(Phase::ABORT);
      }
      break; }

    case Phase::ACTIVATE_FLIGHT_CONTROL: {
      auto & off = nodes_[OFFBOARD_KEY];
      // active 플래그는 transition 성공 시 설정됨
      if (off.active) {
        // 다음 단계: aruco_tracker_node ready 관측 대기
        change_phase(Phase::WAIT_MISSION_END_READY);
      } else if (in_phase() > ACTIVATE_TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "Offboard ACTIVATE timeout -> ABORT");
        change_phase(Phase::ABORT);
      }
      break; }

    case Phase::WAIT_MISSION_END_READY: {
      // 시나리오 정의: /aruco_tracker_node/ready true -> aruco ACTIVATE
      auto & aruco = nodes_[ARUCO_KEY];
      if (aruco.ready && !aruco.active) {
        dispatch_transition(ARUCO_KEY, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        change_phase(Phase::ACTIVATE_ARUCO_TRACKER);
      }
      break; }

    case Phase::ACTIVATE_ARUCO_TRACKER: {
      auto & aruco = nodes_[ARUCO_KEY];
      if (aruco.active) {
        change_phase(Phase::ACTIVATE_PRECISION_LAND);
      } else if (in_phase() > ACTIVATE_TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "Aruco ACTIVATE timeout -> ABORT");
        change_phase(Phase::ABORT);
      }
      break; }

    case Phase::ACTIVATE_PRECISION_LAND: {
      // /precision_land_node/ready true 관측되면 precision_land_node ACTIVATE
      auto & plan = nodes_[PLAND_KEY];
      if (plan.ready && !plan.active) {
        dispatch_transition(PLAND_KEY, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        change_phase(Phase::WAIT_PRECISION_LAND_READY);
      }
      break; }

    case Phase::WAIT_PRECISION_LAND_READY: {
      auto & plan = nodes_[PLAND_KEY];
      if (plan.active) {
        // precision land 활성화되었으므로 offboard 비활성화 (요구 5)
        dispatch_transition(OFFBOARD_KEY, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        change_phase(Phase::LANDING);
      } else if (in_phase() > ACTIVATE_TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "Precision land ACTIVATE timeout -> ABORT");
        change_phase(Phase::ABORT);
      }
      break; }

    case Phase::LANDING: {
      // landing 진행 중: /precision_land_node/end true 이면 종료 절차
      auto & plan = nodes_[PLAND_KEY];
      if (plan.end_flag) {
        // 모든 노드 deactivate -> cleanup
        for (auto & kv : nodes_) {
          if (kv.second.active)
            dispatch_transition(kv.first, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
        change_phase(Phase::SHUTDOWN_CLEANUP);
      }
      break; }

    case Phase::SHUTDOWN_CLEANUP: {
      if (all_deactivated()) {
        for (auto & kv : nodes_) {
          if (kv.second.configured)
            dispatch_transition(kv.first, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        }
        // Optional: 종료 상태 유지 혹은 노드 종료 유도
      }
      break; }

    case Phase::ABORT: {
      if (!abort_executed_) {
        RCLCPP_ERROR(get_logger(), "ABORT state entered. Attempting safe deactivation.");
        abort_all_safe();
        abort_executed_ = true;
      }
      break; }
    }
  }

  // --------------------------------------------------------------------------
  // Members
  // --------------------------------------------------------------------------
  Phase phase_;
  rclcpp::Time phase_enter_time_;

  std::unordered_map<std::string, ManagedNode> nodes_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> ready_subs_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr end_sub_;

  std::vector<PendingTransition> pending_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  bool abort_executed_ = false;
};

// ----------------------------------------------------------------------------
// main
// ----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiLifecycleSupervisor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
