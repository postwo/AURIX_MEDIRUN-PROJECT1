#include "arduino_bridge/ultrasonic_safety_policy.hpp"

#include <cmath>

namespace arduino_bridge {

bool SafetyDecision::shouldStop() const noexcept {
  return reason != StopReason::kNone;
}

UltrasonicSafetyPolicy::UltrasonicSafetyPolicy(const SafetyConfig &config)
    : config_(config) {}

void UltrasonicSafetyPolicy::setConfig(const SafetyConfig &config) noexcept {
  config_ = config;
}

const SafetyConfig &UltrasonicSafetyPolicy::config() const noexcept {
  return config_;
}

void UltrasonicSafetyPolicy::updateDistance(SensorSide side, int distance_cm,
                                            TimeNanoseconds now_ns) {
  auto &state = stateFor(side);
  state.distance_cm = distance_cm;
  state.has_reading = true;
  state.last_update_ns = now_ns;
  refreshState(side, now_ns);
}

SafetyDecision
UltrasonicSafetyPolicy::filterCommand(const VelocityCommand &command,
                                      TimeNanoseconds now_ns) {
  last_command_ = command;
  has_last_command_ = true;

  // 속도 메시지 시점에도 양쪽 상태를 다시 평가한다. 센서값은 그대로여도
  // hold_duration_sec가 끝났거나 런타임 파라미터가 바뀐 경우를 반영하기
  // 위해서다.
  refreshState(SensorSide::kFront, now_ns);
  refreshState(SensorSide::kRear, now_ns);

  SafetyDecision decision{command, evaluateStopReason(command)};
  if (decision.shouldStop()) {
    decision.command.linear_x = 0.0;
    decision.command.angular_z = 0.0;
  }
  return decision;
}

std::optional<StopReason>
UltrasonicSafetyPolicy::immediateStopReason() const noexcept {
  if (!has_last_command_) {
    return std::nullopt;
  }

  const StopReason reason = evaluateStopReason(last_command_);
  if (reason == StopReason::kNone) {
    return std::nullopt;
  }
  return reason;
}

std::optional<StopReason>
UltrasonicSafetyPolicy::pollStopReason(TimeNanoseconds now_ns) {
  refreshState(SensorSide::kFront, now_ns);
  refreshState(SensorSide::kRear, now_ns);
  return immediateStopReason();
}

int UltrasonicSafetyPolicy::distanceCm(SensorSide side) const noexcept {
  return stateFor(side).distance_cm;
}

bool UltrasonicSafetyPolicy::isHolding(SensorSide side) const noexcept {
  return stateFor(side).holding;
}

bool UltrasonicSafetyPolicy::isStale(SensorSide side) const noexcept {
  return stateFor(side).stale;
}

UltrasonicSafetyPolicy::ProximityState &
UltrasonicSafetyPolicy::stateFor(SensorSide side) noexcept {
  return side == SensorSide::kFront ? front_ : rear_;
}

const UltrasonicSafetyPolicy::ProximityState &
UltrasonicSafetyPolicy::stateFor(SensorSide side) const noexcept {
  return side == SensorSide::kFront ? front_ : rear_;
}

int UltrasonicSafetyPolicy::stopDistanceCm(SensorSide side) const noexcept {
  return side == SensorSide::kFront ? config_.front_stop_distance_cm
                                    : config_.rear_stop_distance_cm;
}

void UltrasonicSafetyPolicy::refreshState(SensorSide side,
                                          TimeNanoseconds now_ns) {
  auto &state = stateFor(side);
  if (config_.sensor_timeout_sec > 0.0) {
    const double age_sec =
        static_cast<double>(now_ns - state.last_update_ns) / 1'000'000'000.0;
    state.stale = !state.has_reading || age_sec > config_.sensor_timeout_sec;
  } else {
    state.stale = false;
  }

  if (state.distance_cm <= stopDistanceCm(side)) {
    state.holding = true;
    state.hold_started_ns = now_ns;
    return;
  }

  const double elapsed_sec =
      static_cast<double>(now_ns - state.hold_started_ns) / 1'000'000'000.0;
  if (state.holding && elapsed_sec < config_.hold_duration_sec) {
    return;
  }
  state.holding = false;
}

bool UltrasonicSafetyPolicy::isBlocking(
    const ProximityState &state) const noexcept {
  // 값이 끊긴 센서는 "안 보인다"가 아니라 "알 수 없다"이므로 막는 쪽으로 본다.
  return state.holding || state.stale;
}

StopReason UltrasonicSafetyPolicy::evaluateStopReason(
    const VelocityCommand &command) const noexcept {
  const bool front_blocking = isBlocking(front_) && command.linear_x > 0.0;
  const bool rear_blocking = isBlocking(rear_) && command.linear_x < 0.0;
  // 순수 제자리 회전(선속도 0)일 때만 전후방 상관없이 둘 다 체크한다.
  // 전진/후진에 조향이 섞인 경우는 진행 방향 센서만 본다 — 그렇지 않으면
  // 전진 중 후방 센서가 걸려 계속 멈추는 문제가 있었다(2026-08-19).
  const bool in_place_rotation = std::abs(command.linear_x) <= kMotionEpsilon &&
                                 std::abs(command.angular_z) > kMotionEpsilon;
  const bool rotation_blocking =
      in_place_rotation && (isBlocking(front_) || isBlocking(rear_));

  if (rotation_blocking) {
    return StopReason::kRotationObstacle;
  }
  if (front_blocking) {
    return StopReason::kFrontObstacle;
  }
  if (rear_blocking) {
    return StopReason::kRearObstacle;
  }
  return StopReason::kNone;
}

} // namespace arduino_bridge
