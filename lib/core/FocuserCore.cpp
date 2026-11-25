#include "FocuserCore.h"

#include <cstring>

namespace pteroptyx {

namespace {
bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}
}

ZwoEafBackend::ZwoEafBackend(const char* id, Gpio& gpio, ZwoPins pins,
                             uint32_t minDurationMs, uint32_t maxDurationMs)
    : id_(id), gpio_(gpio), pins_(pins), minDurationMs_(minDurationMs),
      maxDurationMs_(maxDurationMs), direction_(Direction::Stop),
      deadlineMs_(0), moving_(false) {}

const char* ZwoEafBackend::id() const { return id_; }
const char* ZwoEafBackend::kind() const { return "zwo-eaf-isolated-closure"; }

void ZwoEafBackend::begin() {
  gpio_.modeOutput(pins_.speed);
  gpio_.modeOutput(pins_.up);
  gpio_.modeOutput(pins_.down);
  stop();
}

Result ZwoEafBackend::move(Direction direction, uint32_t durationMs, uint32_t nowMs) {
  if (direction == Direction::Stop || durationMs < minDurationMs_ ||
      durationMs > maxDurationMs_) {
    return Result::InvalidArgument;
  }
  stop();
  gpio_.write(pins_.speed, true);
  if (direction == Direction::Up) {
    gpio_.write(pins_.up, true);
    gpio_.write(pins_.down, false);
  } else {
    gpio_.write(pins_.down, true);
    gpio_.write(pins_.up, false);
  }
  direction_ = direction;
  deadlineMs_ = nowMs + durationMs;
  moving_ = true;
  return Result::Ok;
}

void ZwoEafBackend::stop() {
  gpio_.write(pins_.up, false);
  gpio_.write(pins_.down, false);
  gpio_.write(pins_.speed, false);
  moving_ = false;
  direction_ = Direction::Stop;
  deadlineMs_ = 0;
}

void ZwoEafBackend::tick(uint32_t nowMs, uint32_t nowUs) {
  (void)nowUs;
  if (moving_ && deadlineReached(nowMs, deadlineMs_)) stop();
}

Status ZwoEafBackend::status() const {
  return {id_, kind(), direction_, moving_, 0, PositionValidity::Unknown,
          deadlineMs_, false};
}

StepperBackend::StepperBackend(const char* id, Gpio& gpio, StepperPins pins,
                               uint32_t pulseIntervalUs, uint32_t maxDurationMs)
    : id_(id), gpio_(gpio), pins_(pins), pulseIntervalUs_(pulseIntervalUs),
      maxDurationMs_(maxDurationMs), direction_(Direction::Stop), deadlineMs_(0),
      nextPulseUs_(0), position_(0), validity_(PositionValidity::Estimated),
      moving_(false), stepHigh_(false) {}

const char* StepperBackend::id() const { return id_; }
const char* StepperBackend::kind() const { return "generic-step-dir"; }

void StepperBackend::begin() {
  gpio_.modeOutput(pins_.step);
  gpio_.modeOutput(pins_.direction);
  gpio_.modeOutput(pins_.enable);
  stop();
}

Result StepperBackend::move(Direction direction, uint32_t durationMs, uint32_t nowMs) {
  if (direction == Direction::Stop || durationMs == 0 || durationMs > maxDurationMs_) {
    return Result::InvalidArgument;
  }
  stop();
  direction_ = direction;
  gpio_.write(pins_.direction, direction == Direction::Up);
  gpio_.write(pins_.enable, !pins_.enableActiveLow);
  deadlineMs_ = nowMs + durationMs;
  nextPulseUs_ = 0;
  moving_ = true;
  return Result::Ok;
}

void StepperBackend::stop() {
  gpio_.write(pins_.step, false);
  gpio_.write(pins_.enable, pins_.enableActiveLow);
  moving_ = false;
  stepHigh_ = false;
  direction_ = Direction::Stop;
  deadlineMs_ = 0;
  nextPulseUs_ = 0;
}

void StepperBackend::tick(uint32_t nowMs, uint32_t nowUs) {
  if (!moving_) return;
  if (deadlineReached(nowMs, deadlineMs_)) {
    stop();
    return;
  }
  if (nextPulseUs_ == 0) nextPulseUs_ = nowUs;
  if (static_cast<int32_t>(nowUs - nextPulseUs_) < 0) return;
  stepHigh_ = !stepHigh_;
  gpio_.write(pins_.step, stepHigh_);
  nextPulseUs_ = nowUs + pulseIntervalUs_ / 2;
  if (stepHigh_) position_ += direction_ == Direction::Up ? 1 : -1;
}

Status StepperBackend::status() const {
  return {id_, kind(), direction_, moving_, position_, validity_,
          deadlineMs_, false};
}

void StepperBackend::markHomed(int64_t position) {
  position_ = position;
  validity_ = PositionValidity::Homed;
}

ActivityDetector::ActivityDetector(uint32_t debounceMs)
    : debounceMs_(debounceMs), candidateSinceMs_(0), candidate_(Direction::Stop),
      stable_(Direction::Stop), lastDirectionChangeMs_(0),
      reversalWindowStartMs_(0), reversals_(0), autofocusPattern_(false) {}

ActivityState ActivityDetector::sample(bool speed, bool up, bool down, uint32_t nowMs) {
  Direction observed = Direction::Stop;
  if (speed && up && !down) observed = Direction::Up;
  if (speed && down && !up) observed = Direction::Down;
  if (observed != candidate_) {
    candidate_ = observed;
    candidateSinceMs_ = nowMs;
  }
  if (candidate_ != stable_ && nowMs - candidateSinceMs_ >= debounceMs_) {
    Direction previous = stable_;
    stable_ = candidate_;
    if (previous != Direction::Stop && stable_ != Direction::Stop &&
        previous != stable_) {
      if (reversalWindowStartMs_ == 0 || nowMs - reversalWindowStartMs_ > 60000) {
        reversalWindowStartMs_ = nowMs;
        reversals_ = 0;
      }
      ++reversals_;
      autofocusPattern_ = reversals_ >= 3;
    }
    lastDirectionChangeMs_ = nowMs;
  }
  return state();
}

ActivityState ActivityDetector::state() const {
  return {stable_, stable_ != Direction::Stop, autofocusPattern_, reversals_};
}

Controller::Controller() : focusers_{nullptr}, count_(0) {}

Result Controller::add(FocuserBackend& backend) {
  if (count_ >= kMaxFocusers || find(backend.id()) != nullptr) return Result::Busy;
  focusers_[count_++] = &backend;
  return Result::Ok;
}

FocuserBackend* Controller::find(const char* id) const {
  if (id == nullptr) return nullptr;
  for (size_t i = 0; i < count_; ++i) {
    if (std::strcmp(focusers_[i]->id(), id) == 0) return focusers_[i];
  }
  return nullptr;
}

Result Controller::move(const char* id, Direction direction, uint32_t durationMs,
                        uint32_t nowMs) {
  FocuserBackend* backend = find(id);
  return backend == nullptr ? Result::NotFound
                            : backend->move(direction, durationMs, nowMs);
}

Result Controller::moveSynchronized(const char* const* ids, size_t count,
                                    Direction direction, uint32_t durationMs,
                                    uint32_t nowMs) {
  if (ids == nullptr || count == 0 || count > kMaxFocusers) {
    return Result::InvalidArgument;
  }
  FocuserBackend* selected[kMaxFocusers] = {nullptr};
  for (size_t i = 0; i < count; ++i) {
    selected[i] = find(ids[i]);
    if (selected[i] == nullptr) return Result::NotFound;
  }
  for (size_t i = 0; i < count; ++i) {
    Result result = selected[i]->move(direction, durationMs, nowMs);
    if (result != Result::Ok) {
      for (size_t j = 0; j <= i; ++j) selected[j]->stop();
      return result;
    }
  }
  return Result::Ok;
}

Result Controller::stop(const char* id) {
  FocuserBackend* backend = find(id);
  if (backend == nullptr) return Result::NotFound;
  backend->stop();
  return Result::Ok;
}

void Controller::stopAll() {
  for (size_t i = 0; i < count_; ++i) focusers_[i]->stop();
}

void Controller::tick(uint32_t nowMs, uint32_t nowUs) {
  for (size_t i = 0; i < count_; ++i) focusers_[i]->tick(nowMs, nowUs);
}

size_t Controller::count() const { return count_; }

FocuserBackend* Controller::at(size_t index) const {
  return index < count_ ? focusers_[index] : nullptr;
}

const char* directionName(Direction value) {
  switch (value) {
    case Direction::Up: return "up";
    case Direction::Down: return "down";
    default: return "stop";
  }
}

const char* validityName(PositionValidity value) {
  switch (value) {
    case PositionValidity::Estimated: return "estimated";
    case PositionValidity::Homed: return "homed";
    case PositionValidity::Measured: return "measured";
    default: return "unknown";
  }
}

const char* resultName(Result value) {
  switch (value) {
    case Result::Ok: return "ok";
    case Result::InvalidArgument: return "invalid_argument";
    case Result::Busy: return "busy";
    case Result::SafetyLocked: return "safety_locked";
    case Result::BackendFault: return "backend_fault";
    case Result::NotFound: return "not_found";
  }
  return "unknown";
}

}  // namespace pteroptyx
