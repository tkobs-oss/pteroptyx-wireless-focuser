#pragma once

#include <cstddef>
#include <cstdint>

namespace pteroptyx {

enum class Direction : uint8_t { Stop = 0, Up = 1, Down = 2 };
enum class PositionValidity : uint8_t { Unknown = 0, Estimated = 1, Homed = 2, Measured = 3 };
enum class Result : uint8_t {
  Ok = 0,
  InvalidArgument,
  Busy,
  SafetyLocked,
  BackendFault,
  NotFound
};

struct Status {
  const char* id;
  const char* backend;
  Direction direction;
  bool moving;
  int64_t position;
  PositionValidity positionValidity;
  uint32_t deadlineMs;
  bool activityObserved;
};

class Gpio {
 public:
  virtual ~Gpio() = default;
  virtual void modeOutput(uint8_t pin) = 0;
  virtual void modeInput(uint8_t pin) = 0;
  virtual void write(uint8_t pin, bool high) = 0;
  virtual bool read(uint8_t pin) const = 0;
};

class FocuserBackend {
 public:
  virtual ~FocuserBackend() = default;
  virtual const char* id() const = 0;
  virtual const char* kind() const = 0;
  virtual void begin() = 0;
  virtual Result move(Direction direction, uint32_t durationMs, uint32_t nowMs) = 0;
  virtual void stop() = 0;
  virtual void tick(uint32_t nowMs, uint32_t nowUs) = 0;
  virtual Status status() const = 0;
};

struct ZwoPins {
  uint8_t speed;
  uint8_t up;
  uint8_t down;
};

class ZwoEafBackend final : public FocuserBackend {
 public:
  ZwoEafBackend(const char* id, Gpio& gpio, ZwoPins pins,
                uint32_t minDurationMs, uint32_t maxDurationMs);
  const char* id() const override;
  const char* kind() const override;
  void begin() override;
  Result move(Direction direction, uint32_t durationMs, uint32_t nowMs) override;
  void stop() override;
  void tick(uint32_t nowMs, uint32_t nowUs) override;
  Status status() const override;

 private:
  const char* id_;
  Gpio& gpio_;
  ZwoPins pins_;
  uint32_t minDurationMs_;
  uint32_t maxDurationMs_;
  Direction direction_;
  uint32_t deadlineMs_;
  bool moving_;
};

struct StepperPins {
  uint8_t step;
  uint8_t direction;
  uint8_t enable;
  bool enableActiveLow;
};

class StepperBackend final : public FocuserBackend {
 public:
  StepperBackend(const char* id, Gpio& gpio, StepperPins pins,
                 uint32_t pulseIntervalUs, uint32_t maxDurationMs);
  const char* id() const override;
  const char* kind() const override;
  void begin() override;
  Result move(Direction direction, uint32_t durationMs, uint32_t nowMs) override;
  void stop() override;
  void tick(uint32_t nowMs, uint32_t nowUs) override;
  Status status() const override;
  void markHomed(int64_t position = 0);

 private:
  const char* id_;
  Gpio& gpio_;
  StepperPins pins_;
  uint32_t pulseIntervalUs_;
  uint32_t maxDurationMs_;
  Direction direction_;
  uint32_t deadlineMs_;
  uint32_t nextPulseUs_;
  int64_t position_;
  PositionValidity validity_;
  bool moving_;
  bool stepHigh_;
};

struct ActivityState {
  Direction direction;
  bool active;
  bool autofocusPattern;
  uint32_t reversals;
};

class ActivityDetector {
 public:
  explicit ActivityDetector(uint32_t debounceMs = 20);
  ActivityState sample(bool speed, bool up, bool down, uint32_t nowMs);
  ActivityState state() const;

 private:
  uint32_t debounceMs_;
  uint32_t candidateSinceMs_;
  Direction candidate_;
  Direction stable_;
  uint32_t lastDirectionChangeMs_;
  uint32_t reversalWindowStartMs_;
  uint32_t reversals_;
  bool autofocusPattern_;
};

class Controller {
 public:
  static constexpr size_t kMaxFocusers = 8;
  Controller();
  Result add(FocuserBackend& backend);
  FocuserBackend* find(const char* id) const;
  Result move(const char* id, Direction direction, uint32_t durationMs, uint32_t nowMs);
  Result moveSynchronized(const char* const* ids, size_t count, Direction direction,
                          uint32_t durationMs, uint32_t nowMs);
  Result stop(const char* id);
  void stopAll();
  void tick(uint32_t nowMs, uint32_t nowUs);
  size_t count() const;
  FocuserBackend* at(size_t index) const;

 private:
  FocuserBackend* focusers_[kMaxFocusers];
  size_t count_;
};

const char* directionName(Direction value);
const char* validityName(PositionValidity value);
const char* resultName(Result value);

}  // namespace pteroptyx
