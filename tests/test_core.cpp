#include "FocuserCore.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>

using namespace pteroptyx;

class MockGpio final : public Gpio {
 public:
  void modeOutput(uint8_t pin) override { outputs[pin] = true; }
  void modeInput(uint8_t pin) override { outputs[pin] = false; }
  void write(uint8_t pin, bool high) override { states[pin] = high; }
  bool read(uint8_t pin) const override { return states[pin]; }

  std::array<bool, 64> outputs{};
  std::array<bool, 64> states{};
};

int main() {
  MockGpio gpio;
  ZwoEafBackend zwo("zwo-1", gpio, {2, 3, 4}, 25, 5000);
  StepperBackend stepper("stepper-1", gpio, {6, 7, 8, true}, 1000, 5000);
  Controller controller;

  zwo.begin();
  stepper.begin();
  assert(!gpio.states[2] && !gpio.states[3] && !gpio.states[4]);
  assert(controller.add(zwo) == Result::Ok);
  assert(controller.add(stepper) == Result::Ok);
  assert(controller.count() == 2);

  assert(controller.move("zwo-1", Direction::Up, 250, 1000) == Result::Ok);
  assert(gpio.states[2] && gpio.states[3] && !gpio.states[4]);
  controller.tick(1249, 0);
  assert(zwo.status().moving);
  controller.tick(1250, 0);
  assert(!zwo.status().moving);
  assert(!gpio.states[2] && !gpio.states[3] && !gpio.states[4]);

  assert(controller.move("zwo-1", Direction::Down, 10, 2000) ==
         Result::InvalidArgument);
  assert(controller.move("zwo-1", Direction::Down, 5001, 2000) ==
         Result::InvalidArgument);

  const char* ids[] = {"zwo-1", "stepper-1"};
  assert(controller.moveSynchronized(ids, 2, Direction::Down, 100, 3000) ==
         Result::Ok);
  assert(zwo.status().moving && stepper.status().moving);
  assert(gpio.states[2] && !gpio.states[3] && gpio.states[4]);

  for (uint32_t us = 3000000; us < 3010000; us += 500) {
    controller.tick(3000 + (us - 3000000) / 1000, us);
  }
  assert(stepper.status().position < 0);
  assert(stepper.status().positionValidity == PositionValidity::Estimated);
  controller.stopAll();
  assert(!zwo.status().moving && !stepper.status().moving);

  ActivityDetector detector(20);
  detector.sample(true, true, false, 0);
  assert(!detector.sample(true, true, false, 19).active);
  assert(detector.sample(true, true, false, 20).direction == Direction::Up);
  detector.sample(true, false, true, 100);
  detector.sample(true, false, true, 120);
  detector.sample(true, true, false, 200);
  detector.sample(true, true, false, 220);
  detector.sample(true, false, true, 300);
  detector.sample(true, false, true, 320);
  detector.sample(true, true, false, 400);
  ActivityState activity = detector.sample(true, true, false, 420);
  assert(activity.reversals >= 3);
  assert(activity.autofocusPattern);

  assert(controller.move("missing", Direction::Up, 100, 0) == Result::NotFound);

  std::cout << "PASS: core safety, timing, synchronization, position validity, activity detection\n";
  return 0;
}
