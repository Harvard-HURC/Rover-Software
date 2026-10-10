// The rover's ECM components (rover_components.hh): a setpoint compares and streams as its three fields, so the ECM's
// change tracking and its serialization see each of them, and both components carry their data.
#include <chrono>
#include <sstream>
#include <string>

#include "check.hh"
#include "rover_components.hh"

namespace components = rover_sim::components;

namespace {

void SetpointEquality() {
  const components::Setpoint a{2.5, std::chrono::milliseconds(1234), true};
  components::Setpoint b = a;
  CHECK(a == b);
  b.velocity = 2.4;
  CHECK(a != b);
  b = a;
  b.time = std::chrono::milliseconds(1235);
  CHECK(a != b);
  b = a;
  b.ros2_control = false;
  CHECK(a != b);
}

void SetpointStreams() {
  const components::Setpoint a{-3.25, std::chrono::nanoseconds(1500000001), true};
  std::stringstream text;
  text << a;
  CHECK(text.str() == "-3.25 1500000001 1");
  components::Setpoint b;
  text >> b;
  CHECK(a == b);
  const components::Setpoint fine{0.1 + 0.2, std::chrono::nanoseconds(0), false};  // 17 digits survive
  std::stringstream again;
  again << fine;
  components::Setpoint back;
  again >> back;
  CHECK(back == fine);
}

void ComponentsCarryTheirData() {
  const components::WheelSetpoint setpoint(components::Setpoint{1.0, std::chrono::seconds(2), false});
  CHECK(setpoint.Data().velocity == 1.0);
  CHECK(setpoint.Data().time == std::chrono::seconds(2));
  const components::WheelMotorCurrent current(12.5);
  CHECK(current.Data() == 12.5);
  // Registered (the header's static registration in this program set the names).
  CHECK(components::WheelSetpoint::typeName != nullptr &&
        std::string(components::WheelSetpoint::typeName) == "rover_sim_components.WheelSetpoint");
  CHECK(components::WheelMotorCurrent::typeName != nullptr &&
        std::string(components::WheelMotorCurrent::typeName) == "rover_sim_components.WheelMotorCurrent");
}

}  // namespace

int main() {
  SetpointEquality();
  SetpointStreams();
  ComponentsCarryTheirData();
  if (Failures() == 0) std::cout << "test_rover_components: all checks passed\n";
  return Failures() == 0 ? 0 : 1;
}
