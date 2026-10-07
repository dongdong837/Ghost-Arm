// Reproduce the blocked-finger windup with Gazebo's actual PID implementation.
// Compile: c++ tests/integration/check_finger_pid.cpp $(pkg-config --cflags --libs ignition-math6) -o /tmp/check_finger_pid
#include <ignition/math/PID.hh>
#include <chrono>
#include <iostream>

static double afterLongHold(double integralGain, double integralLimit)
{
  ignition::math::PID pid(30, integralGain, .3, integralLimit,
                          -integralLimit, 3, -3);
  const auto dt = std::chrono::milliseconds(2);
  // A 4 cm block keeps each finger at 1 cm despite a zero position command.
  for (int i = 0; i < 500000; ++i)
    pid.Update(.01, dt);
  double force = 0;
  // Ask for a 2.5 cm opening; examine the force before the finger can move.
  for (int i = 0; i < 40000; ++i)
    force = pid.Update(.01 - .025, dt);
  return force;
}

int main()
{
  const double oldForce = afterLongHold(.1, 1);
  const double fixedForce = afterLongHold(0, 0);
  std::cout << "After a 1000 s blocked hold and 80 s open command: old="
            << oldForce << " N, fixed=" << fixedForce << " N\n";
  // Negative force continues closing; positive force opens the finger.
  return oldForce < 0 && fixedForce > 0 ? 0 : 1;
}
