// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "plato_robot_system/sensor/nari_touch.hpp"
#include "plato_robot_system/sensor/tactile_state.hpp"

namespace plato_robot_system::sensor
{

int ToTactileContactState(NARITouchContactState state);
double ComputeNARITouchConfidence(const NARITouchSample & sample, const TactileState & tactile_state);
TactileState ConvertNARITouchToTactileState(const NARITouchSample & sample);

}  // namespace plato_robot_system::sensor
