// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/logging/mppi_rollout_logger.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "mppi_core/logging/vector_csv.hpp"

namespace mppi_core::logging {
namespace {

std::string ScalarCell(const double value) {
  return ScalarToCsvValue(value);
}

double TactileTotalForceN(const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>& tactile) {
  double total = 0.0;
  for (const auto& sensor : tactile) {
    double sensor_force = sensor.activeHemisphereNormalForceN();
    if ((!std::isfinite(sensor_force) || sensor_force <= 0.0) &&
        sensor.total_force_n.allFinite()) {
      sensor_force = sensor.total_force_n.norm();
    }
    if (std::isfinite(sensor_force)) {
      total += std::max(0.0, sensor_force);
    }
  }
  return total;
}

int ActiveTactileSensorCount(
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>& tactile) {
  int count = 0;
  for (const auto& sensor : tactile) {
    if (sensor.hasActiveHemisphereContact()) {
      ++count;
    }
  }
  return count;
}

int ActiveHemisphereCountTotal(
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>& tactile) {
  int count = 0;
  for (const auto& sensor : tactile) {
    count += static_cast<int>(sensor.activeHemisphereCount());
  }
  return count;
}

void WriteCell(std::ofstream& file, const std::string& value) {
  file << EscapeCsvCell(value);
}

void WriteVectorCell(std::ofstream& file, const Eigen::VectorXd& value) {
  file << VectorToCsvCell(value);
}

}  // namespace

bool MppiRolloutLogger::Configure(const MppiRolloutLoggerConfig& config) {
  config_ = config;
  enabled_ = false;
  headers_written_ = false;
  write_count_ = 0;

  if (ticks_file_.is_open()) {
    ticks_file_.close();
  }
  if (rollouts_file_.is_open()) {
    rollouts_file_.close();
  }
  if (events_file_.is_open()) {
    events_file_.close();
  }

  if (!config_.enabled) {
    return true;
  }
  if (config_.rollout_log_stride <= 0) {
    config_.rollout_log_stride = 1;
  }
  if (config_.flush_every_n_ticks <= 0) {
    config_.flush_every_n_ticks = 1;
  }

  enabled_ = OpenFiles();
  if (enabled_) {
    WriteTickHeader();
    WriteRolloutHeader();
    WriteEventHeader();
    headers_written_ = true;
    std::cout << "[mppi_logger] writing CSV logs to "
              << std::filesystem::path(config_.output_directory).string()
              << " prefix=" << config_.file_prefix
              << std::endl;
  } else {
    std::cerr << "[mppi_logger] failed to open CSV logs in "
              << std::filesystem::path(config_.output_directory).string()
              << " prefix=" << config_.file_prefix
              << std::endl;
  }
  return enabled_;
}

bool MppiRolloutLogger::OpenFiles() {
  try {
    std::filesystem::create_directories(config_.output_directory);
    const std::filesystem::path directory(config_.output_directory);
    ticks_file_.open(directory / (config_.file_prefix + "_ticks.csv"),
                     std::ios::out | std::ios::trunc);
    rollouts_file_.open(directory / (config_.file_prefix + "_rollouts.csv"),
                        std::ios::out | std::ios::trunc);
    events_file_.open(directory / (config_.file_prefix + "_events.csv"),
                      std::ios::out | std::ios::trunc);
    const bool opened =
        ticks_file_.is_open() && rollouts_file_.is_open() && events_file_.is_open();
    if (!opened) {
      if (ticks_file_.is_open()) {
        ticks_file_.close();
      }
      if (rollouts_file_.is_open()) {
        rollouts_file_.close();
      }
      if (events_file_.is_open()) {
        events_file_.close();
      }
    }
    return opened;
  } catch (...) {
    if (ticks_file_.is_open()) {
      ticks_file_.close();
    }
    if (rollouts_file_.is_open()) {
      rollouts_file_.close();
    }
    if (events_file_.is_open()) {
      events_file_.close();
    }
    return false;
  }
}

void MppiRolloutLogger::WriteTickHeader() {
  ticks_file_
      << "tick_index,time_s,controller_state,phase,q_meas,qdot_meas,tau_meas,"
         "q_ref_current,qdot_ref_current,q_cmd,qdot_cmd,tau_cmd,kp,kd,"
         "selected_action_qddot,nominal_total_cost,command_valid,used_fallback,"
         "tactile_sensor_count,active_tactile_sensor_count,"
         "active_hemisphere_count_total,tactile_total_force_n\n";
}

void MppiRolloutLogger::WriteRolloutHeader() {
  rollouts_file_
      << "tick_index,time_s,horizon_index,pred_time_s,q_pred,qdot_pred,tau_pred,"
         "action_qddot,step_cost,total_cost_so_far,valid,"
         "active_tactile_sensor_count,active_hemisphere_count_total,"
         "tactile_total_force_n\n";
}

void MppiRolloutLogger::WriteEventHeader() {
  events_file_ << "tick_index,time_s,event,detail\n";
}

void MppiRolloutLogger::LogTick(const MppiTickLogRecord& record) {
  if (!enabled_) {
    return;
  }
  try {
    ticks_file_ << record.tick_index << ',' << ScalarCell(record.time_s) << ',';
    WriteCell(ticks_file_, record.controller_state);
    ticks_file_ << ',';
    WriteCell(ticks_file_, record.phase);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.q_meas);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.qdot_meas);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.tau_meas);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.q_ref_current);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.qdot_ref_current);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.q_cmd);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.qdot_cmd);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.tau_cmd);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.kp);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.kd);
    ticks_file_ << ',';
    WriteVectorCell(ticks_file_, record.selected_action_qddot);
    ticks_file_ << ',' << ScalarCell(record.nominal_total_cost) << ','
                << BoolToString(record.command_valid) << ','
                << BoolToString(record.used_fallback) << ','
                << record.tactile_sensor_count << ','
                << record.active_tactile_sensor_count << ','
                << record.active_hemisphere_count_total << ','
                << ScalarCell(record.tactile_total_force_n) << '\n';

    ++write_count_;
    if (write_count_ % static_cast<uint64_t>(config_.flush_every_n_ticks) == 0) {
      Flush();
    }
  } catch (...) {
    enabled_ = false;
  }
}

void MppiRolloutLogger::LogRollout(
    const uint64_t tick_index, const double time_s, const RolloutTrace& trace) {
  if (!enabled_) {
    return;
  }

  try {
    const std::size_t horizon_steps =
        std::min(trace.actions.size(), trace.step_costs.size());
    const std::size_t state_steps =
        trace.states.empty() ? 0 : trace.states.size() - 1;
    std::size_t steps = std::min(horizon_steps, state_steps);
    if (config_.max_logged_horizon_steps > 0) {
      steps = std::min(steps, static_cast<std::size_t>(config_.max_logged_horizon_steps));
    }

    double total_cost_so_far = 0.0;
    for (std::size_t h = 1; h <= steps; ++h) {
      const auto& state = trace.states[h];
      const auto& action = trace.actions[h - 1];
      const double step_cost = trace.step_costs[h - 1];
      const double pred_time_s =
          std::isfinite(state.robot.time_s) ? state.robot.time_s : time_s;
      total_cost_so_far += std::isfinite(step_cost) ? step_cost : 0.0;

      rollouts_file_ << tick_index << ',' << ScalarCell(time_s) << ',' << h << ','
                     << ScalarCell(pred_time_s) << ',';
      WriteVectorCell(rollouts_file_, state.robot.q);
      rollouts_file_ << ',';
      WriteVectorCell(rollouts_file_, state.robot.qdot);
      rollouts_file_ << ',';
      WriteVectorCell(rollouts_file_, state.robot.tau);
      rollouts_file_ << ',';
      WriteVectorCell(rollouts_file_, action);
      rollouts_file_ << ',' << ScalarCell(step_cost) << ','
                     << ScalarCell(total_cost_so_far) << ','
                     << BoolToString(state.valid) << ','
                     << ActiveTactileSensorCount(state.tactile_sensors) << ','
                     << ActiveHemisphereCountTotal(state.tactile_sensors) << ','
                     << ScalarCell(TactileTotalForceN(state.tactile_sensors)) << '\n';
    }
  } catch (...) {
    enabled_ = false;
  }
}

void MppiRolloutLogger::LogEvent(const uint64_t tick_index, const double time_s,
                                 const std::string& event,
                                 const std::string& detail) {
  if (!enabled_) {
    return;
  }
  try {
    events_file_ << tick_index << ',' << ScalarCell(time_s) << ',';
    WriteCell(events_file_, event);
    events_file_ << ',';
    WriteCell(events_file_, detail);
    events_file_ << '\n';
  } catch (...) {
    enabled_ = false;
  }
}

void MppiRolloutLogger::Flush() {
  if (!enabled_) {
    return;
  }
  try {
    ticks_file_.flush();
    rollouts_file_.flush();
    events_file_.flush();
  } catch (...) {
    enabled_ = false;
  }
}

}  // namespace mppi_core::logging
