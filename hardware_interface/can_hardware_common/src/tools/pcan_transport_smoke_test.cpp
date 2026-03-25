#include "can_hardware_common/can_transport.hpp"

#include <PCANBasic.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{
enum class CaseStatus
{
  kPass,
  kSkip,
  kFail
};

struct Summary
{
  int passed = 0;
  int skipped = 0;
  int failed = 0;
};

struct Options
{
  int timeout_ms = 20;
  int gap_us = 5000;
  uint32_t can_id = 0x123;
  bool strict_transport = false;
};

bool is_bus_related_status(TPCANStatus status)
{
  return (status & PCAN_ERROR_BUSLIGHT) != 0 ||
         (status & PCAN_ERROR_BUSHEAVY) != 0 ||
         (status & PCAN_ERROR_BUSPASSIVE) != 0 ||
         (status & PCAN_ERROR_BUSOFF) != 0 ||
         (status & PCAN_ERROR_XMTFULL) != 0 ||
         (status & PCAN_ERROR_QXMTFULL) != 0 ||
         (status & PCAN_ERROR_OVERRUN) != 0 ||
         (status & PCAN_ERROR_QOVERRUN) != 0;
}

std::string status_label(TPCANStatus status)
{
  if (status == PCAN_ERROR_OK) {
    return "OK";
  }
  if (status == PCAN_ERROR_QRCVEMPTY) {
    return "QRCVEMPTY";
  }
  return can_hardware_common::CanTransport::bus_status_string(status);
}

CaseStatus report_result(
  const std::string & case_name,
  CaseStatus status,
  const std::string & detail)
{
  const char * label = "FAIL";
  if (status == CaseStatus::kPass) {
    label = "PASS";
  } else if (status == CaseStatus::kSkip) {
    label = "SKIP";
  }
  std::cout << "[" << label << "] " << case_name;
  if (!detail.empty()) {
    std::cout << ": " << detail;
  }
  std::cout << "\n";
  return status;
}

void update_summary(Summary & summary, CaseStatus status)
{
  if (status == CaseStatus::kPass) {
    ++summary.passed;
  } else if (status == CaseStatus::kSkip) {
    ++summary.skipped;
  } else {
    ++summary.failed;
  }
}

CaseStatus run_diagnostics_case(can_hardware_common::CanTransport & transport)
{
  const auto diag = transport.get_diagnostics();
  std::string detail =
    "bus_status=" + status_label(diag.bus_status) +
    " channel_condition=" + std::to_string(diag.channel_condition) +
    " receive_status=" + std::to_string(diag.receive_status);
  return report_result("diagnostics_query", CaseStatus::kPass, detail);
}

CaseStatus run_read_timeout_case(
  can_hardware_common::CanTransport & transport,
  const Options & options)
{
  TPCANMsg frame{};
  const auto st = transport.read_frame(
    frame, std::chrono::milliseconds(options.timeout_ms));

  if (st == PCAN_ERROR_OK || st == PCAN_ERROR_QRCVEMPTY) {
    return report_result("read_timeout", CaseStatus::kPass, "status=" + status_label(st));
  }
  if (is_bus_related_status(st) && !options.strict_transport) {
    return report_result("read_timeout", CaseStatus::kSkip, "status=" + status_label(st));
  }
  return report_result("read_timeout", CaseStatus::kFail, "status=" + status_label(st));
}

CaseStatus run_tx_pacing_case(
  can_hardware_common::CanTransport & transport,
  const Options & options)
{
  transport.set_min_inter_frame_gap(std::chrono::microseconds(options.gap_us));

  TPCANMsg frame{};
  std::memset(&frame, 0, sizeof(frame));
  frame.ID = options.can_id;
  frame.MSGTYPE = PCAN_MESSAGE_STANDARD;
  frame.LEN = 1;
  frame.DATA[0] = 0xA5;

  const auto s1 = transport.send_if_ready(frame);
  if (s1 != PCAN_ERROR_OK) {
    if (is_bus_related_status(s1) && !options.strict_transport) {
      return report_result("tx_pacing", CaseStatus::kSkip, "first_send=" + status_label(s1));
    }
    return report_result("tx_pacing", CaseStatus::kFail, "first_send=" + status_label(s1));
  }

  const auto s2 = transport.send_if_ready(frame);
  if (s2 != PCAN_ERROR_QXMTFULL) {
    if (is_bus_related_status(s2) && !options.strict_transport) {
      return report_result("tx_pacing", CaseStatus::kSkip, "immediate_second_send=" + status_label(s2));
    }
    return report_result(
      "tx_pacing",
      CaseStatus::kFail,
      "expected QXMTFULL, got " + status_label(s2));
  }

  std::this_thread::sleep_for(std::chrono::microseconds(options.gap_us + 1000));
  const auto s3 = transport.send_if_ready(frame);
  if (s3 == PCAN_ERROR_QXMTFULL) {
    return report_result("tx_pacing", CaseStatus::kFail, "gap elapsed but still QXMTFULL");
  }
  if (s3 != PCAN_ERROR_OK && is_bus_related_status(s3) && !options.strict_transport) {
    return report_result("tx_pacing", CaseStatus::kSkip, "post_gap_send=" + status_label(s3));
  }
  if (s3 != PCAN_ERROR_OK) {
    return report_result("tx_pacing", CaseStatus::kFail, "post_gap_send=" + status_label(s3));
  }

  return report_result("tx_pacing", CaseStatus::kPass, "pacing gate works as expected");
}

bool parse_options(int argc, char ** argv, Options & options)
{
  std::vector<std::string> args(argv + 1, argv + argc);
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--timeout-ms" && i + 1 < args.size()) {
      options.timeout_ms = std::stoi(args[++i]);
      continue;
    }
    if (args[i] == "--gap-us" && i + 1 < args.size()) {
      options.gap_us = std::stoi(args[++i]);
      continue;
    }
    if (args[i] == "--id" && i + 1 < args.size()) {
      options.can_id = static_cast<uint32_t>(std::stoul(args[++i], nullptr, 0));
      continue;
    }
    if (args[i] == "--strict-transport") {
      options.strict_transport = true;
      continue;
    }
    if (args[i] == "--help" || args[i] == "-h") {
      std::cout
        << "Usage: pcan_transport_smoke_test [options]\n"
        << "  --timeout-ms <int>       read timeout in ms (default: 20)\n"
        << "  --gap-us <int>           TX pacing gap for test (default: 5000)\n"
        << "  --id <hex|dec>           TX CAN ID for test frame (default: 0x123)\n"
        << "  --strict-transport       treat bus-related transport errors as FAIL\n";
      return false;
    }
    std::cerr << "Unknown argument: " << args[i] << "\n";
    return false;
  }
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parse_options(argc, argv, options)) {
    return 2;
  }

  Summary summary;
  try {
    can_hardware_common::CanTransport transport;
    update_summary(summary, run_diagnostics_case(transport));
    update_summary(summary, run_read_timeout_case(transport, options));
    update_summary(summary, run_tx_pacing_case(transport, options));
  } catch (const std::exception & e) {
    std::cerr << "[FAIL] init: " << e.what() << "\n";
    return 1;
  }

  std::cout << "[SUMMARY] passed=" << summary.passed
            << " skipped=" << summary.skipped
            << " failed=" << summary.failed << "\n";
  return summary.failed == 0 ? 0 : 1;
}

