// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>

#include <cmath>
#include <sstream>
#include <string>

namespace mppi_core::logging {

inline std::string BoolToString(const bool value) { return value ? "true" : "false"; }

inline std::string EscapeCsvCell(const std::string& value) {
  bool needs_quotes = false;
  for (const char c : value) {
    if (c == ',' || c == '"' || c == '\n' || c == '\r' || c == ';') {
      needs_quotes = true;
      break;
    }
  }
  if (!needs_quotes) {
    return value;
  }

  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (const char c : value) {
    if (c == '"') {
      escaped.push_back('"');
    }
    escaped.push_back(c);
  }
  escaped.push_back('"');
  return escaped;
}

inline std::string ScalarToCsvValue(const double value) {
  if (std::isnan(value)) {
    return "nan";
  }
  if (std::isinf(value)) {
    return value < 0.0 ? "-inf" : "inf";
  }

  std::ostringstream stream;
  stream << value;
  return stream.str();
}

inline std::string VectorToCsvCell(const Eigen::VectorXd& v) {
  if (v.size() == 0) {
    return "";
  }

  std::ostringstream stream;
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (i > 0) {
      stream << ';';
    }
    stream << ScalarToCsvValue(v[i]);
  }
  return EscapeCsvCell(stream.str());
}

}  // namespace mppi_core::logging
