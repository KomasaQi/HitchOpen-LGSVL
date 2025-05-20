// Copyright 2021 Gaia Platform LLC

#include <filesystem>
#include <cstdio>
#include <array>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "ttl_row.hpp"
#include "ttl_loader.hpp"
#include "ttl.hpp"

using race::ttl::Ttl;
using race::ttl::TtlIndex;
using race::ttl::TrackLocation;
using race::ttl::TacticLocation;
using race::ttl::TtlArray;
using race::ttl::TtlRow;
using race::ttl::TtlColumn;

#define TTL_LOADER_LOGGER rclcpp::get_logger("ttl_loader")

static void log_parse_error(
  const char * field_name, size_t column_index, size_t line_number)
{
  RCLCPP_ERROR(
    TTL_LOADER_LOGGER, "Error parsing field '%s' at column %lu on line %lu",
    field_name, column_index, line_number);
}

#define PARSE_TTL_DOUBLE(field_name, column_index) \
  try {if (column_index >= row.size()) {throw std::out_of_range("TTL column index out of range");} \
    (field_name) = std::stod(row[(column_index)]);} \
  catch (std::exception & e) {log_parse_error( \
      #field_name, (column_index), line_number); throw;}

#define PARSE_TTL_UNSIGNED(field_name, column_index) \
  try {if (column_index >= row.size()) {throw std::out_of_range("TTL column index out of range");} \
    (field_name) = std::stoul(row[(column_index)]);} \
  catch (std::exception & e) {log_parse_error( \
      #field_name, (column_index), line_number); throw;}

#define PARSE_TTL_ENUM(field_name, column_index, enum_type) \
  try {if (column_index >= row.size()) {throw std::out_of_range("TTL column index out of range");} \
    (field_name) = static_cast<enum_type>(std::stoul(row[(column_index)]));} \
  catch (std::exception & e) {log_parse_error( \
      #field_name, (column_index), line_number); throw;}

// TTL file
static bool load_ttl(std::string const & filename, std::ifstream & f, TtlRow & row, Ttl & ttl);

// Helper for determining the file type
static bool open_header(std::string const & filename, std::ifstream & f, TtlRow & row);

bool race::ttl::load_all(
  std::string const & directory, TtlArray & ttl_array,
  std::vector<uint8_t> & valid_ttl_idx)
{
  const std::filesystem::path ttl_path = directory;

  for (const auto & entry : std::filesystem::directory_iterator(ttl_path)) {
    auto result = race::ttl::load(entry.path(), ttl_array);
    if (!result.first) {
      return false;
    } else {
      if (result.second > race::ttl::TtlIndex::GRID_BOX &&
        result.second < race::ttl::TtlIndex::INVALID)
      {
        valid_ttl_idx.push_back(static_cast<uint8_t>(result.second));
      }
    }
  }

  return true;
}

std::pair<bool, race::ttl::TtlIndex> race::ttl::load(
  std::string const & filename,
  TtlArray & ttl_array)
{
  if (filename.find("EXIT_PIT") != std::string::npos) {
    // Skip this file.
    return std::make_pair(true, race::ttl::TtlIndex::INVALID);
  }

  std::ifstream f;
  TtlRow row;

  if (!open_header(filename, f, row)) {
    return std::make_pair(false, race::ttl::TtlIndex::INVALID);
  }

  const size_t line_number = 0;
  uint8_t ttl_index = 0;
  PARSE_TTL_UNSIGNED(ttl_index, 0);

  if (ttl_index >= MAX_ACTIVE_TTLS) {
    RCLCPP_ERROR(TTL_LOADER_LOGGER, "Invalid trajectory '%ul' in %s", ttl_index, filename.c_str());
    return std::make_pair(false, race::ttl::TtlIndex::INVALID);
  }

  auto & ttl = ttl_array.trajectories.at(ttl_index);
  RCLCPP_INFO(TTL_LOADER_LOGGER, "Loading '%s'", filename.c_str());
  const auto load_result = load_ttl(filename, f, row, ttl);

  return std::make_pair(load_result, static_cast<race::ttl::TtlIndex>(ttl_index));
}

static inline void replace_string(
  std::string & str, const std::string & from,
  const std::string & to)
{
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();     // Handles case where 'to' is a substring of 'from'
  }
}

// TTL file
static bool load_ttl(std::string const & filename, std::ifstream & f, TtlRow & row, Ttl & ttl)
{
  // handle header
  size_t line_number = 0;
  ttl.header.ttl_name = filename;
  std::filesystem::path ttl_path(ttl.header.ttl_name);
  ttl.header.ttl_name = ttl_path.filename();
  replace_string(ttl.header.ttl_name, ".csv", "");
  PARSE_TTL_ENUM(ttl.header.number, 0, TtlIndex);
  PARSE_TTL_UNSIGNED(ttl.header.loop, 1);
  PARSE_TTL_DOUBLE(ttl.header.total_distance, 2);
  PARSE_TTL_DOUBLE(ttl.header.origin.latitude, 3);
  PARSE_TTL_DOUBLE(ttl.header.origin.longitude, 4);
  PARSE_TTL_DOUBLE(ttl.header.origin.altitude, 5);

  // Throw error if number of waypoints exceeds storage capacity
  if (ttl.header.loop > race::ttl::MAX_WAYPOINTS) {
    throw std::logic_error("Number of waypoints in TTL exceeds storage capacity.");
  }

  // Row 0 is the header, start at row 1 for the actual waypoint data
  line_number = 1;
  while (row.read_next(f)) {
    auto & waypoint = ttl.waypoints.at(line_number - 1);
    PARSE_TTL_DOUBLE(waypoint.location.x, TtlColumn::X);
    PARSE_TTL_DOUBLE(waypoint.location.y, TtlColumn::Y);
    PARSE_TTL_DOUBLE(waypoint.target_yaw, TtlColumn::TARGET_YAW);
    PARSE_TTL_DOUBLE(waypoint.target_speed, TtlColumn::TARGET_SPEED);
    PARSE_TTL_DOUBLE(waypoint.curvature, TtlColumn::CURVATURE);
    PARSE_TTL_DOUBLE(waypoint.dist_to_sf_bwd, TtlColumn::DIST_TO_SF_BWD);
    PARSE_TTL_DOUBLE(waypoint.dist_to_sf_fwd, TtlColumn::DIST_TO_SF_FWD);
    PARSE_TTL_ENUM(waypoint.region, TtlColumn::REGION, TrackLocation);
    PARSE_TTL_DOUBLE(waypoint.left_bound.x, TtlColumn::LEFT_BOUND_X);
    PARSE_TTL_DOUBLE(waypoint.left_bound.y, TtlColumn::LEFT_BOUND_Y);
    PARSE_TTL_DOUBLE(waypoint.right_bound.x, TtlColumn::RIGHT_BOUND_X);
    PARSE_TTL_DOUBLE(waypoint.right_bound.y, TtlColumn::RIGHT_BOUND_Y);
    PARSE_TTL_DOUBLE(waypoint.bank_angle, TtlColumn::BANK_ANGLE);
    PARSE_TTL_DOUBLE(waypoint.normal_x, TtlColumn::NORMAL_X);
    PARSE_TTL_DOUBLE(waypoint.normal_y, TtlColumn::NORMAL_Y);
    line_number++;
  }

  return true;
}

static bool open_header(std::string const & filename, std::ifstream & f, TtlRow & row)
{
  f.open(filename);
  if (f.fail()) {
    RCLCPP_ERROR(TTL_LOADER_LOGGER, "Could not open '%s'", filename.c_str());
    return false;
  }

  // The first row is the header.
  if (!row.read_next(f)) {
    RCLCPP_ERROR(TTL_LOADER_LOGGER, "Invalid data format in '%s'", filename.c_str());
    return false;
  }

  return true;
}
