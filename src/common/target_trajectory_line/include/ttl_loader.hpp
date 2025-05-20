// Copyright 2021 Gaia Platform LLC

#ifndef TTL_LOADER_HPP_
#define TTL_LOADER_HPP_

#include <string>
#include <vector>
#include <utility>

#include "ttl.hpp"

using race::ttl::TtlArray;

namespace race::ttl
{

// Load all CSV files in the directory.
bool load_all(
  std::string const & directory, TtlArray & ttl_array,
  std::vector<uint8_t> & valid_ttl_idx);

// Load a .csv file.  Use the "TTL Index" to figure out what
// type of file to load
std::pair<bool, race::ttl::TtlIndex> load(std::string const & filename, TtlArray & ttl_array);
}  // namespace race::ttl

#endif  // TTL_LOADER_HPP_
