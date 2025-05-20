// Copyright 2021 Gaia Platform LLC

#ifndef TTL_ROW_HPP_
#define TTL_ROW_HPP_

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace race::ttl
{

class TtlRow
{
public:
  std::string const & operator[](size_t index) const
  {
    return row_[index];
  }

  size_t size() const
  {
    return row_.size();
  }

  std::istream & read_next(std::istream & str)
  {
    std::string line;
    if (!getline(str, line)) {
      return str;
    }
    row_.clear();
    parse_line(line);
    return str;
  }

  // parse a comma delimited string into a row
  void parse_line(std::string line)
  {
    std::string token;
    auto it = line.begin();
    while (it != line.end()) {
      char c = *it++;
      if (c == ',') {
        row_.push_back(token);
        token.clear();
      } else {
        token.push_back(c);
      }
    }
    // get our last column
    row_.push_back(token);
  }

private:
  std::vector<std::string> row_;
};

}  // namespace race::ttl

#endif  // TTL_ROW_HPP_
