#ifndef _FILES_HPP_

#define _FILES_HPP_

#include <string>
#include <vector>

namespace tools {
  class Files {
    public:
      static std::vector<std::string> read(const std::string& fileName);
      static void write(const std::vector<std::string>& content, const std::string& fileName);
  };
}

#endif
