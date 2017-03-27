#include "files.h"

#include <fstream>

std::vector<std::string> tools::Files::read(const std::string& fileName) {
  std::ifstream file(fileName.c_str());
  std::string line;
  std::vector<std::string> content = std::vector<std::string>();
  if(file.is_open()) {
    while (std::getline(file, line)) {
      content.push_back(line);
    }
    file.close();
  } else {
    throw -1;
  }
  return content;
}

void tools::Files::write(const std::vector<std::string>& input, const std::string& fileName) {
  std::ofstream file(fileName.c_str(), std::ios::out | std::ios::trunc);
  if(file.is_open()) {
    for(unsigned i = 0; i < input.size(); i++) {
      file << input[i] << std::endl;
    }
    file.close();
  } else {
    throw -1;
  }
}
