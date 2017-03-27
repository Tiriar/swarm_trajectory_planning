#include "polygon.h"

#include <iostream>

#include "files.h"
#include "functional.h"


geom::Polygon::Polygon() {
  this->_points = std::vector<geom::Point>();
}

geom::Polygon::Polygon(std::vector<geom::Point> points) {
  this->_points = points;
}

std::vector<geom::Point> geom::Polygon::points() const {
  return this->_points;
}

std::vector<geom::Segment> geom::Polygon::edges() const {
  std::vector<geom::Segment> edges = std::vector<geom::Segment>();
  if(this->_points.size() == 0) {
    return edges;
  }

  for(unsigned int i = 1; i < this->_points.size(); i++) {
    edges.push_back(Segment(this->_points[i - 1], this->_points[i]));
  }
  if(this->_points.size() > 2) {
    edges.push_back(Segment(this->_points[this->_points.size() - 1], this->_points[0]));
  }
  return edges;
}

std::pair<geom::Polygon, std::vector<geom::Polygon> > geom::Polygon::loadFromFile(const std::string& fileName) {
  class : public :: tools::Predicate<std::string> { 
    public: bool apply(const std::string& input) const {
      return input == "[OBSTACLE]" || input == "[obstacle]";
  }} splitter;
  
  class : public :: tools::Function<std::vector<std::string>, geom::Polygon> {
    public: geom::Polygon apply(const std::vector<std::string>& input) const {
      return geom::Polygon(tools::Vectors::transform(input, geom::Points::FROM_STRING()));
  }} strings2polygon;

  std::vector<std::string> content; 
  try {
    content = tools::Vectors::filter(tools::Files::read(fileName), tools::Predicates::IS_NOT_EMPTY_STRING());
  } catch (int e) {
    std::cerr << "File not found." << std::endl;
    throw -1;
  }

  if(content.size() > 0 && content[0] == "[BORDER]") {
    content.erase(content.begin());
    std::vector<std::vector<std::string> > splitted = tools::Vectors::split(content, splitter);
    if(splitted.size() > 0) {
      geom::Polygon border = strings2polygon.apply(splitted[0]);
      splitted.erase(splitted.begin());
      return std::make_pair(border, tools::Vectors::transform(
        tools::Vectors::filter(splitted, tools::Predicates::IS_NOT_EMPTY_VECTOR<std::string>()), strings2polygon
      ));
    }
  }
  std::cerr << "Unsupported file format." << std::endl;
  throw -1;
}
