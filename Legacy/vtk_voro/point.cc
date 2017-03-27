#include "point.h"
#include "doubles.h"

#include <cstdio>
#include <cmath>
#include <algorithm>

geom::Point::Point() {
  this->_x = 0;
  this->_y = 0;
}

geom::Point::Point(double x, double y) {
  this->_x = x;
  this->_y = y;
}

geom::Point::Point(const geom::Point& other) {
  this->_x = other.x();
  this->_y = other.y();
}

double geom::Point::length() const {
  return sqrt(geom::Points::scalar(*this, *this));
}

bool geom::Point::compareByXFirst(const geom::Point& point1, const geom::Point& point2) {
  return point1.x() != point2.x()? point1.x() < point2.x(): point1.y() < point2.y();
}

bool geom::Point::compareByYFirst(const geom::Point& point1, const geom::Point& point2) {
  return point1.y() != point2.y()? point1.y() < point2.y(): point1.x() < point2.x();
}

bool geom::Point::compareByXFirstDecreasing(const geom::Point& point1, const geom::Point& point2) {
  return !geom::Point::compareByXFirst(point1, point2);
}

bool geom::Point::compareByYFirstDecreasing(const geom::Point& point1, const geom::Point& point2) {
  return !geom::Point::compareByYFirst(point1, point2);
}

std::string geom::Point::toString() const {
  char str[50];
  std::sprintf(str, "%e %e", this->x(), this->y());
  return std::string(str);
}

geom::Point geom::Point::fromString(const std::string& string) {
  char *c_str;
  double x = std::strtod(string.c_str(), &c_str);
  double y = std::strtod(c_str, NULL);
  return geom::Point(x, y);
}

void geom::Point::sortBy(std::vector<Point> *vector, bool (*comparator) (const geom::Point& point1, const geom::Point& point2)) {
  sort(vector->begin(), vector->end(), comparator);
}

std::ostream& geom::operator<<(std::ostream& os, const geom::Point& point) {
  return os << point.toString() << std::endl;
}

std::istream& geom::operator>>(std::istream& is, geom::Point& point) {
  std::string line;
  getline(is, line);
  point = geom::Point::fromString(line);
  if(false) {
    is.setstate(std::ios::failbit);
  }
  return is;
}

bool geom::operator==(const geom::Point& left, const geom::Point& right) {
  return tools::Doubles::equals(left.x(), right.x()) && tools::Doubles::equals(left.y(), right.y());
}

bool geom::operator!=(const geom::Point& left, const geom::Point& right) {
  return !(geom::operator==(left, right));
}

geom::Point geom::operator-(const geom::Point& left, const geom::Point& right) {
  return Point(left.x() - right.x(), left.y() - right.y());
}

geom::Point geom::Points::FROM_STRING::apply(const std::string& input) const {
  return geom::Point::fromString(input);
}

double geom::Points::scalar(const geom::Point& left, const geom::Point& right) {
  return left.x() * right.x() + left.y() * right.y();
}

double geom::Points::zComponentOfCrossProduct(const geom::Point& left, const geom::Point& right) {
  return (left.x() * right.y()) - (left.y() * right.x());
}
