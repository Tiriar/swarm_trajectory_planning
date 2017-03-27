#ifndef _POINT_HPP_

#define _POINT_HPP_

#include <ostream>
#include <istream>
#include <string>
#include <vector>

#include "functional_def.h"

namespace geom {
  class Point {
    private:
      double _x;
      double _y;
    public:
      Point();
      Point(double x, double y);
      Point(const Point& other);
      inline double x() const {return _x;}
      inline double y() const {return _y;}
      double length() const;
      std::string toString() const;
      static Point fromString(const std::string& string);
      static bool compareByXFirst(const Point& point1, const Point& point2);
      static bool compareByYFirst(const Point& point1, const Point& point2);
      static bool compareByXFirstDecreasing(const Point& point1, const Point& point2);
      static bool compareByYFirstDecreasing(const Point& point1, const Point& point2);
      static void sortBy(std::vector<Point> *vector, bool (*comparator)(const Point& point1, const Point& point2));
  };
  std::ostream& operator<<(std::ostream& os, const Point& point);
  std::istream& operator>>(std::istream& is, Point& point);
  bool operator==(const Point& left, const Point& right);
  bool operator!=(const Point& left, const Point& right);
  Point operator-(const Point& left, const Point& right);

  class Points {
    public:
      class FROM_STRING : public :: tools::Function<std::string, Point> {
        Point apply(const std::string& input) const;
      };
      static double scalar(const Point& left, const Point& right);
      static double zComponentOfCrossProduct(const Point& left, const Point& right);
  };
}

#endif
