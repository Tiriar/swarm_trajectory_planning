#ifndef _SEGMENT_HPP_

#define _SEGMENT_HPP_

#include <vector>

#include "point.h"

namespace geom {
  class Segment {
     private:
       geom::Point pointA;
       geom::Point pointB;
     public:
       inline Segment() {}
       Segment(const geom::Point& pointA, const geom::Point& pointB);
       Segment(const Segment& other);
       Segment shorten(double shortening) const;
       inline geom::Point A() const {return pointA;}
       inline geom::Point B() const {return pointB;}
       double length() const;
       double atX(double x) const;
       std::pair<geom::Point, geom::Point> endpoints() const;
       bool static cellContains(std::vector<geom::Segment> &cell,geom::Point &point);
  };
  bool operator==(const Segment& left, const Segment& right);
  bool operator!=(const Segment& left, const Segment& right);
}

#endif
