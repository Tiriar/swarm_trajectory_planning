#ifndef _POLYGON_HPP_

#define _POLYGON_HPP_

#include <vector>

#include "point.h"
#include "segment.h"

namespace geom {
  class Polygon {
    private:
      std::vector<geom::Point> _points;
    public:
      Polygon();
      Polygon(std::vector<geom::Point> points);
      std::vector<geom::Point> points() const;
      std::vector<geom::Segment> edges() const;
      static std::pair<Polygon, std::vector<Polygon> > loadFromFile(const std::string& fileName);
  };
}

#endif
