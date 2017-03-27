#include "segment.h"

geom::Segment::Segment(const geom::Point& pointA, const geom::Point& pointB) {
  this->pointA = pointA;
  this->pointB = pointB;
}

geom::Segment::Segment(const geom::Segment& other) {
  this->pointA = other.pointA;
  this->pointB = other.pointB;
}

double geom::Segment::length() const {
  return (this->pointB - this->pointA).length();
}

double geom::Segment::atX(double x) const {
  return (x - this->pointA.x()) * (this->pointB.y() - this->pointA.y()) / (this->pointB.x() - this->pointA.x()) + this->pointA.y();
}

geom::Segment geom::Segment::shorten(double shortening) const {
  double length = this->length();
  if (length <= shortening) {
    return *this;
  }
  geom::Point A = geom::Point(this->A().x()+(shortening/length)*(this->B().x()-this->A().x()), this->A().y()+(shortening/length)*(this->B().y()-this->A().y()));
  geom::Point B = geom::Point(this->B().x()+(shortening/length)*(this->A().x()-this->B().x()), this->B().y()+(shortening/length)*(this->A().y()-this->B().y()));
  return geom::Segment(A, B);
}

std::pair<geom::Point, geom::Point> geom::Segment::endpoints() const {
  return std::make_pair(this->pointA, this->pointB);
}

bool geom::operator==(const geom::Segment& left, const geom::Segment& right) {
  return (left.endpoints().first == right.endpoints().first && left.endpoints().second == right.endpoints().second) ||
	 (left.endpoints().second == right.endpoints().first && left.endpoints().first == right.endpoints().second);
}

bool geom::operator!=(const geom::Segment& left, const geom::Segment& right) {
  return !(geom::operator==(left, right));
}

// Check whether the given point lays in the polygon
// From http://alienryderflex.com/polygon/
bool geom::Segment::cellContains(std::vector<geom::Segment> &cell,geom::Point &point)
{
  bool retval = false;
  int count = 0;
  double xx1, xx2, yy1, yy2;
  for(int i=0; i<cell.size(); i++)
  {
    xx1 = point.x()-cell[i].A().x();
    yy1 = point.y()-cell[i].A().y();
    xx2 = point.x()-cell[i].B().x();
    yy2 = point.y()-cell[i].B().y();
    if (xx1*xx1+yy1*yy1 < 0.01 || xx2*xx2+yy2*yy2 < 0.01) {
      return true;
    }
    if((cell[i].A().y()<point.y()&&cell[i].B().y()>=point.y())||(cell[i].B().y()<point.y()&&cell[i].A().y()>=point.y()))
    {
      if(cell[i].A().x()+(point.y()-cell[i].A().y())/(cell[i].B().y()-cell[i].A().y())*(cell[i].B().x()-cell[i].A().x())<point.x())
      {
        retval=!retval;
      }
    }
  }
  return retval;
}
