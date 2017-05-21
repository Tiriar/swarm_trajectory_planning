#ifndef _VTKTOOL_HPP_

#define _VTKTOOL_HPP_

#include <vector>

#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "point.h"
#include "polygon.h"
#include "segment.h"

namespace vtktool {
  class VTK {
    private:
      unsigned int point_id;
    public:
      vtkSmartPointer<vtkPolyData> insertPolygons(std::vector<geom::Polygon>& polygons);
      vtkSmartPointer<vtkPolyData> insertLines(std::vector<geom::Segment>& lines);
      vtkSmartPointer<vtkPolyData> insertMyCross();
      vtkSmartPointer<vtkPolyData> insertPoints(std::vector<geom::Point>& points);
      vtkSmartPointer<vtkActor> showPolygonsFrom(vtkSmartPointer<vtkPolyData>& data, float r, float g, float b, bool wireFrame = false);
      vtkSmartPointer<vtkActor> showPointsFrom(vtkSmartPointer<vtkPolyData>& data, unsigned int size, float r, float g, float b);
      vtkSmartPointer<vtkActor> showLinesFrom(vtkSmartPointer<vtkPolyData>& data, unsigned int width, float r, float g, float b);
      vtkSmartPointer<vtkActor> showMyCross(vtkSmartPointer<vtkPolyData>& data, unsigned int width, float r, float g, float b);
      void show(const std::vector<vtkSmartPointer<vtkActor> >& actors, unsigned int width, unsigned int height);
  };
}

#endif
