#ifndef _VORONOI_HPP_
#define _VORONOI_HPP_

#include <vector>
#include <boost/polygon/voronoi.hpp>


#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTriangleFilter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkSmartPointer.h>

#include "functional.h"
#include "interactor2d.h"

#include <boost/polygon/isotropy.hpp>
#include <boost/polygon/point_concept.hpp>
#include <boost/polygon/segment_concept.hpp>


using boost::polygon::voronoi_diagram;


#include "point.h"
#include "polygon.h"
#include "segment.h"
#include "dijkstra_lite.h"




namespace vor{
class Voro{
private:
  std::vector<geom::Polygon> pgn_dta;


public:
//std::vector<Segment> segmentos;
std::vector<geom::Point> v_points;
    voronoi_diagram<double> vd;
    std::vector<geom::Segment> inputData;
    //void makeSegments(std::vector<geom::Segment>& lines);
    void constructDiagram(std::vector<geom::Segment>& segment_data);
    void printInfo();
    void fillPoints();
    std::vector<geom::Segment> returnDiagramSegments();
    vtkSmartPointer<vtkPolyData> insertLines(imr::dijkstra::CDijkstraLite<double> &dijkstra);
    vtkSmartPointer<vtkPolyData> insertPoint(int i);
    void buildGraph();
    void filterPoints();
    void setPolygons(std::vector<geom::Polygon>&pgn_dta);
};



}
#endif
