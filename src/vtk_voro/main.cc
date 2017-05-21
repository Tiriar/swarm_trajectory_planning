#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <vector>
#include <ostream>
#include <iostream>
#include <fstream>


#include "vtktool.h"
#include "point.h"
#include "polygon.h"
#include "functional.h"
#include "voronoi.h"
#include "dijkstra_lite.h"


vtkSmartPointer<vtkPolyData> genPolyLine(vtkSmartPointer<vtkPolyData> data, std::vector<int> &idx) {
  vtkSmartPointer<vtkPolyData> result = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vtk_lines = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
  vtkPoints *pt = data->GetPoints();
  double tt[3];
  for(int i=0;i<idx.size();i++) {
    pt->GetPoint(idx[i],tt);
    vtk_points->InsertNextPoint(tt[0],tt[1],0);
  }

  for(int i=0;i<idx.size()-1;i++) {
    line->GetPointIds()->SetId(0, i);
    line->GetPointIds()->SetId(1, i+1);
    vtk_lines->InsertNextCell(line);
  }

  result->SetPoints(vtk_points);
  result->SetLines(vtk_lines);
  return result;
}



int main(int argc, char* argv[])
{
  if(argc < 1 || argc > 2) {
    std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl;
    return EXIT_FAILURE;
  }

  if(argc == 2) {
    class : public :: tools::Function<geom::Polygon, std::vector<geom::Segment> > {
      public: std::vector<geom::Segment> apply(const geom::Polygon& input) const {
        return input.edges();
    }} polygon2edges;


    std::pair<geom::Polygon, std::vector<geom::Polygon> > all_polygons = geom::Polygon::loadFromFile(argv[1]);

    std::vector<geom::Polygon> polygons = all_polygons.second;
    polygons.push_back(all_polygons.first);

    std::vector<geom::Segment> edges = tools::Vectors::concat(tools::Vectors::transform(polygons, polygon2edges));


    vtktool::VTK vtk = vtktool::VTK();
    //xxxxxxxxxxxxxxxxx voronoi class test
    vor::Voro voro;



    std::vector<vtkSmartPointer<vtkActor> > actors = std::vector<vtkSmartPointer<vtkActor> >();

    if(all_polygons.first.points().size() > 0) { // if the map has a border
      std::vector<geom::Polygon> border = std::vector<geom::Polygon>();
      border.push_back(all_polygons.first);
      vtkSmartPointer<vtkPolyData> border_data = vtk.insertPolygons(border);
      actors.push_back(vtk.showPolygonsFrom(border_data, .3, .3, .3));
      actors.push_back(vtk.showPointsFrom(border_data, 5, 0, 0, 1));
    }


    vtkSmartPointer<vtkPolyData> data = vtk.insertPolygons(all_polygons.second);
    vtkSmartPointer<vtkPolyData> edges_data = vtk.insertLines(edges);
//    std::cout << "CONSTRUCT VD" << std::endl;

    voro.constructDiagram(edges); //construct Voronoi diagram
    voro.setPolygons(all_polygons.second); //set polygons to filter points lying inside these polygons

    imr::dijkstra::CDijkstraLite<double> dijkstra;
//    std::cout << "INSERT LINES" << std::endl;
    vtkSmartPointer<vtkPolyData> voro_edges = voro.insertLines(dijkstra); //filter the diagram

/*
//compute Dijkstra
    std::cout << "COMPUTE DIJKSTRA" << std::endl;
    std::vector<long int> pred;
    std::vector< imr::dijkstra::CDijkstraLite<double>::Weight> ww;
    dijkstra.solve(0, pred, ww);

    int cc = pred.size()-10;//550;//500;

//retrieve the path
    std::vector<int> idx;
    while(cc != -1) {
      std::cout << cc << "(" << pred[cc] << ") ";
      idx.push_back(cc);
      cc = pred[cc];
    }

    vtkSmartPointer<vtkPolyData> path = genPolyLine(voro_edges, idx);
*/

	// write output to file
	std::ofstream myfile("voro_output.txt");
	for (int i = 0; i < voro_edges->GetNumberOfPoints(); i++) {
		double* myPoint = voro_edges->GetPoint(i);
		myfile << myPoint[0] << " " << myPoint[1] << std::endl;
	}
	myfile << "---" << std::endl;

	voro_edges->GetLines()->InitTraversal();
	vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
	while (voro_edges->GetLines()->GetNextCell(idList)) {
		for (vtkIdType pointId = 0; pointId < idList->GetNumberOfIds(); pointId++) {
			myfile << idList->GetId(pointId) << " ";
		}
		myfile << std::endl;
	}
	myfile.close();

    actors.push_back(vtk.showPolygonsFrom(data, .8, .8, .8));
//    actors.push_back(vtk.showLinesFrom(edges_data, 3, 0, .5, 0));
    actors.push_back(vtk.showPointsFrom(data, 5, 0, 1, 0));

    actors.push_back(vtk.showLinesFrom(voro_edges, 1, 1, 1, 0));
    actors.push_back(vtk.showPointsFrom(voro_edges, 5, 1, 1, 0));

//    actors.push_back(vtk.showLinesFrom(path, 3, 1, 0, 0));


//     vtkSmartPointer<vtkPolyData> start = voro.insertPoint(3);
//     actors.push_back(vtk.showPointsFrom(start, 10, 1, 0, 0));
//
//     vtkSmartPointer<vtkPolyData> goal = voro.insertPoint(300);
//     actors.push_back(vtk.showPointsFrom(goal, 10, 0, 0, 1));


    vtk.show(actors, 1200, 900);
  }


  return EXIT_SUCCESS;
}
