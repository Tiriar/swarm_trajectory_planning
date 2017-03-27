#include "vtktool.h"

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

#include "functional.h"
#include "interactor2d.h"

vtkSmartPointer<vtkPolyData> vtktool::VTK::insertPolygons(std::vector<geom::Polygon>& inputPolygons) {
  class : public :: tools::Function<geom::Polygon, std::vector<geom::Point> > {
    public: std::vector<geom::Point> apply(const geom::Polygon& input) const {
      return input.points();
  }} polygon2points;

  class : public :: tools::Predicate<geom::Polygon> {
    public: bool apply(const geom::Polygon& input) const {
      return input.points().size() > 0;
  }} notEmptyPolygons;

  std::vector<geom::Polygon> polygons = tools::Vectors::filter(inputPolygons, notEmptyPolygons);

  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkCellArray> vtk_polygons = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();

  std::vector<geom::Point> points = tools::Vectors::concat(tools::Vectors::transform(polygons, polygon2points));
  unsigned int point_id = 0;

  for (unsigned int i = 0; i < points.size(); i++) {
    vtk_points->InsertNextPoint(points[i].x(), points[i].y(), 0);
  }

  for (unsigned int i = 0; i < polygons.size(); i++) {
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
    polygon->GetPointIds()->SetNumberOfIds(polygons[i].points().size());
    for (unsigned int j = 0; j < polygons[i].points().size(); j++) {
      polygon->GetPointIds()->SetId(j, point_id++);
    }
    vtk_polygons->InsertNextCell(polygon);
  }
  data->SetPoints(vtk_points);
  data->SetPolys(vtk_polygons);

  return data;
}

vtkSmartPointer<vtkPolyData> vtktool::VTK::insertLines(std::vector<geom::Segment>& lines) {
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vtk_lines = vtkSmartPointer<vtkCellArray>::New();

  unsigned int point_id = 0;
  std::pair<geom::Point, geom::Point> segment;

  for (unsigned int i = 0; i < lines.size(); i++) {
    segment = lines[i].endpoints();
    vtk_points->InsertNextPoint(segment.first.x(), segment.first.y(), 0);
    vtk_points->InsertNextPoint(segment.second.x(), segment.second.y(), 0);
  }

  for (unsigned int i = 0; i < lines.size(); i++) {
    line->GetPointIds()->SetId(0, point_id++);
    line->GetPointIds()->SetId(1, point_id++);
    vtk_lines->InsertNextCell(line);
  }

  data->SetPoints(vtk_points);
  data->SetLines(vtk_lines);
  return data;
}

vtkSmartPointer<vtkPolyData> vtktool::VTK::insertMyCross() {
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vtk_lines = vtkSmartPointer<vtkCellArray>::New();

  unsigned int point_id = 0;
  std::pair<geom::Point, geom::Point> segment;

  // zacatek
  /*
  for (unsigned int i = 0; i < lines.size(); i++) {
    segment = lines[i].endpoints();
    vtk_points->InsertNextPoint(segment.first.x(), segment.first.y(), 0);
    vtk_points->InsertNextPoint(segment.second.x(), segment.second.y(), 0);
  }
  */
  // konec
  double x = 5;
  double y = 1000;
  vtk_points->InsertNextPoint(x,y, 0);
  x = 1995;
  y = 1000;
  vtk_points->InsertNextPoint(x,y, 0);
  x = 1000;
  y = 5;
  vtk_points->InsertNextPoint(x,y, 0);
  x = 1000;
  y = 1995;
  vtk_points->InsertNextPoint(x,y, 0);

  for (unsigned int i = 0; i < 2; i++) {  // lines.size() nahrazeno 2
    line->GetPointIds()->SetId(0, point_id++);
    line->GetPointIds()->SetId(1, point_id++);
    vtk_lines->InsertNextCell(line);
  }

  data->SetPoints(vtk_points);
  data->SetLines(vtk_lines);
  return data;
}

vtkSmartPointer<vtkPolyData> vtktool::VTK::insertPoints(std::vector<geom::Point>& points) {
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();

  for (unsigned int i = 0; i < points.size(); i++) {
    vtk_points->InsertNextPoint(points[i].x(), points[i].y(), 0);
  }

  data->SetPoints(vtk_points);
  return data;
}

vtkSmartPointer<vtkActor> vtktool::VTK::showPolygonsFrom(vtkSmartPointer<vtkPolyData>& data, float r, float g, float b, bool wireFrame) {
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

#if VTK_MAJOR_VERSION <= 5
  triangleFilter->SetInput(data);
#else
  triangleFilter->SetInputData(data);
#endif

  triangleFilter->Update();

#if VTK_MAJOR_VERSION <= 5
  mapper->SetInput(triangleFilter->GetOutput());
#else
  mapper->SetInputData(triangleFilter->GetOutput());
#endif
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(r, g, b);
  if(wireFrame) {
    actor->GetProperty()->SetRepresentationToWireframe();
  }
  return actor;
}

vtkSmartPointer<vtkActor> vtktool::VTK::showPointsFrom(vtkSmartPointer<vtkPolyData>& data, unsigned int size, float r, float g, float b) {
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();

#if VTK_MAJOR_VERSION <= 5
  vertexGlyphFilter->AddInput(data);
#else
  vertexGlyphFilter->AddInputData(data);
#endif
  vertexGlyphFilter->Update();

  mapper->SetInputConnection(vertexGlyphFilter->GetOutputPort());
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(size);
  actor->GetProperty()->SetColor(r, g, b);
  return actor;
}

vtkSmartPointer<vtkActor> vtktool::VTK::showLinesFrom(vtkSmartPointer<vtkPolyData>& data, unsigned int width, float r, float g, float b) {
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

#if VTK_MAJOR_VERSION <= 5
  mapper->SetInput(data);
#else
  mapper->SetInputData(data);
#endif

  actor->SetMapper(mapper);
  actor->GetProperty()->SetLineWidth(width);
  actor->GetProperty()->SetColor(r, g, b);
  return actor;
}

vtkSmartPointer<vtkActor> vtktool::VTK::showMyCross(vtkSmartPointer<vtkPolyData>& data, unsigned int width, float r, float g, float b) {
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

#if VTK_MAJOR_VERSION <= 5
  mapper->SetInput(data);
#else
  mapper->SetInputData(data);
#endif

  actor->SetMapper(mapper);
  actor->GetProperty()->SetLineWidth(width);
  actor->GetProperty()->SetColor(r, g, b);
  return actor;
}

void vtktool::VTK::show(const std::vector<vtkSmartPointer<vtkActor> >& actors, unsigned int width, unsigned int height) {
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(width, height);
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<imr::CInteractor2d> style = vtkSmartPointer<imr::CInteractor2d>::New();
  renderWindowInteractor->SetInteractorStyle(style);
  renderWindowInteractor->SetRenderWindow(renderWindow);
  for(unsigned int i = 0; i < actors.size(); i++) {
    renderer->AddActor(actors[i]);
  }
  renderWindow->Render();
  renderWindowInteractor->Start();
}



