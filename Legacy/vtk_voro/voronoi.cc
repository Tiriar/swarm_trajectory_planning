
#include <vector>

#include <boost/polygon/voronoi.hpp>
#include <limits>

#include "voronoi.h"


using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;



namespace boost {
namespace polygon {

template <>
struct geometry_concept<geom::Point> { typedef point_concept type; };

template <>
struct point_traits<geom::Point> {
  typedef int coordinate_type;
	// Pozor na toto !!!!!!!!!!! Pretypovani na int. mapz jsou sice v double ale muze delat problemy????
  static inline coordinate_type get(const geom::Point& point, orientation_2d orient) {
    //return (orient == HORIZONTAL) ? (int)point.x() : (int)point.y();
    return (orient == HORIZONTAL) ? point.x() : point.y();
  }
};

template <>
struct geometry_concept<geom::Segment> { typedef segment_concept type; };

template <>
struct segment_traits<geom::Segment> {
  typedef int coordinate_type;
  typedef geom::Point point_type;


  static inline point_type get(const geom::Segment& segment, direction_1d dir) {
    return dir.to_int() ? segment.A() : segment.B();
  }
};


}}

void vor::Voro::printInfo(){
	cout << "Number of cells: " << vd.num_cells() << "\n";

	cout << "Number of edges: " << vd.num_edges() << "\n";

	cout << "Number of nodes: " << vd.num_vertices() << "\n";
	for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin();
         it != vd.vertices().end(); ++it) {
    cout << "X:" << it->x() << "Y:" << it->y() << "\n";}
}

void vor::Voro::fillPoints(){
  v_points.clear();
	for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin();
         it != vd.vertices().end(); ++it){
	v_points.push_back(geom::Point(it->x(),it->y()));
	//cout << "X:" << it->x() << "Y:" << it->y() << "\n";
	}
}



void vor::Voro::constructDiagram(std::vector<geom::Segment>& segment_data){
  inputData.resize(segment_data.size());
  std::copy(segment_data.begin(), segment_data.end(),inputData.begin());
construct_voronoi(segment_data.begin(), segment_data.end(),&vd);
}



std::vector<geom::Segment> vor::Voro::returnDiagramSegments(){

std::vector<geom::Segment> segmentator;
return segmentator;

}

// #include <boost/graph/graph_traits.hpp>
// #include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/dijkstra_shortest_paths.hpp>
//
// using namespace boost;
// typedef adjacency_list < listS, vecS, directedS, no_property, property < edge_weight_t, int > > graph_t;
// typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
// typedef graph_traits < graph_t >::edge_descriptor edge_descriptor;
// typedef std::pair<int, int> Edge;



 void vor::Voro::buildGraph() {
   /*
  int n = 0;
  for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
    it->color(n++);
  }
  graph_t graph;


  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
    if ( it->is_finite() && it->is_primary() ) {
      if ( it->vertex0()->color() <= it->vertex1()->color() )   {
        add_edge(it->vertex0()->color(), it->vertex1()->color(), graph);
      }
    }
  }
//  write_graphviz(std::cout, graph);//,boost::make_label_writer(boost::get(&vert::pos, graph)));
// print boost graph structure

std::ofstream fout("fig.dot");
fout << "graph A {\n"
<< "  rankdir=LR\n"
<< "size=\"5,3\"\n"
<< "ratio=\"fill\"\n"
<< "edge[style=\"bold\" fontsize=21, fontcolor=\"blue\",fontname=\"Times-Roman bold\"]\n"
<< "node[shape=\"circle\" , width=1.6, fontsize=21, fillcolor=\"yellow\", style=filled]\n";


typedef boost::graph_traits<BoostGraph>::vertex_iterator vertex_iter;
std::pair<vertex_iter, vertex_iter> vp;
for (vp = vertices(graph); vp.first != vp.second; ++vp.first)
  fout << vp.first->color()  << std::endl;




// boost::graph_traits < BoostGraph >::edge_iterator ei, ei_end;
// for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei)
//   fout << nodes[boost::source(*ei, g)].name << " -- " << nodes[boost::target(*ei, g)].name
//   //    fout << boost::source(*ei, g) << " -> " << boost::target(*ei, g)
//   << "[label=" << boost::get(boost::edge_weight, g)[*ei] << "]\n";

fout << "}\n";
fout.close();
//system("dot -Tpdf  -n  fig.dot > fig.pdf");



*/
}




vtkSmartPointer<vtkPolyData> vor::Voro::insertLines(imr::dijkstra::CDijkstraLite<double> &dijkstra) {
  static const int INVALID_NODE = std::numeric_limits<short>::max();
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vtk_lines = vtkSmartPointer<vtkCellArray>::New();
  std::pair<geom::Point, geom::Point> segment;
  std::vector<geom::Segment> tmp;
  int n = 0;

//invalidate points outside polygons
  for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
    bool inside = false;
    geom::Point pt = geom::Point(it->x(), it->y());
    for(int k=0;k<pgn_dta.size();k++) {
      tmp = pgn_dta[k].edges();
      if(geom::Segment::cellContains(tmp,pt)) {
          inside = true;
      }
    }
    if (inside) {
      it->color(INVALID_NODE);
    } else {
      it->color(n++);
    }
  }

std::vector<int> rank(n,0);


// invalidate points with rank < 2 and edges incedent to these
// repeat the process until there is no such point
bool finish;
do {
  finish = true;
  std::fill (rank.begin(),rank.end(),0);

  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
    if (it->is_finite() && it->is_primary() &&  it->vertex0()->color() <= it->vertex1()->color() && it->vertex0()->color() != INVALID_NODE && it->vertex1()->color() != INVALID_NODE)   {
      rank[it->vertex0()->color()]++;
      rank[it->vertex1()->color()]++;
    }
  }

  for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
    if (it->color() != INVALID_NODE && rank[it->color()] <2) {
      it->color(INVALID_NODE);
      finish = false;
    }
  }
} while (!finish);

// insert only nodes not marked to be invalid
  n=0;
  for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
    if (it->color() != INVALID_NODE) {
      vtk_points->InsertNextPoint(it->x(), it->y(),0);
      it->color(n++);
    }
  }

// insert edges of VD connecting two valid nodes
  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
    if ( it->is_finite() && it->is_primary() ) {
      if ( it->vertex0()->color() <= it->vertex1()->color() && it->vertex0()->color() != INVALID_NODE && it->vertex1()->color() != INVALID_NODE)   {
        std::size_t index = it->cell()->source_index();
        geom::Point p0 = inputData[index].A();
        geom::Point p1 = inputData[index].B();

		/*
        if (it->cell()->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
          std::cout << "Cell contains segment start point: (" << p0.x() <<", " << p0.y() << ")" << std::endl;
        } else if (it->cell()->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) {
          std::cout << "Cell contains segment end point:  (" << p0.x() <<", " << p0.y() << ")" << std::endl;
        } else {
          std::cout <<"Cell contains a segment: (" << p0.x() <<", " << p0.y() << "), (" << p1.x() <<", " << p1.y() << ")"  << std::endl;
        }
		*/

        line->GetPointIds()->SetId(0, it->vertex0()->color());
        line->GetPointIds()->SetId(1, it->vertex1()->color());
        double xx = it->vertex0()->x() - it->vertex1()->x();
        double yy = it->vertex0()->y() - it->vertex1()->y();
        double dd = sqrt(xx*xx+yy*yy);
        dijkstra.addEdge(it->vertex0()->color(),it->vertex1()->color(),dd);
        dijkstra.addEdge(it->vertex1()->color(),it->vertex0()->color(),dd);
//        std::cout << "EDGE " << it->vertex0()->color() << " " << it->vertex1()->color() << " " << dd << std::endl;
        vtk_lines->InsertNextCell(line);
      }
    }
  }

  data->SetPoints(vtk_points);
  data->SetLines(vtk_lines);
  return data;
}


vtkSmartPointer<vtkPolyData> vor::Voro::insertPoint(int i) {
    vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();

  vtk_points->InsertNextPoint(vd.vertices()[i].x(), vd.vertices()[i].y(),0);

  data->SetPoints(vtk_points);


  return data;
}

void vor::Voro::setPolygons(std::vector<geom::Polygon>&pgn_d)
{
  pgn_dta = pgn_d;
}


void vor::Voro::filterPoints()
{

  std::vector<int> ers;
  std::vector<geom::Segment> tmp;
//   std::cout << "ERR SIZES " << pgn_dta.size() << " " << v_points.size()  << std::endl;
  for(int k=0;k<pgn_dta.size();k++)
  {
    tmp = pgn_dta[k].edges();
    for(int i=0;i<v_points.size();i++)
    {
//       std::cout << "ERR " << k << " " << i  << std::endl;
      if(geom::Segment::cellContains(tmp,v_points[i]))
      {
        ers.push_back(i);
      }
    }
  }
  std::sort(ers.begin(),ers.end());
  int ind;
  std::cout << "ERASE SIZE " << ers.size() << std::endl;
  for(int i = ers.size()-1;i>=0;i--)
  {
    v_points.erase(v_points.begin()+ ers[i]);
    std::cout << "ERASE " << ers[i] << std::endl;
  }


}
