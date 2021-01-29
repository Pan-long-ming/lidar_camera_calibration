#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/property_map.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Default_diagonalize_traits.h>
#include <cassert>

#include <cstdlib>
#include <list>
#include <CGAL/Random.h>
#include <fstream>
#include <vector>
#include <limits>
#include <utility>


typedef CGAL::Simple_cartesian<float> Kernel;
typedef Kernel::FT FT;
typedef CGAL::Simple_cartesian<FT>  K;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Point_set_3<Point> Point_set;
typedef Point_set::Property_map<float> FT_map;
typedef K::Plane_3                  Plane;
typedef K::Line_3 Line;

typedef std::array<double,6> Covariance;
typedef std::list<Point> Points;


void fit_point_set(std::list<Point>& points,Plane& plane);
void plane_projection_point(std::list<Point>& points,Plane& plane,std::vector<float>& intensities);
void lines_projection_point(std::list<Point>& points,Line& line);

int main (int argc, char**argv)
{
    FT_map intensity;
    Point_set point_set;
    bool found = false;
    Points points1;

    //read the input file.
    std::ifstream f (argc > 1 ? argv[1] : "/home/pan/working/lidar_camera_calibration/lidar_data/robosense/example/r_1611060353794911_two_triangle.ply",std::ios_base::binary); 
    if (!f || !CGAL::read_ply_point_set (f, point_set)) // same as `f >> point_set`
  {
    std::cerr << "Can't read input file " << std::endl;
  }
  
  //find intensity property.
    boost::tie (intensity, found)  = point_set.property_map<float> ("scalar_intensity");
    if(! found)
    {
        std::cout<<"Erroe:property not found!"<<std::endl;
    }
    std::cout<<"Successfully property  found!"<<std::endl;

    //find the points which intensity is below 70.
    std::vector<float> intensities;
    for(size_t i = 0;i<point_set.size();i++)
   {
       
      float intensity_compare = intensity[i];
      intensities.push_back(intensity_compare);

      if(intensity_compare <70)
     
      {
         
          Point m_point = point_set.point(i);
          points1.push_back(m_point);
          
       }    
       
   }
   

  //fitting plane  
  Plane plane;  
  
  fit_point_set(points1,plane);
  std::cout<<"plane:"<<plane<<std::endl;

  //projection point onto plane
  plane_projection_point(points1,plane,intensities);

  
  
  
  
  
  std::cout<<"Successfully !"<<std::endl;
  
  
  return 0;
}




void fit_point_set(std::list<Point>& points,Plane& plane)
{
  // fit a plane
  // call all versions of the function
  Kernel kernel;
  FT quality;
  Point centroid;


  
  std::cout << "fit 3D plane...";
  //quality = linear_least_squares_fitting_3(points.begin(),points.end(),plane,CGAL::Dimension_tag<0>());
  quality = linear_least_squares_fitting_3(points.begin(),points.end(),plane,centroid,CGAL::Dimension_tag<0>());
 

  std::cout << "done (quality: " << quality << ")" << std::endl;
}

void plane_projection_point(std::list<Point>& points,Plane& plane,std::vector<float>& intensities)
{   
    //projection points onto the plane.
    FT_map intensity;
    Points plane_pro_points;
    Point_set extract_pro_plane_point_set;
    for(std::list<Point>::iterator i = points.begin();i != points.end();++i)
    {
        plane_pro_points.push_back(plane.projection(*i));
        extract_pro_plane_point_set.insert(plane.projection(*i));
    }
       
    //add the original property of intensity.
    bool success = false;
    boost::tie (intensity, success) = extract_pro_plane_point_set.add_property_map<FT> ("intensity", 0.);
    assert (success);
    int k = 0;
    for(Point_set::iterator i = extract_pro_plane_point_set.begin();i !=
        extract_pro_plane_point_set.end();++i)
    {

        intensity[*i] = intensities[k];
        
        k = k+1;
    }   
   

     //ouput the projection points of the plane.
     std::ofstream out("extract_pro_plane_point_set.ply");
     CGAL::write_ply_point_set(out,extract_pro_plane_point_set);
     std::cout << "successfully output the plane's data ! "  ;
}
