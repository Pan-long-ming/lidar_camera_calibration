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
typedef K::Line_3 Line;
typedef std::array<double,6> Covariance;
typedef std::list<Point> Points;

int main(int argc, char **argv) 
{
    //input the pointset.
    Point_set point_set;
    std::ifstream f (argc > 1 ? argv[1] : "/home/pan/working/lidar_camera_calibration/lidar_data/robosense/example/extract_pro_plane_point_set _labeled.ply",std::ios_base::binary); 
    if (!f || !CGAL::read_ply_point_set (f, point_set)) // same as `f >> point_set`
  {
    std::cerr << "Can't read input file " << std::endl;
  }
    //find the scalar_Original_cloud_index property.
    FT_map cloud_index;
    FT_map first_intensity;
    FT_map final_intensity;
    bool found_index = false; 
    bool found_int = false;
    boost::tie (cloud_index, found_index)  = point_set.property_map<float> ("scalar_Original_cloud_index");
    boost::tie(first_intensity,found_int) = point_set.property_map<float> ("scalar_intensity");
    if(found_index)
    {
        std::cout<<"Successfully,find the cloud_index of cloudpoints."<<std::endl;
    }
    if(found_index)
    {
        std::cout<<"Successfully,find the intensity of cloudpoints."<<std::endl;
    }
    
    
    //extract the intensity of the point_cloud
    std::vector<float> intern_intensities;
    for(size_t i = 0;i<point_set.size();i++)
   {
       
      float intensity_compare = first_intensity[i];
      intern_intensities.push_back(intensity_compare);

   }
    
    //projection points onto lines. 
   Point_set extract_pro_plane_point_set;
   Points projection_points;
   float k = 0;

   while(k<10)
   {
       
    Points points1;
    
    for(size_t i = 0;i<point_set.size();i++)
    {
        
      float id_compare = cloud_index[i];
      
      if(id_compare  == k)
      {

         Point m_point = point_set.point(i);
         points1.push_back(m_point);
    }
        
    }
    
    Kernel kernel;
    FT quality;
    Point centroid;
    Line  line;
  
    std::cout << "fit 3D line..."<<std::endl;
    //quality = linear_least_squares_fitting_3(points.begin(),points.end(),plane,CGAL::Dimension_tag<0>());
    quality = linear_least_squares_fitting_3(points1.begin(),points1.end(),line,centroid,CGAL::Dimension_tag<0>());
    
    for(std::list<Point>::iterator i = points1.begin();i != points1.end();++i)
    {

        extract_pro_plane_point_set.insert(line.projection(*i));
    }
    k=k+1;
   }

    // add the original intensity back.
    bool success = false;
    boost::tie (final_intensity, success) = extract_pro_plane_point_set.add_property_map<FT> ("intensity", 0.);
    assert (success);
    int j = 0;
    for(Point_set::iterator i = extract_pro_plane_point_set.begin();i != extract_pro_plane_point_set.end();++i)
    {

        final_intensity[*i] = intern_intensities[j];
        
        j = j+1;
    }   
   
  //ouput the fit line.    
  std::ofstream out("extract_pro_plane_point_set.ply");
  CGAL::write_ply_point_set(out,extract_pro_plane_point_set);
  std::cout << "successfully output the plane's data ! "  ;
  
  
}
