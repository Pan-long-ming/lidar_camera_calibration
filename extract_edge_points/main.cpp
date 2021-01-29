#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/property_map.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Default_diagonalize_traits.h>
#include <CGAL/intersections.h>
#include <cassert>
#include <CGAL/Cartesian.h>
#include <CGAL/Ray_3.h>

#include <cstdlib>
#include <list>
#include <CGAL/Random.h>
#include <fstream>
#include <vector>
#include <limits>
#include <utility>

typedef CGAL::Simple_cartesian<float> Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel G;
typedef Kernel::FT FT;
typedef CGAL::Simple_cartesian<FT>  K;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Point_set_3<Point> Point_set;
typedef Point_set::Property_map<float> FT_map;
typedef K::Line_3 Line;
typedef std::array<double,6> Covariance;
typedef std::list<Point> Points;
typedef K::Plane_3      Plane;





int main(int argc, char **argv) 

{
    //input the pointset.
    Point_set point_set;
    Point_set point_set_extraction_1;
    Point_set point_set_extraction_2;
    
    std::ifstream f (argc > 1 ? argv[1] : "/home/pan/working/lidar_camera_calibration/lidar_data/robosense/example/extract_pro_plane_point_set - Cloud_labeled _2.ply",std::ios_base::binary); 
    if (!f || !CGAL::read_ply_point_set (f, point_set)) // same as `f >> point_set`
  {
    std::cerr << "Can't read input file " << std::endl;
  }
    //find the scalar_Original_cloud_index property.
    FT_map cloud_index;
    FT_map intensity;
    bool found_index = false; 
    bool found_int = false;
    boost::tie (cloud_index, found_index)  = point_set.property_map<float> ("scalar_Original_cloud_index");
    boost::tie(intensity,found_int) = point_set.property_map<float> ("scalar_intensity");
   
    Points points_collection_1;
    Points points_collection_2;
    Points plane_points;
    double k = 0;

    //find the edge points of triangle.
    while(k< 4)
   {
       

    for(size_t i = 0;i<point_set.size();i++)
    {
        
      float id_compare = cloud_index[i];


     if(id_compare  == k && i+1 < point_set.size() )
  {
          
        float intensity_compare_1 = intensity[i];
        float intensity_compare_2 = intensity[i+1];
        float compare = abs(intensity[i] - intensity[i+1]);
        if(compare > 13 &&  intensity[i+1] < 30)
        {
            
        Point point1 = point_set.point(i+1);
        points_collection_1.push_back(point1);
        point_set_extraction_1.insert(point1);

        std::cout<<"......................Point1s:"<< point1 <<std::endl;  
        
        }
        if(compare > 13 &&  intensity[i] < 30)
        {
            
        Point point2 = point_set.point(i);
        points_collection_2.push_back(point2);
        std::cout<<"......................Point2s:"<< point2 <<std::endl;
        point_set_extraction_2.insert(point2);

        
        }
  
 }
         
    }
    
    k=k+1;
   }

    //compute the line of triangle.
    Kernel kernel;
    FT quality;
    Point centroid;
    Line  line1;
    Line  line2;

    //quality = linear_least_squares_fitting_3(points.begin(),points.end(),plane,CGAL::Dimension_tag<0>());
    quality = linear_least_squares_fitting_3(points_collection_1.begin(),points_collection_1.end(),line1,centroid,CGAL::Dimension_tag<0>());
    
    quality = linear_least_squares_fitting_3(points_collection_2.begin(),points_collection_2.end(),line2,centroid,CGAL::Dimension_tag<0>());

    //3d point that has the shortest  distance to both lines
    // get plane p1 that contains l1 and is parallel to l2
    Plane p1( line1, line1.point(0) + line2.to_vector() );

    // get plane p2 that contains l1 and is perpendicular to p1

    Plane p2( line1, line1.point(0) + p1.orthogonal_vector() );

    // get intersection i1 between p2 and l2
    
    Point i1;
    CGAL::Object result = CGAL::intersection( p2, line2 );
    if( !assign( i1, result ) ) {
        Line il;
        if( assign( il, result ) )
               std::cout << "intersection between plane and line is a line --> l1 and l2 are parallel!" << std::endl;
                return -1.0;
                }

    // get intersection i2 on l1

    Point i2 = line1.projection( i1 );

    // final point is (i1+i2)/2

    Point intersection_point((i1.x() + i2.x()) / 2.0, (i1.y() + i2.y()) / 2.0, (i1.z() + i2.z()) / 2.0);

    std::cout<<"intersection_point:"<<intersection_point<<std::endl;
    point_set_extraction_2.insert(intersection_point);
    
    std::cout<<"line1:"<<line1<<" \nline2:"<<line2<<std::endl;

    std::ofstream out1("extract_point_set_1.ply");
    CGAL::write_ply_point_set(out1,point_set_extraction_1);
 
    std::ofstream out2("extract_point_set_2.ply");
    CGAL::write_ply_point_set(out2,point_set_extraction_2);  

    
}
