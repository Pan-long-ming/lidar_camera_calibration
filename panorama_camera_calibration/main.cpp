#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,8}; 

int main()
{
  // Creating vector to store vectors of 3D points for each checkerboard image
  std::vector<std::vector<cv::Point3f> > objpoints;

  // Creating vector to store vectors of 2D points for each checkerboard image
  std::vector<std::vector<cv::Point2f> > imgpoints;

  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for(int i{0}; i<CHECKERBOARD[1]; i++)
  {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
      objp.push_back(cv::Point3f(j,i,0));
  }


  // Extracting path of individual image stored in a given directory
  std::vector<cv::String> images;
  // Path of the folder containing checkerboard images
  std::string path = "/home/pan/projects/camera_calibration/images/*.jpg";

  cv::glob(path, images);

  cv::Mat frame, gray;
  // vector to store the pixel coordinates of detected checker board corners 
  std::vector<cv::Point2f> corner_pts;
  bool success;

  //define the number of interation and precision(EPS)
  cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);
  // Looping over all the images in the directory
  for(int i{0}; i<images.size(); i++)
  {
    frame = cv::imread(images[i]);
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
    // Finding checker board corners
    // If desired number of corners are found in the image then success = true  
    success = cv::findChessboardCorners(gray,cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

    
    /*
     * If desired number of corner are detected,
     * we refine the pixel coordinates and display 
     * them on the images of checker board
    */
    if(success)
    {
        


      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);

      // Displaying the detected corner points on the checker board
     //cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts,success);

      objpoints.push_back(objp);
      imgpoints.push_back(corner_pts);
    }

    cv::imshow("Image",frame);
    cv::waitKey(1);
  }

  cv::destroyAllWindows();

  cv::Mat K,xi,D,R,T,idx;

  /*
   * Performing camera calibration by 
   * passing the value of known 3D points (objpoints)
   * and corresponding pixel coordinates of the 
   * detected corners (imgpoints)
  */
  int f = 128;
  double rms = cv::omnidir::calibrate(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), K, xi, D, R, T, f, criteria, idx);

  std::cout << "cameraMatrix : " << K << std::endl;
  std::cout << "distCoeffs : " << D << std::endl;
  //std::cout << "Rotation vector : " << R << std::endl;
  //std::cout << "Translation vector : " << T << std::endl;
  
  //output to the wenjian.txt.
  std::ofstream fout("calibration_result.txt");
  fout << "cameraMatrix :\n " << K << std::endl;
  fout << "distCoeffs : " << D << std::endl;
  fout.close();
  return 0;
}
