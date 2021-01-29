#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

int main()
{
	//Read the image;
    IplImage * input_image = cvLoadImage("/home/pan/Desktop/test.jpg");
    cvSmooth(input_image,input_image,CV_GAUSSIAN,3,3);
    std::vector<CvPoint> triangle_Points;
	//Convert to gray_image.
    IplImage* imgGrayScale = cvCreateImage(cvGetSize(input_image), 8, 1);  
	cvCvtColor(input_image,imgGrayScale,CV_BGR2GRAY);
    //cvNamedWindow("GrayScale Image");
    //cvShowImage("GrayScale Image",imgGrayScale);
    
    //image binaryzation
    cvThreshold(imgGrayScale,imgGrayScale,100,255,CV_THRESH_BINARY_INV);
    
    //cvNamedWindow("Thresholded Image");cvShowImage("Thresholded Image",imgGrayScale);
    
    //finding all the contours in the image
    CvSeq* contour;
    CvSeq* result;
    CvMemStorage *storage = cvCreateMemStorage(0);
    cvFindContours(imgGrayScale, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
    
    //iterating through each contour
    while(contour)
    {   
        //obtain a sequence of points of the countour, pointed by the variable 'countour'
        result = cvApproxPoly(contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour)*0.02, 0);
        
        if(result->total==3 && fabs(cvContourArea(result, CV_WHOLE_SEQ))>100 )
        {
            //iterating through eachpoint
            CvPoint *pt[3];
            for(int i=0;i<3;i++)
            {
                pt[i] = (CvPoint*)cvGetSeqElem(result, i);
                triangle_Points.push_back(*pt[i]);
                cvCircle(input_image,*pt[i],4,CvScalar(0,0,255),-1,8,0);

        }

        
    }
        contour = contour->h_next;   
        }
        

    CvFont font;  
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1, 1, 2, 8);  
    for(size_t i =0;i < triangle_Points.size();i++)
    {   
        
        // int change to char
        std::string pp = std::to_string(i);
        char p[0];
        p[0]=pp[0];
        cvPutText(input_image, p,triangle_Points[i], &font, CV_RGB(0,255,0));
        
        std::cout<<std::to_string(i)<<std::endl;
        std::cout<<"x:"<<triangle_Points[i].x<<",y:"<<triangle_Points[i].y<<std::endl;
    }
    

    //print the vertic of triangles and the  annotation of it 
        
    cvNamedWindow("Tracked");
    cvShowImage("Tracked",input_image);
    cvWaitKey(0);
    cvDestroyAllWindows();
    cvReleaseMemStorage(&storage);
    cvReleaseImage(&input_image);
    cvReleaseImage(&imgGrayScale);
    return 0;
    

}
