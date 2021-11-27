#ifndef SCANNER_H
#define SCANNER_H


#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <zbar.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>    //Converting ROS to OpenCV images
#include <cv_bridge/cv_bridge.h>            //Converting ROS to OpenCV images
#include <image_transport/image_transport.h>//Publishing and subscribing to images in ROS
#include <opencv2/imgproc/imgproc.hpp>      //Converting ROS to OpenCV images
#include "opencv2/core/core.hpp"            //OpenCV Stitching
#include "opencv2/features2d/features2d.hpp"//OpenCV Stitching
#include "opencv2/highgui/highgui.hpp"      //OpenCV Stitching 
//#include "opencv2/nonfree/nonfree.hpp"      //OpenCV Stitching
#include "opencv2/calib3d/calib3d.hpp"      //OpenCV Stitching
#include "opencv2/imgproc/imgproc.hpp"      //OpenCV Stitching

#include "for_dyn.h"

using namespace cv;
using namespace std;
using namespace zbar;





class SCANNER{

  public:

    

  SCANNER(QUADRUPED &quadruped);

  typedef struct
  {
    string type;
    string data;
    vector <Point> location;
  } decodedObject;

  void decode(Mat &im, vector<decodedObject>&decodedObjects);

  void display(Mat &im, vector<decodedObject>&decodedObjects);

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void qrcode_scanning();



  private:

  QUADRUPED*  dogbot;

  ros::Subscriber _sub_scanner;

  image_transport::Subscriber sub;

  Mat _im;

  //double _block =false;

  bool _image_loaded=false;

  





};



#endif