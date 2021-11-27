#include "scanner.h"

using namespace cv;
using namespace std;
using namespace zbar;


SCANNER::SCANNER(QUADRUPED &quadruped)
{
  dogbot=&quadruped;

  image_transport::ImageTransport it(dogbot->get_nh());

  sub = it.subscribe("/dogbot/camera/image_raw", 1, &SCANNER::imageCallback, this);
  
  
    

}

// Find and decode barcodes and QR codes
void SCANNER::decode(Mat &im, vector<decodedObject>&decodedObjects)
{
  
  // Create zbar scanner
  ImageScanner scanner;

  // Configure scanner
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
  
  // Convert image to grayscale
  Mat imGray;
  cvtColor(im, imGray, COLOR_BGR2GRAY);

  // Wrap image data in a zbar image
  Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);
  
  // Print results
  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;
    
    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();
    
    // Print type and data
    cout << "Type : " << obj.type << endl;
    cout << "Data : " << obj.data << endl << endl;
    
    // Obtain location
    for(int i = 0; i< symbol->get_location_size(); i++)
    {
      obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
    }
    
    decodedObjects.push_back(obj);

    std::string str1 ("a1");
    std::string str2 ("a2");
    std::string str3 ("a3");

    std::cout<<"**********QR-CODE SCANNER**********"<<endl;
    std::cout<<"Scanning..."<<endl;

    if (str1.compare(obj.data) == 0){
      std::cout<<"Target a1  found!"<<endl;
    }
    else if(str2.compare(obj.data) == 0){
      std::cout<<"Target a2 found!"<<endl;
    }
    else if(str3.compare(obj.data) == 0){
      std::cout<<"Target a3 found!"<<endl;
    }

    std::cout<<"Decoding done!"<<endl;
    std::cout<<"***********************************"<<endl;



  }
}

// Display barcode and QR code location  
void SCANNER::display(Mat &im, vector<decodedObject>&decodedObjects)
{
  // Loop over all decoded objects
  for(int i = 0; i < decodedObjects.size(); i++)
  {
    vector<Point> points = decodedObjects[i].location;
    vector<Point> hull;
    
    // If the points do not form a quad, find convex hull
    if(points.size() > 4)
      convexHull(points, hull);
    else
      hull = points;
    
    // Number of points in the convex hull
    int n = hull.size();
    
    for(int j = 0; j < n; j++)
    {
      line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
    }
    
  }
  
  // Display results 
  imshow("Results", im);
  waitKey(0);
  
}

void SCANNER::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


  //namedWindow("view"); 
  //while(!_block){
  //  std::cout<<"Waiting for walk..."<<endl;
  //}

  cv_bridge::CvImageConstPtr cv_ptr;
  //cv_bridge::CvImagePtr cv_ptr;

  try
  {      
    //imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    //while(!cv_ptr){
    //  std::cout<<"Waiting for pointer getting filled..."<<endl;
    //}

    _im = cv_ptr->image;

    _image_loaded=true;
           
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  //waitKey(0);

    

}

void SCANNER::qrcode_scanning()
{

  while(!_image_loaded){
    std::cout<<"Waiting for image callback..."<<endl;
  }
  

  // Variable for decoded objects 
  vector<decodedObject> decodedObjects;
  
  // Find and decode barcodes and QR codes
  decode(_im, decodedObjects);


  // Display location 
  //display(_im, decodedObjects);

}

