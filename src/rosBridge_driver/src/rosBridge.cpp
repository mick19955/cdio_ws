#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "std_msgs/Bool.h"
#include <opencv2/opencv.hpp>
#include <zbar.h>

using namespace cv;
using namespace std;
using namespace zbar;

sensor_msgs::Image raw_image;
cv::Mat BGR_im;
cv::Mat HSV_im;
cv::Mat orig_image;
cv_bridge::CvImagePtr cv_ptr;
std_msgs::Bool circle_detected;

typedef struct
{
  string type;
  string data;
  vector <Point> location;
} decodedObject;

// Find and decode barcodes and QR codes
void decode(Mat &im, vector<decodedObject>&decodedObjects)
{
  
	// Create zbar scanner
  	ImageScanner scanner;

	// Configure scanner
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
  
	// Convert image to grayscale
	Mat QR_im;
	cvtColor(im, QR_im,COLOR_BGR2GRAY);

  	// Wrap image data in a zbar image
  	Image image(im.cols, im.rows, "Y800", (uchar *)QR_im.data, im.cols * im.rows);

  	// Scan the image for barcodes and QRCodes
  	int n = scanner.scan(image);
  
  	// Print results
  	for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  	{
    		decodedObject obj;
    		
    		obj.type = symbol->get_type_name();
    		obj.data = symbol->get_data();
    
    		// Print type and data
    		std::cout << "Type : " << obj.type << std::endl;
    		std::cout << "Data : " << obj.data << std::endl << std::endl;
    
    		// Obtain location
		for(int i = 0; i< symbol->get_location_size(); i++)
		{
			obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
		}    
    decodedObjects.push_back(obj);
  }
}

// Display barcode and QR code location  
void display(Mat &im, vector<decodedObject>&decodedObjects)
{
  	// Loop over all decoded objects
  	for(int i = 0; i < decodedObjects.size(); i++)
  	{
    		vector<Point> points = decodedObjects[i].location;
    		vector<Point> hull;
    
    		// If the points do not form a quad, find convex hull
    		if(points.size() > 4){
			convexHull(points, hull);
		}    		
		else{
	      		hull = points;
		}    
    		// Number of points in the convex hull
    		int n = hull.size();
    
    		for(int j = 0; j < n; j++)
    		{
      			line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
    		}
    
  	}
}

void load_new(const sensor_msgs::ImageConstPtr& input)
{
	raw_image = *input; 
  	//ROS_INFO("Input from ardrone");
	try
	{
	    	cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		printf("got exception\n\n");
	    	ROS_ERROR("cv_bridge exception: %s", e.what());
	    	return;
	}
}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "Circle_Detect");
  	
 	ros::NodeHandle n;

	//initializing subscription and publisher
	ros::Subscriber sub = n.subscribe ("/ardrone/image_raw", 1, load_new);
	ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("/image_converter/output_video", 1);
	ros::Publisher circle_found_pub = n.advertise<std_msgs::Bool>("/Circle_found", 1);

	ros::Rate loop_rate(25);

	


	while(ros::ok())
	{
		
		if(cv_ptr){ //pointer is loaded, do image stuff
			
			// vector for all qr codes found 
			vector<decodedObject> decodedObjects;				

			BGR_im = cv_ptr->image; //Loading image ptr into cv::mat

			//cloning the image for so we can work with BGR
			orig_image = BGR_im.clone();
  
			// Finding QR codes from orig_image
			decode(orig_image, decodedObjects);

			// Adding outline of qr code to orig_image
			display(orig_image, decodedObjects);

			
			//circle detection starts here ---------------------
			cv::medianBlur(BGR_im, BGR_im, 3);

			cv::cvtColor(BGR_im, HSV_im, cv::COLOR_BGR2HSV);

			cv::Mat lower_red_hue_range;
			cv::Mat upper_red_hue_range;

			cv::inRange(HSV_im, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
			cv::inRange(HSV_im, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

			// Combine the above two images
			cv::Mat red_hue_image;
			cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

			cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

			// Use the Hough transform to detect circles in the combined threshold image
			std::vector<cv::Vec3f> circles;
				
			cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows, 100, 20, 40, 0); //find only one circle

			/*if(circles.size() == 1){ //if a circle is found
						//print the 3 values for the circle
				std::cout << circles[0][0] << "  ";
				std::cout << circles[0][1] << "  ";
				std::cout << circles[0][2] << std::endl;
			}*/
							
			if(circles.size() > 0){
				circle_detected.data = true;
			}else{
				circle_detected.data = false;
			}
			circle_found_pub.publish(circle_detected);

			// Loop over all detected circles and outline them on the original image
			for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
				cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
				int radius = std::round(circles[current_circle][2]);

				cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5); //actual drawing
			}
			cv::namedWindow("Detected red circles and QR", cv::WINDOW_AUTOSIZE);
			cv::imshow("Detected red circles and QR", orig_image);

			cv::waitKey(1);
			
		}
		else
		{
			//sleep(1);
			//Do nothing
		}

	ros::spinOnce();
	loop_rate.sleep();
	}

	printf("\n\nExiting!\n\n");

  	return 0;
}




