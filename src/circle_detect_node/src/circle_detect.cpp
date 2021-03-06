/*										
	Written by:	Group 2, 62410 CDIO, DTU Diplom
							Mikkel Ambjørn Boholdt-Pedersen
							Emily Skovgaard Rasmussen
							Nicolai André Stæhr Kruhøffer
							Jørn Salonin
							Simon Lundorf
							Robert Sand
	This code will detect red circles and outline them in a gui.
	The video feed is made available by ardrone_autonomy, when run while connected to the parrot ARDrone2

*/
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
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

using namespace cv;
using namespace std;


sensor_msgs::Image raw_image;
cv::Mat BGR_im;
cv::Mat HSV_im;
cv::Mat orig_image;
cv_bridge::CvImagePtr cv_ptr;
std_msgs::Bool circle_detected;


//prototype

void load_new(const sensor_msgs::ImageConstPtr& input); //loading new images gotten from the ARdrone into cv_ptr




int main(int argc, char** argv)
{
  	
	ros::init(argc, argv, "circle_detect");
  	
	ros::NodeHandle n;


	
	//initializing subscription and publisher
	ros::Subscriber sub 		= n.subscribe ("/ardrone/image_raw", 1, load_new);
	ros::Publisher circle_found_pub = n.advertise<std_msgs::Bool>("/circle_found", 1);


	ros::Rate loop_rate(25);

	while(ros::ok())
	{
		
		if(cv_ptr){ //pointer is loaded, do image stuff
		
			BGR_im = cv_ptr->image; //Loading image ptr into cv::mat

			//cloning the image for so we can work with BGR
			orig_image = BGR_im.clone();
  
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
							
			if(circles.size() > 0){
				circle_detected.data = true;
			}else{
				circle_detected.data = false;
			}
			circle_found_pub.publish(circle_detected);

			// Loop over all detected circles and outline them on the original image
			for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
				cv::Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
				int radius = round(circles[current_circle][2]);

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

void load_new(const sensor_msgs::ImageConstPtr& input) //loading new images gotten from the ARdrone into cv_ptr
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


