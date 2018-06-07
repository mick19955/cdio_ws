#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>

sensor_msgs::Image raw_image;
cv::Mat BGR_im;
cv::Mat HSV_im;
cv::Mat orig_image;
cv_bridge::CvImagePtr cv_ptr;

void load_new(const sensor_msgs::ImageConstPtr& input)
{
	raw_image = *input; 
  	ROS_INFO("Input from ardrone");
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

	ros::Rate loop_rate(25);

	int count = 0;

	while(ros::ok())
	{
		//printf("while ros::ok");
		
		if(cv_ptr){ //pointer is loaded, do image stuff
			
			BGR_im = cv_ptr->image; //now loaded into BGR
		
			//cloning the image for later use
			orig_image = BGR_im.clone();

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

			//std::cout << red_hue_image.rows/8 << std::endl;

			//red_hue_image.rows/8
				
			cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows, 100, 20, 0, 0);

			// Loop over all detected circles and outline them on the original image
			//if(circles.size() == 0) std::exit(-1);
				
			//std::cout << circles.size();

			for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
				cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
				int radius = std::round(circles[current_circle][2]);

				cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5);
			}
			cv::namedWindow("Threshold lower image", cv::WINDOW_AUTOSIZE);
			cv::imshow("Threshold lower image", lower_red_hue_range);
			cv::namedWindow("Threshold upper image", cv::WINDOW_AUTOSIZE);
			cv::imshow("Threshold upper image", upper_red_hue_range);
			cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
			cv::imshow("Combined threshold images", red_hue_image);
			cv::namedWindow("Detected red circles on the input image", cv::WINDOW_AUTOSIZE);
			cv::imshow("Detected red circles on the input image", orig_image);

			cv::waitKey(1);
			
		}
		else
		{
			//sleep(1);
//Do nothing
		}

	ros::spinOnce();
	loop_rate.sleep();
	count++;
	//printf("looping\n");
	}

	printf("\n\nExiting!\n\n");

  	return 0;
}
