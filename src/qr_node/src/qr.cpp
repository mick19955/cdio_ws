/*			
	Edited by:	Group 2, 62410 CDIO, DTU Diplom
							Mikkel Ambjørn Boholdt-Pedersen
							Emily Skovgaard Rasmussen
							Nicolai André Stæhr Kruhøffer
							Jørn Salonin
							Simon Lundorf
							Robert Sand
							
	This code will scan the videofeed frame by frame, for QR codes
	If a code is found then the text from the QR code will be translated
	Furthermore an approximate of the distance to the QR code will be calculated as well as a left-right
	displacement of the drone, relative to the detected QR code
	
	The found QR code will be outlined on a GUi window
	
	All the extracted data will be published in an int array at "/qr_info_array"
	
	Original implementation by: https://github.com/andreaslorentzen/dronemis2.0
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
using namespace zbar;

sensor_msgs::Image raw_image;
cv::Mat BGR_im;
cv::Mat HSV_im;
cv::Mat orig_image;
cv_bridge::CvImagePtr cv_ptr;
std_msgs::Bool circle_detected;
int relative_pos_to_QR;
int yDiversionAngle;
double y2Diversion;
double y1Diversion;
double distance_to_QR[200];
std_msgs::Int32MultiArray QR_info;

typedef struct
{
  string type;
  string data;
  vector <Point> location;
} decodedObject;


//prototype
void init_distance();
double calculatedistance_to_QR(int pixels);
void load_new(const sensor_msgs::ImageConstPtr& input); //loading new images gotten from the ARdrone into cv_ptr
void decode(Mat &im, vector<decodedObject>&decodedObjects);
void display(Mat &im, vector<decodedObject>&decodedObjects);




int main(int argc, char** argv)
{
  	
	ros::init(argc, argv, "qr");
  	
	ros::NodeHandle n;
	
	//initializing subscription and publisher
	ros::Publisher QR_info_pub 	= n.advertise<std_msgs::Int32MultiArray>("/qr_info_array", 100);
	ros::Subscriber sub 		= n.subscribe ("/ardrone/image_raw", 1, load_new);


	ros::Rate loop_rate(50);

	init_distance(); //initializes crazy stupid array


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

			//publishing the decoded information
			QR_info_pub.publish(QR_info);			

			// Adding outline of qr code to orig_image
			display(orig_image, decodedObjects);
			
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

// Find and decode barcodes and QR codes
void decode(Mat &im, vector<decodedObject>&decodedObjects)
{
	//init variables
	int distance_to_QR = 0;  
	int x_Distance = 0;
	int y_Distance = 0;
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
  	int number_of_detections = scanner.scan(image);
  
	int x_0, x_1, x_2, x_3, y_0, y_1, y_2, y_3, x_size, y_size, x_middle, y_middle, QR_size;
	vector<Point> vp;	

	QR_info.data.clear();
	
	for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {


		//Used to get QR information so we can draw it on the image
		decodedObject obj;

    		obj.type = symbol->get_type_name();
    		obj.data = symbol->get_data();

	  	// Obtain location
		for(int i = 0; i< symbol->get_location_size(); i++)
		{
			obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
		}    
    		decodedObjects.push_back(obj);
		//Used to get QR information so we can draw it on the image ----------------------------



            // do something useful with results
            int n = symbol->get_location_size();        // Returns 4 if QR code is scanned
            for (int i = 0; i < n; i++) {
                vp.push_back(Point(symbol->get_location_x((unsigned int)i), symbol->get_location_y((unsigned int)i)));      //Update VP
             }

            x_0 = symbol->get_location_x(0);    //
            x_1 = symbol->get_location_x(1);    //
            x_2 = symbol->get_location_x(2);    //
            x_3 = symbol->get_location_x(3);    //  Saves the size of the QR code being detected
            y_0 = symbol->get_location_y(0);    //
            y_1 = symbol->get_location_y(1);    //
            y_2 = symbol->get_location_y(2);    //
            y_3 = symbol->get_location_y(3);    //
            x_size = (abs((x_0 - x_2)) + abs((x_1 - x_3))) / 2;                          // Calculate Vertical size of the QR
            y_size = (abs((y_0 - y_2)) + abs((y_3 - y_1))) / 2;                          // Calculate Horizontal size of the QR
            x_middle = (x_0 + x_1 + x_2 + x_3) / 4;                                        // The horizontal middle of the QR
	    y_middle = (y_0 + y_1 + y_2 + y_3) / 4; 

            QR_size = (x_size + y_size) / 2;                                           // The size of the QR
            double distance_to_QRdouble = calculatedistance_to_QR(QR_size);               // To prevent errors when typecasting
            distance_to_QR = (int)distance_to_QRdouble; // double to int
            std::string QR_name = symbol->get_data();

            double y_left, y_right, y_temp, y_ratio;

            y_left = y_1 - y_0;                                                        // calculate left side of QR
            y_right = y_2 - y_3;                                                       // Calculate right side of QR

            if (y_left > y_right) {
                relative_pos_to_QR = 1;                                                      // If relative_pos_to_QR is 1 - Drone is left of the QR code
            }
            else if (y_left == y_right) {
                relative_pos_to_QR = 0;                                                      // If relative_pos_to_QR is 0 - Drone is in front of QR code
            }
            else {
                relative_pos_to_QR = -1;                                                     // If relative_pos_to_QR is -1, Drone is right of the QR code.
                y_temp = y_left;
                y_left = y_right; // Swap
                y_right = y_temp;
            }

            y_ratio = y_left / y_right;
	    
	    //used to check if drone is looking at the QR from an angle
            y1Diversion = (y_ratio * 360.0395) - 359.2821;                           // Two different equations that sum up the angle of the QR
            y2Diversion = (y_ratio * 637.3656) - 642.2072;                           //


            cout << "distance to the QR code = " << distance_to_QR << "cm, with the text: " << QR_name << endl << endl;

            double x_DistanceStatic = 3.61194;                                       // Calculated using 243 pixels for 67cm on 150cm distance.
            double x_Distancedouble = ((x_middle - 320) / x_DistanceStatic * distance_to_QR / 150); // Calculate in cm's where the center of the camera is, according to the center of the QR
            x_Distance = (int) x_Distancedouble;

	    double y_DistanceStatic = 3.61194;                                       
            double y_Distancedouble = ((y_middle - 180) / y_DistanceStatic * distance_to_QR / 150); // Calculate in cm's where the center of the camera is, according to the center of the QR
            y_Distance = (int) y_Distancedouble;

            cout << "Kamera center er: " << x_Distance << "cm til venstre for QR-koden" << endl;
	    cout << "Kamera center er: " << y_Distance << "cm over QR-koden" << endl;
	    
	   
	} //end big ass for loop
 	QR_info.data.push_back(distance_to_QR);
	QR_info.data.push_back(x_Distance);
 	QR_info.data.push_back(y_Distance);
	//push also the data read from the QR code it self

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



double calculatedistance_to_QR(int pixels) {
    if (pixels < 38) return -1;       // If distance is more than 2,9m
    else if (pixels > 130) return -2;    // If distance is less than 1m.
    else return distance_to_QR[pixels]; // Returns distance in cm.
}


//only shameful code lies beneath


































void init_distance(){
    distance_to_QR[38] = 310;
    distance_to_QR[39] = 298;
    distance_to_QR[40] = 290;
    distance_to_QR[41] = 284;
    distance_to_QR[42] = 279;
    distance_to_QR[43] = 274;
    distance_to_QR[44] = 269;
    distance_to_QR[45] = 264;
    distance_to_QR[46] = 259;
    distance_to_QR[47] = 254;
    distance_to_QR[48] = 250;
    distance_to_QR[49] = 245;
    distance_to_QR[50] = 241;
    distance_to_QR[51] = 236;
    distance_to_QR[52] = 232;
    distance_to_QR[53] = 228;
    distance_to_QR[54] = 224;
    distance_to_QR[55] = 220;
    distance_to_QR[56] = 217;
    distance_to_QR[57] = 213;
    distance_to_QR[58] = 209;
    distance_to_QR[59] = 206;
    distance_to_QR[60] = 203;
    distance_to_QR[61] = 199;
    distance_to_QR[62] = 196;
    distance_to_QR[63] = 193;
    distance_to_QR[64] = 190;
    distance_to_QR[65] = 187;
    distance_to_QR[66] = 184;
    distance_to_QR[67] = 181;
    distance_to_QR[68] = 179;
    distance_to_QR[69] = 176;
    distance_to_QR[70] = 173;
    distance_to_QR[71] = 171;
    distance_to_QR[72] = 168;
    distance_to_QR[73] = 166;
    distance_to_QR[74] = 164;
    distance_to_QR[75] = 161;
    distance_to_QR[76] = 159;
    distance_to_QR[77] = 157;
    distance_to_QR[78] = 155;
    distance_to_QR[79] = 153;
    distance_to_QR[80] = 151;
    distance_to_QR[81] = 149;
    distance_to_QR[82] = 148;
    distance_to_QR[83] = 146;
    distance_to_QR[84] = 144;
    distance_to_QR[85] = 142;
    distance_to_QR[86] = 141;
    distance_to_QR[87] = 139;
    distance_to_QR[88] = 137;
    distance_to_QR[89] = 136;
    distance_to_QR[90] = 136;
    distance_to_QR[91] = 133;
    distance_to_QR[92] = 132;
    distance_to_QR[93] = 130;
    distance_to_QR[94] = 129;
    distance_to_QR[95] = 128;
    distance_to_QR[96] = 126;
    distance_to_QR[97] = 125;
    distance_to_QR[98] = 124;
    distance_to_QR[99] = 123;
    distance_to_QR[100] = 122;
    distance_to_QR[101] = 121;
    distance_to_QR[102] = 121;
    distance_to_QR[103] = 120;
    distance_to_QR[104] = 118;
    distance_to_QR[105] = 118;
    distance_to_QR[106] = 117;
    distance_to_QR[107] = 116;
    distance_to_QR[108] = 116;
    distance_to_QR[109] = 115;
    distance_to_QR[110] = 114;
    distance_to_QR[111] = 113;
    distance_to_QR[112] = 113;
    distance_to_QR[113] = 111;
    distance_to_QR[114] = 111;
    distance_to_QR[115] = 110;
    distance_to_QR[116] = 109;
    distance_to_QR[117] = 108;
    distance_to_QR[118] = 107;
    distance_to_QR[119] = 106;
    distance_to_QR[120] = 906;
    distance_to_QR[121] = 105;
    distance_to_QR[122] = 104;
    distance_to_QR[123] = 104;
    distance_to_QR[124] = 103;
    distance_to_QR[125] = 102;
    distance_to_QR[126] = 101;
    distance_to_QR[127] = 101;
    distance_to_QR[128] = 100;
    distance_to_QR[129] = 99;
    distance_to_QR[130] = 98;
}

