/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah


This code actuates the AR Drone back and forth.
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

	geometry_msgs::Twist twist;
	geometry_msgs::Twist twist_neg;
	geometry_msgs::Twist twist_hover;
	geometry_msgs::Twist twist_up;
	geometry_msgs::Twist twist_spin;
	std_msgs::Empty empty;
	
enum State_machine {Landing, Flying, Still};

int main(int argc, char** argv)
{
	ROS_INFO("ARdrone Test Back and Forth Starting");
	ros::init(argc, argv,"ARDrone_test");
    	ros::NodeHandle node;
    	ros::Rate loop_rate(50);

	ros::Publisher pub_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_takeoff;
	ros::Publisher pub_reset;
	double start_time;

	//hover message
	twist_hover.linear.x=0.0; 
	twist_hover.linear.y=0.0;
	twist_hover.linear.z=0.0;
	twist_hover.angular.x=0.0; 
	twist_hover.angular.y=0.0;
	twist_hover.angular.z=0.0;  

	//up message not used for now
	twist_up.linear.x=0.0; 
	twist_up.linear.y=0.0;
	twist_up.linear.z=0.30;
	twist_up.angular.x=0.0; 
	twist_up.angular.y=0.0;
	twist_up.angular.z=0.0;

	//command message
	float takeoff_time=5.0;
	float fly_time=7.0;
	float land_time=3.0;
	float kill_time =2.0;	
	State_machine State = Still;

	
	//spin
	twist_spin.linear.x=0.0; //+forward 	-backwards
	twist_spin.linear.y=0.0; //+left	-right
	twist_spin.linear.z=0.0; //+up		-down
	twist_spin.angular.x=0.0; //ignore
	twist_spin.angular.y=0.0; //ignore
	twist_spin.angular.z=1.0; //+turn left	-turn right

	//used for the original commented-out code
	twist_neg.linear.x=-twist.linear.x; 
	twist_neg.linear.y=-twist.linear.y;
	twist_neg.linear.z=-twist.linear.z;
	twist_neg.angular.x=-twist.angular.x; 
	twist_neg.angular.y=-twist.angular.y;
	twist_neg.angular.z=-twist.angular.z;


	
    	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //publish here to give flight controls
	pub_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); //publish here to start takeoff sequence
	pub_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); //publish here to start landing sequence
	pub_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); //publish here to reset drone

	
	start_time =(double)ros::Time::now().toSec();	 //start timer
	ROS_INFO("Starting ARdrone_test loop");

	
	
	while (ros::ok()){
		if(State == Still){
			std::cout << "State == " << State << std::endl;
			pub_takeoff.publish(empty); //launches the drone
			pub_twist.publish(twist_hover); //Send hover command
			while ((double)ros::Time::now().toSec()< start_time + takeoff_time){ //loop while waiting for takeoff to finish
				//------Waiting------
				//std::cout << "Time == " <<(double)ros::Time::now().toSec() << std::endl;
				pub_takeoff.publish(empty); //launches the drone
				pub_twist.publish(twist_hover); //Send hover command
				ros::spinOnce();
				loop_rate.sleep();
			}
			ros::spinOnce();
			loop_rate.sleep();
			State = Flying;

		}else if(State == Flying){
			std::cout << "State == " << State << std::endl;
			pub_twist.publish(twist_spin); //Send spin command
			//do detection here
			/*
			if(subscribe.qrdetection == true){
				//first qr has been found
				while(subscribe.ringdetection != true){
					//ring is not found
					//fly up
					//double up_start_time = (double)ros::Time::now().toSec(); //get current time
					//double up_time = 0.5 + up_start_time; //fly up for half a second
					while(up_time > (double)ros::Time::now().toSec()){
						//pub_twist.publish(twist_up); //Send fly up command
						ros::spinOnce();
						loop_rate.sleep();
					}//while flying up
					//Loop if ring is still not found
					ros::spinOnce();
					loop_rate.sleep();
				}//while not found
				//ring has been found!
				//and is hopoefully in front of us
				//pub_twist.publish(twist); //Fly forward through the ring
				
				//double forwand_start_time = (double)ros::Time::now().toSec(); //get current time
				//double forward_time = 1.0 + forward_start_time; //fly forward for a second
				while(forwand_time > (double)ros::Time::now().toSec()){
					//pub_twist.publish(twist); //Send fly forwand command
					ros::spinOnce();
					loop_rate.sleep();
				}//while flying forward
				
			ros::spinOnce();
			loop_rate.sleep();
			}
			
			*/

			//this is just for testing purpose
			sleep(4);
			//testing ^ ^ ^ ^
			pub_twist.publish(twist_hover); //Send spin command
			sleep(2);
			//job is done, now land
			State = Landing;

		}else if(State == Landing){
			std::cout << "State == " << State << std::endl;
			double landing_start_time = (double)ros::Time::now().toSec(); //get current time
			double landing_time = 2 + landing_start_time; //fly up for half a second
			while(landing_time > (double)ros::Time::now().toSec()){
				pub_land.publish(empty); //lands the drone
				//waiting for drone to have landed
				ros::spinOnce();
				loop_rate.sleep();
			}//while landing
			printf("Drone has landed\n");
			printf("Goodbye\n");	
			ros::spinOnce();
			loop_rate.sleep();		
			exit(0);
		}
/*
		while ((double)ros::Time::now().toSec()< start_time+takeoff_time){ //takeoff
		
			pub_takeoff.publish(empty); //launches the drone
			pub_twist.publish(twist_hover); //drone is flat
			ROS_INFO("Taking off");
			ros::spinOnce();
			loop_rate.sleep();
			}//while takeoff

		while  ((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time){
		
			pub_twist.publish(twist_hover); //drone is flat
			pub_land.publish(empty); //lands the drone
			ROS_INFO("Landing");
			
					
			if ((double)ros::Time::now().toSec()> takeoff_time+start_time+fly_time+land_time+kill_time){
		
				ROS_INFO("Closing Node");
				//pub_reset.publish(empty); //kills the drone		
				exit(0); 	}//kill node
				ros::spinOnce();
				loop_rate.sleep();			
			}//while land

		while ( (double)ros::Time::now().toSec()> start_time+takeoff_time && (double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time){	
		

			if((double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time/2){
				pub_twist.publish(twist);
				ROS_INFO("Flying +ve");

			}//fly according to desired twist
			
			if((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time/2){
				pub_twist.publish(twist_neg);
				ROS_INFO("Flying -ve");

			}//fly according to desired twist
			
			ros::spinOnce();
			loop_rate.sleep();
			}
*/
	ros::spinOnce();
	loop_rate.sleep();

}//ros::ok

}//main

