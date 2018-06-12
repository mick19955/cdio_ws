/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah


This code actuates the AR Drone back and forth.
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <unistd.h> //contains usleep

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

//make predefined instruction messages ----
geometry_msgs::Twist instruct;
geometry_msgs::Twist instruct_hover;
geometry_msgs::Twist instruct_up;
geometry_msgs::Twist instruct_down;
geometry_msgs::Twist instruct_right_spin;
geometry_msgs::Twist instruct_left_spin;
geometry_msgs::Twist instruct_forward;
geometry_msgs::Twist instruct_backward;
geometry_msgs::Twist instruct_right;
geometry_msgs::Twist instruct_left;

std_msgs::Empty empty;


	
ros::Publisher pub_land; 	//publish empty here to start landing sequence
ros::Publisher pub_takeoff; 	//publish empty here to start takeoff sequence
ros::Publisher pub_reset;	//publish empty here to reset drone

ros::Publisher pub_instruct;	//publish fly instructions here

enum State_machine {Still, Flying, Landing};
enum Instructions_state {up, down, left, right, forward, backward, takeoff, land, hover};

//prototype functions
void found_circle(const std_msgs::Bool::ConstPtr& circle_detected);
void init_instructions();
void extract_qr_info(const std_msgs::Int32MultiArray::ConstPtr& qr_info);
void do_instruction(Instructions_state instruction, double time);

int circle = 0; //info if circle is found or not


//std::array<int,20> QR; TODO: implement QR info into c++ array
//traditional c array
int QR[3];

int main(int argc, char** argv)
{
	ROS_INFO("ARdrone Test Back and Forth Starting");
	ros::init(argc, argv,"ARDrone_test");
    	ros::NodeHandle node;
    	ros::Rate loop_rate(50);

	State_machine State = Still;
	Instructions_state Instructions;

	double start_time;
	//command message
	float takeoff_time=5.0;
	float fly_time=7.0;
	float land_time=3.0;
	float kill_time =2.0;	

	
    	pub_instruct = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //publish here to give flight controls
	pub_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); //publish here to start takeoff sequence
	pub_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); //publish here to start landing sequence
	pub_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); //publish here to reset drone
	

	ros::Subscriber circle_sub = node.subscribe ("/circle_found", 1, found_circle);
	ros::Subscriber qr_info_sub = node.subscribe("/qr_info_array", 100, extract_qr_info);

	start_time =(double)ros::Time::now().toSec();	 //start timer
	ROS_INFO("Starting ARdrone_test loop");

	init_instructions();

	bool job_done = false;	

	while (ros::ok()){
		if(State == Still){
			std::cout << "State == " << State << std::endl;
			
			Instructions = takeoff;
			do_instruction(takeoff, 5);

			ros::spinOnce();
			loop_rate.sleep();
			State = Flying;

		}else if(State == Flying){
			std::cout << "State == " << State << std::endl;

			while(!job_done){

				std::cout << QR[0] << "  <-  k  = 0 ||\t" << QR[1] << "  <-  k  = 1 ||\t" << QR[2] << "  <-  k  = 2 " << std::endl;
				

				if(QR[1] >= 20){ //correcting position relative to the y-axis
					Instructions = right;
					do_instruction(right, 0.1);
				}else if(QR[1] <= -20){
					Instructions = left;
					do_instruction(left, 0.1);
				}else if(QR[2] >= 12){ //correcting position relative to the y-axis
					Instructions = down;
					do_instruction(down, 0.1);
					std::cout << "Down" << std::endl;
				}else if(QR[2] <= -12){
					Instructions = up;
					do_instruction(up, 0.1);
					std::cout << "up" << std::endl;
				}else{
					Instructions = hover;
					do_instruction(hover, 0.1);
				}
				

				ros::spinOnce(); //spin to get updated ros values
				loop_rate.sleep(); //Will sleep to maintain loop_rate

	/* PSEUDO
				if QR.info == P.01 //first QR code
					first_qr_flag = true;

				while(first_qr_flag)
					if !correct_pos
						if QR.x_Distance > 20
							pub_instruct.publish(instruct_left); //Send left command
						else if QR.x_Distance < -20
							pub_instruct.publish(instruct_right); //Send roght command
						else if QR.y_Distance > 20
							pub_instruct.publish(instruct_down); //Send down command
						else if QR.y_Distance < -20
							pub_instruct.publish(instruct_up); //Send up command
						else
							//drone is now in correct position - break out
							pub_instruct.publish(instruct_hover); //Send hover command
							correct_pos = true;
						sleep(1);

					pub_instruct.publish(instruct_up); //Send down command
					sleep(1);
					pub_instruct.publish(instruct_forward); //Send down command
					//we're not through the ring?
		*/
					
			} //!job_done

			if(job_done){
				State = Landing;
				ros::spinOnce(); //spin to get updated ros values
				loop_rate.sleep(); //Will sleep to maintain loop_rate
			}

		}else if(State == Landing){
			std::cout << "State == " << State << std::endl;
			Instructions = land;
			do_instruction(land, 4); //starting landing sequence
			printf("Drone has landed\n");
			printf("Goodbye\n");	
			ros::spinOnce();
			loop_rate.sleep();		
			exit(0);
		}
	ros::spinOnce(); //spin to get updated ros values
	loop_rate.sleep(); //Will sleep to maintain loop_rate

	}//ros::ok

}//main




void found_circle(const std_msgs::Bool::ConstPtr& circle_detected)
{
	circle = circle_detected->data; //write published value into circle
	return;
}

void extract_qr_info(const std_msgs::Int32MultiArray::ConstPtr& qr_info)
{
	int i = 0;
	for(std::vector<int>::const_iterator j = qr_info->data.begin(); j != qr_info->data.end(); ++j)
	{
		QR[i] = *j; //saving QR information into QR[]
		i++;
	}

	return;
}


void do_instruction(Instructions_state Instruction, double time){
	
	double current_time = (double)ros::Time::now().toSec(); //get current time
	double instruction_time = time + current_time; //define time used for landing before exiting program
	while ((double)ros::Time::now().toSec() < instruction_time){ //loop while doing instruction
		if(Instruction == up){					
			pub_instruct.publish(instruct_up); 
		}else if(Instruction == down){
			pub_instruct.publish(instruct_down); 
		}else if(Instruction == right){
			pub_instruct.publish(instruct_right); 
		}else if(Instruction == left){
			pub_instruct.publish(instruct_left); 
		}else if(Instruction == forward){
			pub_instruct.publish(instruct_forward);
		}else if(Instruction == backward){
			pub_instruct.publish(instruct_backward);
		}else if(Instruction == takeoff){
			pub_instruct.publish(instruct_hover); 		//Send hover command
			pub_takeoff.publish(empty); 			//launches the drone
		}else if(Instruction == land){
			pub_instruct.publish(instruct_hover); 		//Send hover command
			pub_land.publish(empty); 			//land the drone
		}else{
			pub_instruct.publish(instruct_hover); 		//Send hover command
		}
		ros::spinOnce();
		//loop_rate.sleep(); not defined in this scope ------ is it even neccesary?
	}

}


void init_instructions(){

	//hover message
	instruct_hover.linear.x=0.0; 	//+forward 	-backwards
	instruct_hover.linear.y=0.0;	//+left		-right
	instruct_hover.linear.z=0.0;	//+up		-down    	
	instruct_hover.angular.x=0.0;  	//ignore
	instruct_hover.angular.y=0.0;	//ignore
	instruct_hover.angular.z=0.0;  	//+turn left	-turn right

	//up message
	instruct_up.linear.x=0.0; 
	instruct_up.linear.y=0.0;
	instruct_up.linear.z=0.20;
	instruct_up.angular.x=0.0; 
	instruct_up.angular.y=0.0;
	instruct_up.angular.z=0.0;

	//down message
	instruct_down.linear.x=0.0; 
	instruct_down.linear.y=0.0;
	instruct_down.linear.z=-0.20;
	instruct_down.angular.x=0.0; 
	instruct_down.angular.y=0.0;
	instruct_down.angular.z=0.0;

	//forward message
	instruct_forward.linear.x=0.05; 
	instruct_forward.linear.y=0.0;
	instruct_forward.linear.z=0.0;
	instruct_forward.angular.x=0.0; 
	instruct_forward.angular.y=0.0;
	instruct_forward.angular.z=0.0;

	//backward message
	instruct_backward.linear.x=-0.05; 
	instruct_backward.linear.y=0.0;
	instruct_backward.linear.z=0.0;
	instruct_backward.angular.x=0.0; 
	instruct_backward.angular.y=0.0;
	instruct_backward.angular.z=0.0;	

	//right message
	instruct_right.linear.x=0.0; 
	instruct_right.linear.y=-0.05; 
	instruct_right.linear.z=0.0; 
	instruct_right.angular.x=0.0;
	instruct_right.angular.y=0.0; 
	instruct_right.angular.z=0.0; 

	//left message
	instruct_left.linear.x=0.0; 
	instruct_left.linear.y=0.05; 
	instruct_left.linear.z=0.0; 
	instruct_left.angular.x=0.0;
	instruct_left.angular.y=0.0; 
	instruct_left.angular.z=0.0; 

	//right spin
	instruct_right_spin.linear.x=0.0; 
	instruct_right_spin.linear.y=0.0; 
	instruct_right_spin.linear.z=0.0; 
	instruct_right_spin.angular.x=0.0;
	instruct_right_spin.angular.y=0.0; 
	instruct_right_spin.angular.z=1.0; 

	//left spin
	instruct_left_spin.linear.x=0.0; 
	instruct_left_spin.linear.y=0.0; 
	instruct_left_spin.linear.z=0.0; 
	instruct_left_spin.angular.x=0.0;
	instruct_left_spin.angular.y=0.0; 
	instruct_left_spin.angular.z=-1.0; 
}

