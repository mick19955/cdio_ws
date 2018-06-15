/*

	Original template by:	Parker Conroy
												Algorithmic Robotics Lab @ University of Utah

												This code actuates the AR Drone back and forth.
												It is intended as a simple example for those starting with the AR Drone platform.
												
	Edited by:	Group 2, 62410 CDIO, DTU Diplom
							Mikkel Ambjørn Boholdt-Pedersen
							Emily Skovgaard Rasmussen
							Nicolai André Stæhr Kruhøffer
							Jørn Salonin
							Simon Lundorf
							Robert Sand

*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
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
geometry_msgs::Twist instruct_up_forward;
geometry_msgs::Twist instruct_down_forward;

std_msgs::Empty empty;


	
ros::Publisher pub_land; 	//publish empty here to start landing sequence
ros::Publisher pub_takeoff; 	//publish empty here to start takeoff sequence
ros::Publisher pub_reset;	//publish empty here to reset drone

ros::Publisher pub_instruct;	//publish fly instructions here

enum State_machine {Still, Flying, Landing, Circle, Searching, Position_left, Prepare_for_next};
enum Instructions_state {up, down, left, right, forward, backward, takeoff, land, up_forward, down_forward, hover, spin_right, spin_left};

//prototype functions
void found_circle(const std_msgs::Bool::ConstPtr& circle_detected);
void init_instructions();
void extract_qr_info(const std_msgs::Int32MultiArray::ConstPtr& qr_info);
void do_instruction(Instructions_state instruction, double time);

int circle = 0; //info if circle is found or not
int search_counter = 0;
int circle_counter = 0;


//std::array<int,20> QR; TODO: implement QR info into c++ array
//traditional c array
int QR[3];

int main(int argc, char** argv)
{
	ros::init(argc, argv,"ARDrone_test");
    	ros::NodeHandle node;
    	ros::Rate loop_rate(25);

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
	
	ros::ServiceClient flattrim;
	std_srvs::Empty srvflattrim;

	ros::Subscriber circle_sub = node.subscribe ("/circle_found", 1, found_circle);
	ros::Subscriber qr_info_sub = node.subscribe("/qr_info_array", 100, extract_qr_info);

	start_time =(double)ros::Time::now().toSec();	 //start timer
	ROS_INFO("Starting ARdrone_test loop");

	init_instructions();

	bool found_center = false;
	bool qr_found = false;	
	bool at_height = false;	
	bool go_through = false;
	while (ros::ok()){
		if(State == Still){
			std::cout << "State == " << State << std::endl;
			
			flattrim.call(srvflattrim);
			flattrim.call(srvflattrim);

			sleep(1);
			
			do_instruction(takeoff, 5);

			ros::spinOnce();
			loop_rate.sleep();
		//	State = Searching;
		//	State = Flying;
			State = Prepare_for_next;

		}else if(State == Flying){
	//		do_instruction(hover,2);
			std::cout << "State == " << State << std::endl;

			while(!found_center){
				std::cout << QR[0] << "  <-  k  = 0 ||\t" << QR[1] << "  <-  k  = 1 ||\t" << QR[2] << "  <-  k  = 2 " << std::endl;
				//start centering self to the QR code
				if((160 > QR[0]) && (-10 <= QR[1]) && (QR[1] <= 10) && (25 <= QR[2]) &&(QR[2] <= 45)){ //if position is correct
					ROS_INFO("IN CENTER");
					State = Circle;
					found_center = true;
					sleep(1);				
				}else if(QR[1] >= 8){ //correcting position relative to the y-axis
					do_instruction(right, 0.1);
				}else if(QR[1] <= -8){
					do_instruction(left, 0.1);
				}else if(QR[2] >= 35){ //correcting position relative to the y-axis
					do_instruction(down, 0.1);
				}else if((QR[2] <= 25) && (QR[2] != 0)){
					do_instruction(up, 0.1);
				}else if((QR[0] >= 160) && (QR[0] != 0)){
					do_instruction(forward, 0.2);
				}else if(QR[0] == -2){
					do_instruction(backward, 0.2);
				}else{
					//Instructions = hover;
					//do_instruction(hover, 0.4);
				} //end if

				ros::spinOnce(); //spin to get updated ros values
			
				do_instruction(hover, 0.1);


			loop_rate.sleep(); //Will sleep to maintain loop_rate
			} //!found_center

			loop_rate.sleep(); //Will sleep to maintain loop_rate

		}else if(State == Landing){
			std::cout << "State == " << State << std::endl;
			Instructions = land;
			do_instruction(land, 4); //starting landing sequence
			printf("Drone has landed\n");
			printf("Goodbye\n");	
			ros::spinOnce();
			loop_rate.sleep();		
			exit(0);
		}else if(State == Circle){
			do_instruction(hover,2);
			std::cout << "State == " << State << std::endl;
			ROS_INFO("Going for circle!");
			do_instruction(hover, 1.5);
			
			do_instruction(up_forward, 2.3);
			do_instruction(hover, 1);
			ros::spinOnce(); //spin to get updated ros values
			loop_rate.sleep(); //Will sleep to maintain loop_rate
			search_counter = 0;
		//	State = Searching;
			circle_counter++;
			State = Prepare_for_next;
			bool found_center = false;	
			bool at_height = false;
			loop_rate.sleep(); //Will sleep to maintain loop_rate
		}else if (State == Searching){
		do_instruction(hover,2);
			std::cout << "State == " << State << std::endl;
			int i = 0;
			int k = 0;
			while(!qr_found){//Searching for QR
				for(int j = 0; j < 150; j++){	
					ros::spinOnce(); //spin to get updated ros values
					if(QR[0] != 0){
						i++;
						std::cout << "QR was found, i = " << i << std::endl;
						if(i >= 2){ //reliable qr found (5 times)
							qr_found = true;
							std::cout << "QR was found after " << j << " for-loops" << std::endl;
							break;
						}	//end if i > 5	
					}	//end if QR[0] != 0			
				} //end for

				if(!qr_found) {
					if(search_counter < 2) {
					std::cout << "counter under 2";
					do_instruction(spin_left, 0.7);
					do_instruction(spin_right, 0.1);
					do_instruction(hover, 2);
					search_counter++;
					} else if(search_counter < 6) {
					std::cout << "counter under 6";
					do_instruction(spin_right, 0.7);
					do_instruction(spin_left, 0.1);
					do_instruction(hover,2);
					search_counter++;
					} else {
					std::cout << "else";
					do_instruction(spin_right, 0.7);
					do_instruction(spin_left, 0.1);
					do_instruction(hover, 2);
				}} //end if !qr_found
				
				i = 0; //resetting i

				ros::spinOnce(); //spin to get updated ros values
				loop_rate.sleep(); //Will sleep to maintain loop_rate
			} //end while !qr_found
			
			State = Flying;
		} else if (State == Position_left){
			do_instruction(hover,2);
			while(!qr_found){
			do_instruction(left, 0.5);
			do_instruction(hover, 2);
				if(QR[0] != 0){ //Found QR code
					qr_found = true;
					State = Landing;
				}
			ros::spinOnce(); //spin to get updated ros values
			loop_rate.sleep(); //Will sleep to maintain loop_rate
			}
			//end while !qr_found
		}else if(State == Prepare_for_next){//end state_machine if

			if ( circle_counter == 0) {
			std::cout << "DEEEEEEEEEERP";
			do_instruction(forward, 2.3); //startup sequence
			do_instruction(hover, 1); //startup sequence
			do_instruction(left, 1); //startup sequence
			do_instruction(hover, 1); //startup sequence
			State = Searching;
			} else if (circle_counter ==  1) {
			do_instruction(spin_left, 0.6);
			do_instruction(hover, 1);
			do_instruction(down_forward, 2.3);	
			
			
			State = Searching;
			} else if (circle_counter == 2) {
			do_instruction(spin_left, 0.6);
			do_instruction(hover, 1);
			do_instruction(down_forward, 2.3);	
			
			State = Searching;
			}
		//	do_instruction(down, 1);
		//	do_instruction(hover, 1);
		//	do_instruction(spin_left, 0.5);
		//	do_instruction(hover, 1);
		//	do_instruction(forward, 1.5);

		//	State = Searching;
			ros::spinOnce(); //spin to get updated ros values
			loop_rate.sleep(); //Will sleep to maintain loop_rate
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
		}else if(Instruction == up_forward){
			pub_instruct.publish(instruct_up_forward); 		//Send hover command
		}else if(Instruction == down_forward){
			pub_instruct.publish(instruct_down_forward);
		}else if(Instruction == spin_right){
			pub_instruct.publish(instruct_right_spin); 		//Send spin command
			//std::cout << "right_spin" << std::endl;
		}else if(Instruction == spin_left){
			pub_instruct.publish(instruct_left_spin); 		//Send spin command
			//std::cout << "left_spin" << std::endl;
		}else if(Instruction == hover){
			pub_instruct.publish(instruct_hover); 		//Send hover command
			//ROS_INFO("wrong instruction given, hovering");
		}
		ros::spinOnce();

		//loop_rate.sleep(); not defined in this scope ------ is it even neccesary?
	}

	return;
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
	instruct_forward.linear.x=0.2; 
	instruct_forward.linear.y=0.0;
	instruct_forward.linear.z=0.0;
	instruct_forward.angular.x=0.0; 
	instruct_forward.angular.y=0.0;
	instruct_forward.angular.z=0.0;

	//backward message
	instruct_backward.linear.x=-0.2; 
	instruct_backward.linear.y=0.0;
	instruct_backward.linear.z=0.0;
	instruct_backward.angular.x=0.0; 
	instruct_backward.angular.y=0.0;
	instruct_backward.angular.z=0.0;	

	//right message
	instruct_right.linear.x=0.0; 
	instruct_right.linear.y=-0.2; 
	instruct_right.linear.z=0.0; 
	instruct_right.angular.x=0.0;
	instruct_right.angular.y=0.0; 
	instruct_right.angular.z=0.0; 

	//left message
	instruct_left.linear.x=0.0; 
	instruct_left.linear.y=0.2; 
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
	instruct_right_spin.angular.z=0.3; 

	//left spin
	instruct_left_spin.linear.x=0.0; 
	instruct_left_spin.linear.y=0.0; 
	instruct_left_spin.linear.z=0.0; 
	instruct_left_spin.angular.x=0.0;
	instruct_left_spin.angular.y=0.0;
	instruct_left_spin.angular.z=-0.3;  
	//instruct_left_spin.angular.z=-0.6; 


	//up & forward message
	instruct_up_forward.linear.x=0.2; 
	instruct_up_forward.linear.y=0.0;
	instruct_up_forward.linear.z=0.1;
	instruct_up_forward.angular.x=0.0; 
	instruct_up_forward.angular.y=0.0;
	instruct_up_forward.angular.z=0.0;
	
	
	//down & forward message
	instruct_down_forward.linear.x=0.2; 
	instruct_down_forward.linear.y=0.0;
	instruct_down_forward.linear.z=-0.1;
	instruct_down_forward.angular.x=0.0; 
	instruct_down_forward.angular.y=0.0;
	instruct_down_forward.angular.z=0.0;
}

