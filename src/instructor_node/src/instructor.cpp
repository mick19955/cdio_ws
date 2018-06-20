/*										
	Written by:	Group 2, 62410 CDIO, DTU Diplom
							Mikkel Ambjørn Boholdt-Pedersen
							Emily Skovgaard Rasmussen
							Nicolai André Stæhr Kruhøffer
							Jørn Salonin
							Simon Lundorf
							Robert Sand
	This code will receive flight instructions from the flight_controller and send it to the ARDrone2 through ardrone_autonomy
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
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"

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

//prototype
void init_instructions();
void receive_instruction(const std_msgs::Int32::ConstPtr& instruction_input); //receiving instruction from controller
void do_instruction();

int temp = 2020;

std_msgs::Empty empty;
	
ros::Publisher pub_land; 	//publish empty here to start landing sequence
ros::Publisher pub_takeoff; 	//publish empty here to start takeoff sequence
ros::Publisher pub_reset;	//publish empty here to reset drone

ros::Publisher pub_instruct;	//publish fly instructions here


enum Instructions_state {up, down, left, right, forward, backward, takeoff, land, up_forward, down_forward, hover, spin_right, spin_left};

Instructions_state Instruction = hover;

int main(int argc, char** argv)
{

	std::cout << "Starting instructor!" << std::endl; 
	ROS_INFO("Starting instructor!");

	ros::init(argc, argv, "instructor");
  	
	ros::NodeHandle n;
	
	//initializing publisher
	//ros::Publisher instruction_done_pub 	= n.advertise<std_msgs::Int8>("/instruction_done", 1);
 	pub_instruct = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //publish here to give flight controls
	pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); //publish here to start takeoff sequence
	pub_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1); //publish here to start landing sequence
	pub_reset = n.advertise<std_msgs::Empty>("/ardrone/reset", 1); //publish here to reset drone
	//subscribing to the controllers instruction command
	ros::Subscriber do_instruction_sub = n.subscribe ("/instruction_msg", 1, receive_instruction);


	ros::Rate loop_rate(50);
	ros::ServiceClient flattrim;
	std_srvs::Empty srvflattrim;


	std::cout << "Flat trimming" << std::endl; 
	ROS_INFO("Flat trimming");
	flattrim.call(srvflattrim);

	sleep(1);



	init_instructions();

	std::cout << "instructor started!" << std::endl; 
	ROS_INFO("instructor started!");

	while(ros::ok())
	{	//keep doing instruction
		do_instruction(); //do the globally defined instruction --- gotten from subscription
		ros::spinOnce();
		loop_rate.sleep();
	} // end of while ros::ok

	printf("\n\nExiting!\n\n");

  	return 0;
}

void receive_instruction(const std_msgs::Int32::ConstPtr& instruction_input) //loading new images gotten from the ARdrone into cv_ptr
{

	temp = instruction_input->data; //loading instruction value into Instruction enum
	Instruction = static_cast<Instructions_state>(temp); //loading instruction value into Instruction enum
	
	//std::cout << "temp = " << temp << std::endl;
	
	return;
}

void do_instruction(){
	if(temp == 0){					
		pub_instruct.publish(instruct_up); 
	}else if(temp == 1){
		pub_instruct.publish(instruct_down); 
	}else if(temp == 3){
		pub_instruct.publish(instruct_right); 
	}else if(temp == 2){
		pub_instruct.publish(instruct_left); 
	}else if(temp == 4){
		pub_instruct.publish(instruct_forward);
	}else if(temp == 5){
		pub_instruct.publish(instruct_backward);
	}else if(temp == 6){
		pub_instruct.publish(instruct_hover); 		//Send hover command
		pub_takeoff.publish(empty); 			//launches the drone
	}else if(temp == 7){
		pub_instruct.publish(instruct_hover); 		//Send hover command
		pub_land.publish(empty); 			//land the drone
	}else if(temp == 8){
		pub_instruct.publish(instruct_up_forward); 		//Send hover command
	}else if(temp == 9){
		pub_instruct.publish(instruct_down_forward);
	}else if(temp == 11){
		pub_instruct.publish(instruct_right_spin); 		//Send spin command
	}else if(temp == 12){
		pub_instruct.publish(instruct_left_spin); 		//Send spin command
	}else if(temp == 10){
		pub_instruct.publish(instruct_hover); 		//Send hover command
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
	instruct_up.linear.z=0.2;
	instruct_up.angular.x=0.0; 
	instruct_up.angular.y=0.0;
	instruct_up.angular.z=0.0;

	//down message
	instruct_down.linear.x=0.0; 
	instruct_down.linear.y=0.0;
	instruct_down.linear.z=-0.2;
	instruct_down.angular.x=0.0; 
	instruct_down.angular.y=0.0;
	instruct_down.angular.z=0.0;

	//forward message
	instruct_forward.linear.x=0.3; 
	instruct_forward.linear.y=0.0;
	instruct_forward.linear.z=0.0;
	instruct_forward.angular.x=0.0; 
	instruct_forward.angular.y=0.0;
	instruct_forward.angular.z=0.0;

	//backward message
	instruct_backward.linear.x=-0.3; 
	instruct_backward.linear.y=0.0;
	instruct_backward.linear.z=0.0;
	instruct_backward.angular.x=0.0; 
	instruct_backward.angular.y=0.0;
	instruct_backward.angular.z=0.0;	

	//right message
	instruct_right.linear.x=0.0; 
	instruct_right.linear.y=-0.3; 
	instruct_right.linear.z=0.0; 
	instruct_right.angular.x=0.0;
	instruct_right.angular.y=0.0; 
	instruct_right.angular.z=0.0; 

	//left message
	instruct_left.linear.x=0.0; 
	instruct_left.linear.y=0.3; 
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


	//up & forward message
	instruct_up_forward.linear.x=0.20; 
	instruct_up_forward.linear.y=0.0;
	instruct_up_forward.linear.z=0.1;
	instruct_up_forward.angular.x=0.0; 
	instruct_up_forward.angular.y=0.0;
	instruct_up_forward.angular.z=0.0;
	
	
	//down & forward message
	instruct_down_forward.linear.x=0.2; 
	instruct_down_forward.linear.y=0.0;
	instruct_down_forward.linear.z=-0.15;
	instruct_down_forward.angular.x=0.0; 
	instruct_down_forward.angular.y=0.0;
	instruct_down_forward.angular.z=0.0;
}
/*
void do_instruction(){
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
	}else if(Instruction == spin_left){
		pub_instruct.publish(instruct_left_spin); 		//Send spin command
	}else if(Instruction == hover){
		pub_instruct.publish(instruct_hover); 		//Send hover command
	}	
	std::cout << "doing = " << Instruction << std::endl;

	return;
}
*/


