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
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
std_msgs::Int32 set_instruction;

enum State_machine {Still, Flying, Landing, Circle, Searching, Position_left, Prepare_for_next};
enum Instructions_state {up, down, left, right, forward, backward, takeoff, land, up_forward, down_forward, hover, spin_right, spin_left};


ros::Publisher pub_send_instruction;

//prototype functions
void found_circle(const std_msgs::Bool::ConstPtr& circle_detected);
void init_instructions();
void extract_qr_info(const std_msgs::Int32MultiArray::ConstPtr& qr_info);
void instruction_status(const std_msgs::Int8::ConstPtr& status);
void do_instruction(Instructions_state instruction, double time);


int circle = 0; //info if circle is found or not
int search_counter = 0;
int circle_counter = 0;
int inst_status = 0;

int QR[3];



int main(int argc, char** argv)
{
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


	pub_send_instruction = node.advertise<std_msgs::Int32>("/instruction_msg", 100); //publish here to reset drone
	
	ros::Subscriber qr_info_sub = node.subscribe("/qr_info_array", 100, extract_qr_info);

	bool found_center = false;
	bool qr_found = false;	
	bool at_height = false;	
	bool go_through = false;
	while (ros::ok()){
		if(State == Still){
			std::cout << "State == Takeoff" << State << std::endl;
			
			sleep(1);
			
			do_instruction(takeoff, 5);
			do_instruction(hover, 1);

			//do_instruction(forward, 2.3);
			ros::spinOnce();
			loop_rate.sleep();
		//	State = Searching;
			//State = Flying;
			State = Prepare_for_next;

		}else if(State == Flying){
	//		do_instruction(hover,2);
			std::cout << "State == Flying" << State << std::endl;

			while(!found_center){
				std::cout << QR[0] << "  <-  k  = 0 ||\t" << QR[1] << "  <-  k  = 1 ||\t" << QR[2] << "  <-  k  = 2 " << std::endl;
				//start centering self to the QR code
				if((185 > QR[0]) && (-10 <= QR[1]) && (QR[1] <= 10) && (25 <= QR[2]) &&(QR[2] <= 45)){ //if position is correct
					ROS_INFO("IN CENTER");
					State = Circle;
					found_center = true;
						
				}else if(QR[1] >= 8){ //correcting position relative to the y-axis
					do_instruction(right, 0.1);
				}else if(QR[1] <= -8){
					do_instruction(left, 0.1);
				}else if(QR[2] >= 35){ //correcting position relative to the y-axis
					do_instruction(down, 0.1);
				}else if((QR[2] <= 25) && (QR[2] != 0)){
					do_instruction(up, 0.1);
				}else if((QR[0] >= 180) && (QR[0] != 0)){
					do_instruction(forward, 0.1);
				}else if(QR[0] == -2){
					do_instruction(backward, 0.1);
				}else{
					do_instruction(hover, 0);
				} //end if

				ros::spinOnce(); //spin to get updated ros values
			
				do_instruction(hover, 0.1);


			loop_rate.sleep(); //Will sleep to maintain loop_rate
			} //!found_center

			loop_rate.sleep(); //Will sleep to maintain loop_rate

		}else if(State == Landing){
			std::cout << "State == Landing" << State << std::endl;
			Instructions = land;
			do_instruction(land, 4); //starting landing sequence
			printf("Drone has landed\n");
			printf("Goodbye\n");	
			ros::spinOnce();
			loop_rate.sleep();		
			exit(0);
		}else if(State == Circle){
			//do_instruction(hover,2);
			std::cout << "State == Circle" << State << std::endl;
			ROS_INFO("Going for circle!");
			do_instruction(hover, 0.5);
			
			do_instruction(up_forward, 2.5);
			do_instruction(hover, 1);
			do_instruction(down, 1.5);
			ros::spinOnce(); //spin to get updated ros values
			loop_rate.sleep(); //Will sleep to maintain loop_rate
		//	State = Searching;
			circle_counter += 1;
			State = Prepare_for_next;
			bool found_center = false;	
			bool at_height = false;
			loop_rate.sleep(); //Will sleep to maintain loop_rate
		}else if (State == Searching){
			//do_instruction(hover, 2);
			std::cout << "State == Seaching" << State << std::endl;
			int i = 0;
			int k = 0;
			search_counter = 0;
			while(!qr_found){//Searching for QR
				do_instruction(hover, 0.5);
				if(QR[0] != 0){
					qr_found = true;
				}	//end if QR[0] != 0			
				do_instruction(hover, 0.5);
				if(!qr_found) {
					if(search_counter < 3) {
					//std::cout << "counter under 2";
					do_instruction(spin_left, 0.7);
					do_instruction(spin_right, 0.07);
					std::cout << "spinning right" << std::endl;
					}else if(search_counter < 6) {
					do_instruction(spin_right, 0.7);
					do_instruction(spin_left, 0.07);
					std::cout << "spinning left" << std::endl;
					}
					search_counter+=1;
					std::cout << "search counter = "<< search_counter << std::endl;
					if(search_counter == 7){
						search_counter = 0;
					}
					do_instruction(hover, 0.5);
				} //end if !qr_found

				i = 0; //resetting i

				ros::spinOnce(); //spin to get updated ros values
				loop_rate.sleep(); //Will sleep to maintain loop_rate
			} //end while !qr_found
			found_center = false;
			State = Flying;
		}else if(State == Prepare_for_next){//end state_machine if
			std::cout << "State == Prep" << State << std::endl;
			if ( circle_counter == 0) {
				//do_instruction(forward, 1.4); //point straight at circle
				do_instruction(forward, 1.3); //point straight at circle
				do_instruction(hover, 1); //startup sequence
				do_instruction(left, 1.1); //point straight at circle
				//do_instruction(left, 1); //startup sequence
				//do_instruction(hover, 1); //startup sequence
				State = Flying;
			} else if (circle_counter ==  1) {
				//do_instruction(spin_left, 0.6);
				do_instruction(hover, 1);
				//do_instruction(down_forward, 2);	

				State = Searching;
			} else if (circle_counter == 2) {
				do_instruction(spin_left, 0.5);
				do_instruction(hover, 1);
				//do_instruction(down_forward, 2);	
				//do_instruction(forward, 0.);	
			
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
	
	int utime = (int)(time * 1000000);

	if(Instruction == up){					
		//pub_send_instruction.publish(up); 
		set_instruction.data = 0;
		//pub_send_instruction.publish(set_instruction); 
		//usleep(utime);
	}else if(Instruction == down){
		set_instruction.data = 1;
		//pub_send_instruction.publish(down); 
		//usleep(utime);
	}else if(Instruction == right){
		set_instruction.data = 3;
		//pub_send_instruction.publish(right); 
		//usleep(utime);
	}else if(Instruction == left){
		set_instruction.data = 2;
		//pub_send_instruction.publish(left); 
		//usleep(utime);
	}else if(Instruction == forward){
		set_instruction.data = 4;
		//pub_send_instruction.publish(forward);
		//usleep(utime);
	}else if(Instruction == backward){
		set_instruction.data = 5;
		//pub_send_instruction.publish(backward);
		//usleep(utime);
	}else if(Instruction == takeoff){
		set_instruction.data = 6;
		//pub_send_instruction.publish(takeoff); 			//launches the drone
		//usleep(utime);
	}else if(Instruction == land){
		set_instruction.data = 7;
		//pub_send_instruction.publish(land); 			//land the drone
		//usleep(utime);
	}else if(Instruction == up_forward){
		set_instruction.data = 8;
		//pub_send_instruction.publish(up_forward); 		//Send hover command
		//usleep(utime);
	}else if(Instruction == down_forward){
		set_instruction.data = 9;
		//pub_send_instruction.publish(down_forward);
		//usleep(utime);
	}else if(Instruction == spin_right){
		set_instruction.data = 11;
		//pub_send_instruction.publish(right_spin); 		//Send spin command
		//usleep(utime);
	}else if(Instruction == spin_left){
		set_instruction.data = 12;
		//pub_send_instruction.publish(left_spin); 		//Send spin command
		//usleep(utime);
	}else if(Instruction == hover){
		set_instruction.data = 10;
		//pub_send_instruction.publish(hover); 		//Send hover command
		//usleep(utime);
	}
	pub_send_instruction.publish(set_instruction); 
	usleep(utime);
	ros::spinOnce();

	return;
}

void instruction_status(const std_msgs::Int8::ConstPtr& status)
{
	//inst_status = &status;
	return;
}

