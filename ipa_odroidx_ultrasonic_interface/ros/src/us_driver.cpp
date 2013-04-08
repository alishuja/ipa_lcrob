/*!
*****************************************************************
* \file
*		us_driver.cpp
* \note
* Copyright (c) 2013 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: ipa_lcrob
* \note
* ROS package name: ipa_odroidx_ultrasonic_interface
*
* \author
* Author: Ali Shuja Siddiqui <joshas@ipa.fraunhofer.de>
* \author
* Supervised by: Joshua Hampp <Joshua.Hampp@ipa.fraunhofer.de>
*
* \date Date of creation: 21 Dec 2012
*
* \brief
* Implementation of us_driver node.
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/
#include <stdio.h>
#include <sstream>
#include <vector>
#include <signal.h>
#include "ros/ros.h"
#include "ros/xmlrpc_manager.h"
#include "ipa_odroidx_ultrasonic_interface/UARTDriver.h"
#include "ipa_odroidx_ultrasonic_interface/ExRange.h"
#include "ipa_odroidx_ultrasonic_interface/ExRangeArray.h"
#include "sensor_msgs/Range.h"

#define PINGING_SENSOR 	-1
#define SENSOR_NOT_USED 255
#define PINGING_AND_LISTENING_SENSOR -2

#define MAX_SENSORS 14
#define MAX_CONFIGURATIONS 16
#define MAX_SENSOR_READINGS 15

#define MAX_RANGE 10 //in meters
#define MIN_RANGE 0.5 // in meters, not sure

#define SPEED_OF_SOUND 343.2 // meters per second taken from http://en.wikipedia.org/wiki/Speed_of_sound
#define TIMER_PRESCALER 8
#define F_CPU 2304000

int TIMEOUT_OCCURED =0;

sig_atomic_t volatile g_request_shutdown = 0; //To implement custom SIGINT handler 

void sigint_received(int sig)
{
	g_request_shutdown = 1;
}

enum ACK_RECEIVED {NO, MAYBE, YES}; // Three states of ACK 

/*generateConfigVector stores the pinging sensors, to which pinging sensor is a listening
 * sensor bound to in a cycle and which sensors are not used at all.
 * In the output, the pinging sensors are represented by PINGING_SENSOR, location of
 * each listening sensor holds the address of its corresponding pinging sensor
 * and the sensors not used at all are represented by SENSOR_NOT_USED.
 */
std::vector <std::vector<int> > generateConfigVector(XmlRpc::XmlRpcValue config_list)
{
	std::vector<std::vector<int> > config;
	XmlRpc::XmlRpcValue current_cycle;
	ROS_ASSERT(config_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int i = 0; i < config_list.size(); i++)
	{
		ROS_ASSERT(config_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
		std::vector<int> cycle_vector; //For holding configuration for each cycle.
		cycle_vector.resize(MAX_SENSORS);
		for (int j=0; j<MAX_SENSORS; j++)
			cycle_vector[j]=SENSOR_NOT_USED;

		//better to keep it dynamic
		for (int j = 0; j<MAX_SENSORS; j++){
			char str_index[3] = {'\0','\0', '\0'}; // for storing int to hex conv. 
			sprintf(str_index,"%d", j); // Unsure if yamlcpp parser reads hexadecimal values
			if(config_list[i].hasMember(str_index))
			{
				cycle_vector[j]=PINGING_SENSOR; //Set -1 for pinging sensor
				current_cycle = (config_list[i]).operator[](str_index);
				ROS_ASSERT(current_cycle.getType() == XmlRpc::XmlRpcValue::TypeArray);
				ROS_ASSERT(current_cycle.size()>0);
				//Assign address of pinging sensor to each listening sensor
				for (int k = 0; k<current_cycle.size(); k++){
					cycle_vector[static_cast<int>(current_cycle[k])]=((static_cast<int>(current_cycle[k])!=j)?j:PINGING_AND_LISTENING_SENSOR);
				}
			}
		}
		config.push_back(cycle_vector);
	}
	return config;
}
float get_time(unsigned int timer_value)
{
	return (((float)(TIMER_PRESCALER)/(float)(F_CPU)) * timer_value);
}
float get_distance(float fall_time, float base_time)
{
	return ((fall_time-base_time) * SPEED_OF_SOUND); // Not to be divided by 2
}

/* Generates the config string once given the configuration vector generated by generateConfigVector
*/
int generateConfigString(std::vector< std::vector<int> >config_vector,unsigned char * config_string)
{
	unsigned char temp_config_string[100];
	int config_string_length=config_vector.size()*2+1; //setting up length of config string

	if(config_vector.size()==0)
		return 0;
	temp_config_string[0]=(unsigned int)(config_vector.size()) & 0xff;
	int count=1;
	for (std::vector< std::vector<int> >::iterator list_it = config_vector.begin(); list_it != config_vector.end(); list_it++)
	{
		int temp_mask=0;
		if((*list_it).size()==0)
			return 0;
		//better to keep it dynamic
		for (int i=0; i<MAX_SENSORS; i++)
		{
			if(((*list_it)[i]==PINGING_SENSOR) || ((*list_it)[i]==PINGING_AND_LISTENING_SENSOR))
			{
				if (i<7)
					temp_mask|=(1<<i); //For first port
				else
					temp_mask|=(1<<(i+1)); // For second port
			}
		}
		temp_config_string[count++] = (temp_mask & 0xff00) >> 8;
		temp_config_string[count++] = (temp_mask & 0xff);
	}
	memcpy(config_string, temp_config_string, config_string_length); // copying contents of temp config onto config string before return.
	return config_string_length;
}

ipa_odroidx_ultrasonic_interface::ExRange setupExRangeMeasurement(int sensor_address)
{
	std::ostringstream ss_sensor;
	ipa_odroidx_ultrasonic_interface::ExRange temp_range;
	temp_range.measurement.radiation_type = sensor_msgs::Range::ULTRASOUND;
	temp_range.measurement.min_range = MIN_RANGE;
	temp_range.measurement.max_range = MAX_RANGE;
	ss_sensor<<"us"<<sensor_address;
	temp_range.measurement.header.frame_id = ss_sensor.str();
	temp_range.measurement.header.stamp = ros::Time::now();
	return temp_range;
}

/*Gernerate ExRangeArray from the result gathered from the sensor
*/
ipa_odroidx_ultrasonic_interface::ExRangeArray generateExRangeArray(std::map<int, std::vector< int > > input_map, std::vector<std::vector<int> > config_vector, int sequence_number )
{
	ipa_odroidx_ultrasonic_interface::ExRangeArray measurement_array;
	//Traverse over all sensors to check if they are used in this config
	//and calculate relative distance from the assigned pinging sensor
	//in case they are.
	for(unsigned int i = 0; i< config_vector[sequence_number].size(); i++)
	{
		if(config_vector[sequence_number][i]!=SENSOR_NOT_USED)
		{
			if(config_vector[sequence_number][i]==PINGING_AND_LISTENING_SENSOR)
			{
				if(input_map.count(i)) //Received any reading 
				{
					if(input_map[i].size()>=2) // Considering the first reading after the ping.
					{
						// Setting up Range msg first.
						ipa_odroidx_ultrasonic_interface::ExRange temp_range = setupExRangeMeasurement(i);
						temp_range.measurement.range = get_distance(get_time(input_map[i][1]), get_time(input_map[i][0]));
						temp_range.sender_ch = i;
						temp_range.receiver_ch = i;
						measurement_array.measurements.push_back(temp_range);
					}
				}
			}
			else //Is a listening sensor
			{
				if(input_map.count(i)) //Received any reading 
				{
					if(input_map.count(config_vector[sequence_number][i])) // To check if corresponding pinging sensor received any readings
					{
						if(input_map[config_vector[sequence_number][i]].size()>0) // To check if corresponding pinging sensor has base value
						{
							unsigned int valid_value_index = 0; // for pointing to a value which is greater than the base value of the pinging sensor so that no negetive value is observed.
							for (; valid_value_index < input_map[i].size(); valid_value_index++)
							{
								if (input_map[i][valid_value_index]>input_map[config_vector[sequence_number][i]][0])
									break;
							}
							if (valid_value_index<input_map[i].size())
							{
								ipa_odroidx_ultrasonic_interface::ExRange temp_range = setupExRangeMeasurement(i);
								temp_range.measurement.range = get_distance(get_time(input_map[i][valid_value_index]), get_time(input_map[config_vector[sequence_number][i]][0]));
								temp_range.sender_ch = config_vector[sequence_number][i];
								temp_range.receiver_ch = i;
								measurement_array.measurements.push_back(temp_range);
							}
						}
					}
				}
			}
		}
	}
	return measurement_array;
}
void sigalrm_timeout(int sig) //signal callback for timeout. 
{
	TIMEOUT_OCCURED = 1;
}
// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
	int num_params = 0;
	if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
		num_params = params.size();
	if (num_params > 1)
	{
		std::string reason = params[1];
		ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
		g_request_shutdown = 1; // Set flag
	}
	result = ros::xmlrpc::responseInt(1, "", 0);
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "us_driver");
	ros::NodeHandle nh_;
	XmlRpc::XmlRpcValue config_list_;
	std::map<int, std::vector< int > > input_map_;
	bool debug_ = false;
	ros::init(argc, argv, "us_driver");
	if(!nh_.hasParam("us_driver/configurations"))
	{
		ROS_ERROR("Sensor configurations not found.");
		return(EXIT_FAILURE);
	}
	if(nh_.hasParam("us_driver/debug")) //Checks for debug parameter in launch file.
		nh_.getParam("us_driver/debug", debug_);
		
	ROS_INFO("Configurations found.");
	ros::Publisher pub = nh_.advertise<ipa_odroidx_ultrasonic_interface::ExRangeArray>("us_reading", 5);

	CommPortDriver * comm_port_ = new UARTDriver("/dev/ttyUSB0");

	nh_.getParam("us_driver/configurations", config_list_);
	ROS_ASSERT(config_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);

	std::vector <std::vector<int> > config_vector_ = generateConfigVector(config_list_);
	if(debug_)
		ROS_INFO("%d", (int)config_vector_.size());
	if(config_vector_.size()>MAX_CONFIGURATIONS)
	{
		ROS_ERROR("Cannot have more than %d configurations.", MAX_CONFIGURATIONS);
		return(EXIT_FAILURE);
	}
	if (debug_)
		for (int i=0; i<MAX_SENSORS; i++){
			ROS_INFO("%d", config_vector_[0][i]);
		}

	unsigned char config_string_[100]; // Must be declared before use.
	int config_string_length_ = 0;

	config_string_length_ = generateConfigString(config_vector_, config_string_); //Generating a configuration string to send to slave. 
	if (debug_)
	{
		ROS_INFO("config_string_length_ = %d", config_string_length_);
		for (int i= 0; i<config_string_length_; i++)
		{
			ROS_INFO("0x%02x", config_string_[i]);
		}
		ROS_INFO(" ");
	}
	ROS_INFO("Writing configuration string on comm. port...");
	ROS_ASSERT(comm_port_->writeBytes(config_string_, config_string_length_)==config_string_length_);
	unsigned char * buffer_ = new unsigned char[100];

	ACK_RECEIVED ack_received_ = NO;
	bool ack_stage_2 = false;
	int sequence_number = -1; //For storing sequence number for a configuration. -1 if current cycle is complete or no ack received. 
	int sensor_count = 0; //Current sensor number. 
	int total_sensor_readings = -1; //Gets from slave. -1 if not set.
	int current_sensor_address = -1;
	int current_sensor_reading = 0;
	int temp_reading = -1;
	int no_ack_count = 0; // Counts the number of times an ACK is not recieved. Restarts node if condition is met.
	struct sigaction sact; //To define the signal for custom note shutdown routine. 
	unsigned int connected_sensors = 0xffff; //To hold input from slave regarding connectivity of sensors. 
	int timeout_count = 0; //Holds count for number of times timeout has occurred while reading from slave. 
	int zero_count = 0; //For holding the number of times 0x00 is received after ACK signal. Once a certain count is received the node is restarted. 
	bool has_print_ack_msg = false;
	sigemptyset(&sact.sa_mask);
	sact.sa_flags = 0;
	sact.sa_handler  = sigalrm_timeout;
	sigaction(SIGALRM, &sact, NULL);
	signal(SIGINT, sigint_received);
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
	while(!g_request_shutdown)
	{
		alarm(10); // Setting up timer for a signal. 
		comm_port_->readBytes(buffer_, 1);
		alarm(0);
		if (TIMEOUT_OCCURED)
		{
			if (timeout_count >=5)
			{
				ROS_ERROR("Timeout occured 5 times consecutively, restarting node.");
				sleep(5);
				raise(SIGINT);
			}
			else
			{
				ROS_WARN("TIMEOUT!!!! Sending configuration string again.");
				ROS_ASSERT(comm_port_->writeBytes(config_string_, config_string_length_)==config_string_length_);
				//Resetting all values
				ack_received_ = NO;
				ack_stage_2 = false;
				sequence_number = -1;
				sensor_count = 0;
				total_sensor_readings = -1;
				current_sensor_address = -1;
				current_sensor_reading = 0;
				temp_reading = -1;
				TIMEOUT_OCCURED = 0;
				no_ack_count = 0;
				connected_sensors = 0xffff;
				timeout_count++;
				continue;
			}
		}
		timeout_count=0;
		if (no_ack_count> ((((MAX_SENSOR_READINGS*2)+1)*MAX_SENSORS)*MAX_CONFIGURATIONS+1)) //If ack is not received till max. possible message length. 
		{
			ROS_ERROR("Cannot send data on comm. port, node shutting down.");
			sleep(5);
			raise(SIGINT);
		}
		if(ack_received_ == NO) //Increment  NO_ACK counter 
		{
			has_print_ack_msg = false;
			if (debug_)ROS_INFO("ACK NO (0x%02x)", buffer_[0]);
			if(buffer_[0] == 0x12)
				ack_received_ = MAYBE;
			else
				no_ack_count++;
		}
		else if(ack_received_ == MAYBE) //0x12 received on comm port in a NO_ACK state.
		{
			if (debug_)
				ROS_INFO("ACK MAYBE");
			if (connected_sensors == 0xffff) //Received byte containing sensor status. (connected or not.)
			{
				connected_sensors = ((buffer_[0] <<8) & 0xff00) | 0xff;
			}
			else if ((connected_sensors & 0x00ff) == 0xff)
			{
				connected_sensors = (connected_sensors & 0xff00) | (buffer_[0] & 0xff);
			}
			else if(ack_stage_2 == false)
			{
				if (buffer_[0] == 0x00) //Checking sequence number, should be 0x00 after receiving ack byte.
				{
					if(debug_)
						ROS_INFO("ack_stage_2 = true");
					ack_stage_2 = true;
					sequence_number = 0;
				}
				else
				{
					ack_received_ = NO;
					connected_sensors = 0xffff;
				}
			}
			else if ((buffer_[0] & 0xf0) == 0xd0)
			{
				if(debug_)
					ROS_INFO("Connected sensor: 0x%04x", connected_sensors);
				for (unsigned int count = 0; count < config_vector_.size(); count++)
				{
					for (int i=0; i<MAX_SENSORS; i++)
					{

						int j=0;
						if (i<7)
							j=(1<<i); //For first port
						else
							j=(1<<(i+1)); // For second port
						if ((connected_sensors & j)==0)
						{
							if (config_vector_[count][i] != SENSOR_NOT_USED)
							{
								ROS_ERROR("Sensor %d is used in the configuration %d but not connected.", i, count);
							}
							else if (count==0)
							{
								ROS_WARN("Sensor %d not connected.", i);
							}
						}
					}
				}
				ack_received_ = YES;
			}
			else // False positive, reset values.
			{
				ack_received_ = NO;
				ack_stage_2 = false;
				connected_sensors = 0xffff;
			}
		}
		if(ack_received_==YES) //Receipt of ack confirmed.
		{
			if (zero_count>100)
			{
				ROS_ERROR("Cannot read data from comm. port properly, restarting node.");
				sleep(5);
				raise(SIGINT);
			}

			if (buffer_[0] == 0)
				zero_count++;
			else 
				zero_count=0;
			//	ROS_INFO("ACK YES");
			//	ROS_INFO("0x%02x", buffer_[0]);
			if (!has_print_ack_msg)
			{
				ROS_INFO("Printing messages to topic: /us_reading");
				has_print_ack_msg = true;
			}
			if(sequence_number == -1) //previous cycle complete.
			{
				sequence_number = buffer_[0];
				if (debug_)
					ROS_INFO("sequence_number: 0x%02x", sequence_number);
			}
			else
			{
				if(sensor_count < MAX_SENSORS) // expecting sensor address and total number of readings 
				{
					if (total_sensor_readings == -1)
					{
						current_sensor_address = (buffer_[0] & 0xf0) >> 4;
						total_sensor_readings = (buffer_[0] & 0x0f) ;
						// ROS_INFO("sensor address: %x, total_sensor_readings: %d",current_sensor_address, total_sensor_readings);
						if(total_sensor_readings == 0)
						{
							sensor_count++;
							total_sensor_readings = -1;
							current_sensor_address = -1;
						}
					}
					else{
						if(current_sensor_reading < total_sensor_readings) //expecting sensor readings. Two byte reading
						{
							if (temp_reading == -1)
								temp_reading= (buffer_[0]<<8 & 0xff00); 
							else
							{
								temp_reading |= (buffer_[0] & 0xff);
								input_map_[current_sensor_address].push_back(temp_reading & 0xffff); //After two bytes of a reading have been read, push them onto a vector.
								temp_reading = -1;
								current_sensor_reading++;
							}
						}
						if(current_sensor_reading >= total_sensor_readings)
						{
							current_sensor_reading = 0;
							total_sensor_readings = -1;
							current_sensor_address = -1;
							sensor_count++;
						}
					}
				}
				if(sensor_count >= MAX_SENSORS) //After complete cycle has been read, start processing. 
				{
					if(debug_) // Print all sensor values if debug is enabled. 
						for (std::map<int, std::vector<int> >::iterator map_it = input_map_.begin(); map_it != input_map_.end(); map_it++)
						{
							ROS_INFO("Sensor address: %d", map_it->first);
							ROS_INFO("----");
							for (std::vector<int>::iterator reading_list_it = (*map_it).second.begin(); reading_list_it != (*map_it).second.end(); reading_list_it++)
							{
								ROS_INFO("0x%04x", (*reading_list_it) & 0xffff);
							}
							ROS_INFO("----");
						}

					ipa_odroidx_ultrasonic_interface::ExRangeArray ex_range_array = generateExRangeArray(input_map_, config_vector_, sequence_number); //Create ExRangeArray message.
					if(debug_) // Print all messages if debug is enabled. 
					{
						ROS_INFO("Printing ExRangeArray");
						ROS_INFO("----------");
						for (unsigned int i = 0; i<ex_range_array.measurements.size(); i++)
						{
							ROS_INFO("Sender: %d, Receiver: %d, Range: %f", ex_range_array.measurements[i].sender_ch, ex_range_array.measurements[i].receiver_ch, ex_range_array.measurements[i].measurement.range);
						}
						ROS_INFO("----------");
					}
					if (ex_range_array.measurements.size()>0) // Publish the messages if more than zero are gathered.
						pub.publish(ex_range_array);
					//After one complete cycle has been processed.
					input_map_.clear();
					sequence_number = -1;
					sensor_count = 0;
				}
			}
		}
		ros::spinOnce();
		//	usleep(1000); // Test value, if enabled, be careful, will cause comm. input code to miss characters in input. 
	}
	delete comm_port_;
	ros::shutdown();
	return 0;
}
