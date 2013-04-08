/*!
*****************************************************************
* \file
*		helper.h
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
* ROS package name: lcrob1_simulation
*
* \author
* Author: Ali Shuja Siddiqui <joshas@ipa.fraunhofer.de>
* \author
* Supervised by: Joshua Hampp <Joshua.Hampp@ipa.fraunhofer.de>
*
* \date Date of creation: 19 Mar 2013
*
* \brief
* RangeMsg to ExRangeMsg converter node.
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
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "ipa_odroidx_ultrasonic_interface/ExRange.h"
#include "ipa_odroidx_ultrasonic_interface/ExRangeArray.h"
#include <vector>
#include <stdlib.h>

ros::Publisher publisher;
void MakeExRangeArray(const sensor_msgs::Range range_msg)
{
	static std::vector<ipa_odroidx_ultrasonic_interface::ExRange> ExRangeVector;
	static bool in_cycle = false;
	if (in_cycle!=true)
	{
		if (range_msg.header.frame_id == "/sonar_1_link")
			in_cycle = true;
	}
	if(in_cycle==true){
		ipa_odroidx_ultrasonic_interface::ExRange temp_exrange;
		temp_exrange.sender_ch=range_msg.header.frame_id[7]-'0'-1;
		temp_exrange.receiver_ch=temp_exrange.sender_ch;
		temp_exrange.measurement.radiation_type = 0;
		temp_exrange.measurement.field_of_view = range_msg.field_of_view;
		temp_exrange.measurement.min_range = range_msg.min_range;
		temp_exrange.measurement.max_range = range_msg.max_range;
		temp_exrange.measurement.range = range_msg.range;

		temp_exrange.measurement.header.stamp = ros::Time::now();
		temp_exrange.measurement.header.frame_id ="us"+range_msg.header.frame_id.substr(7,1);
		if( range_msg.range != range_msg.max_range)
			ExRangeVector.push_back(temp_exrange);
	}
	if (in_cycle == true && range_msg.header.frame_id == "/sonar_8_link")
	{
		ipa_odroidx_ultrasonic_interface::ExRangeArray ex_range_array;
		for (std::vector<ipa_odroidx_ultrasonic_interface::ExRange>::iterator exrange_iter = ExRangeVector.begin(); exrange_iter != ExRangeVector.end(); exrange_iter++)
		{
			ex_range_array.measurements.push_back(*exrange_iter);
		}
		in_cycle = false;
		publisher.publish(ex_range_array);
		ExRangeVector.clear();
	}
	
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "range_to_exrange");
	ros::NodeHandle nh_;
	publisher = nh_.advertise<ipa_odroidx_ultrasonic_interface::ExRangeArray>("ultrasonic_msgs", 1000);
	ROS_INFO("sensor_msgs/Range to ipa_odroidx_ultrasonic_interface/ExRangeArray conversion started.");
	ros::Subscriber sub = nh_.subscribe("/sonar", 1000, MakeExRangeArray);
	ros::spin();
	return 0;
}
