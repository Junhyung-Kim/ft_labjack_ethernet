#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>
#include "LabJackM.h"
#include "../include/ft_labjack_ethernet/LJM_Utilities.h"
#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char **argv)
{

	// #define NUM_FRAMES 5

	ros::init(argc, argv, "voltage_publisher_node");

	ros::NodeHandle nh;

	ros::Publisher ros_pub = nh.advertise<std_msgs::Float64MultiArray>("/ft_labjack_ether", 10);

	ros::Rate rate(1);

	std_msgs::Float64MultiArray msg;

	int count = 1;

	int err;
	int handle1, handle2;
	int i;
	int errorAddress = INITIAL_ERR_ADDRESS;

	enum
	{
		NUM_FRAMES_CONFIG = 24
	};

	const char *aNamesConfig[NUM_FRAMES_CONFIG] =
		{"AIN0_NEGATIVE_CH", "AIN0_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN2_NEGATIVE_CH", "AIN2_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN4_NEGATIVE_CH", "AIN4_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN6_NEGATIVE_CH", "AIN6_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN8_NEGATIVE_CH", "AIN8_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US",
		 "AIN10_NEGATIVE_CH", "AIN10_RANGE", "STREAM_RESOLUTION_INDEX", "STREAM_SETTLING_US"};
	const double aValuesConfig[NUM_FRAMES_CONFIG] =
		{1, 10, 8, 80,
		 3, 10, 8, 80,
		 5, 10, 8, 80,
		 7, 10, 8, 80,
		 9, 10, 8, 80 ,
		 11, 10, 8, 80};

	// Set up for reading AIN values
	enum
	{
		NUM_FRAMES_AIN = 6
	};
	double aValuesAIN[NUM_FRAMES_AIN] = {0};
	const char *aNamesAIN[NUM_FRAMES_AIN] =
		{"AIN0",
		 "AIN2",
		 "AIN4",
		 "AIN6",
		 "AIN8",
		 "AIN10"};

	double matrix[6][6] = {{1, 0, 0, 0, 0, 0},
						   {0, 1, 0, 0, 0, 0},
						   {0, 0, 1, 0, 0, 0},
						   {0, 0, 0, 1, 0, 0},
						   {0, 0, 0, 0, 1, 0},
						   {0, 0, 0, 0, 0, 1}};

	double newAValuesAIN[NUM_FRAMES_AIN] = {0};

	// Open first found LabJack
	handle1 = OpenOrDie(LJM_dtT7, LJM_ctETHERNET, "470022509");
	handle2 = OpenOrDie(LJM_dtT7, LJM_ctETHERNET, "470022511");

	PrintDeviceInfoFromHandle(handle1);
	PrintDeviceInfoFromHandle(handle2);

	// Setup and call eWriteNames to configure AINs on the LabJack.
	err = LJM_eWriteNames(handle1, NUM_FRAMES_CONFIG, aNamesConfig, aValuesConfig, &errorAddress);
	ErrorCheckWithAddress(err, errorAddress, "LJM_eWriteNames");
	err = LJM_eWriteNames(handle2, NUM_FRAMES_CONFIG, aNamesConfig, aValuesConfig, &errorAddress);
	ErrorCheckWithAddress(err, errorAddress, "LJM_eWriteNames");

	printf("\nSet configuration:\n");
	for (i = 0; i < NUM_FRAMES_CONFIG; i++)
	{
		printf("    %s : %f\n", aNamesConfig[i], aValuesConfig[i]);
	}

	std::ofstream file("./src/voltage/src/output.txt", std::ios_base::out | std::ios_base::trunc);

	while (ros::ok())
	{
		err = LJM_eReadNames(handle1, NUM_FRAMES_AIN, aNamesAIN, aValuesAIN, &errorAddress);
		ErrorCheckWithAddress(err, errorAddress, "LJM_eReadNames");

		for (i = 0; i < NUM_FRAMES_AIN; i++)
		{
			newAValuesAIN[i] = 0;
			for (int j = 0; j < NUM_FRAMES_AIN; j++)
			{
				newAValuesAIN[i] += matrix[i][j] * aValuesAIN[j];
			}
		}

		for (i = 0; i < NUM_FRAMES_AIN; i++)
		{
			msg.data.push_back(newAValuesAIN[i]);
		}

		err = LJM_eReadNames(handle2, NUM_FRAMES_AIN, aNamesAIN, aValuesAIN, &errorAddress);
		ErrorCheckWithAddress(err, errorAddress, "LJM_eReadNames");

		for (i = 0; i < NUM_FRAMES_AIN; i++)
		{
			newAValuesAIN[i] = 0;
			for (int j = 0; j < NUM_FRAMES_AIN; j++)
			{
				newAValuesAIN[i] += matrix[i][j] * aValuesAIN[j];
			}
		}

		for (i = 0; i < NUM_FRAMES_AIN; i++)
		{
			msg.data.push_back(newAValuesAIN[i]);
		}

		ros_pub.publish(msg);
		rate.sleep();
		msg.data.erase(msg.data.begin(), msg.data.end());

		/*	if(file.is_open()){
				file<<"#"+std::to_string(count);
				for (i = 0; i<NUM_FRAMES_AIN; i++) {
					file<<", "+std::to_string(newAValuesAIN[i]);
				}
				file<<std::endl;
				count++;
			} else{
				std::cout << "error" << std::endl;
			}
		}*/

		//file.close();

		CloseOrDie(handle1);
		CloseOrDie(handle2);

		WaitForUserIfWindows();

		return 0;
	}
}
