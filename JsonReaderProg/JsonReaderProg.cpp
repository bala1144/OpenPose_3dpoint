// JsonReaderProg.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>

#include "Eigen.h"
#include "FreeImageHelper.h"
#include "ReadJson.h"
#include "ProcessDepth.h"
#include "Definitons.h"

int main()
{
    std::cout << "Hello World!\n";  
	std::vector<float> JointsUV;//joints is a vector of size of 75, which is 25 x 3 = (u,v,confidence)
	std::string filepath;
	unsigned int start_frame, end_frame;
	start_frame = 100;
	end_frame = 609 + 1;


	for (int i = start_frame; i < start_frame+1; i++) {

		//filepath = "C:\\Users\\bala\\Documents\\myprojects\\openpose\\openpose\\out\\frame_84_keypoints.json";
		 filepath = "C:\\Users\\bala\\Documents\\myprojects\\BodyFuViconDataset\\szq\\joint_2d\\frame_"+ std::to_string(i) +"_keypoints.json";
		ReadJson(filepath, JointsUV);
		//for (int i = 0; i < JointsUV.size(); i++)
			//std::cout << JointsUV.at(i) << std::endl;
		std::cout <<" number of points = "  <<JointsUV.size() << std::endl;

		DepthProcess sensor;
		//sensor.Bilinear_interpolation_joint(JointsUV);
		
		//DepthProcess sensor;
		for (int i = 0; i < 5; i++)
		{
			sensor.ProcessFrames();
			sensor.GenerateXYZ();
			//sensor.Bilinear_interpolation_joint(JointsUV);
			sensor.Dump_obj();
		}
		

	}


	std::getchar();

}

