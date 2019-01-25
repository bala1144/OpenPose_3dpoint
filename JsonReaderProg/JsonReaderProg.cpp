// JsonReaderProg.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <fstream>
#include <iostream>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>


#include "Eigen.h"
#include "FreeImageHelper.h"
#include "ReadJson.h"
#include "ProcessDepth.h"
#include "Definitons.h"
#include "Visualize_normal_images.h"

namespace fs = std::experimental::filesystem;


//void main()
//{
//	std::cout << "Learn to write a json config file " << std::endl;
//
//}

void main()
{
    std::cout << "Hello World!\n";  

	DepthProcess sensor;
	std::vector<float> JointsUV;//joints is a vector of size of 75, which is 25 x 3 = (u,v,confidence)
	std::string filepath = "C:\\Users\\bala\\Documents\\myprojects\\Jsonreader\\Dataset\\Yawar\\depth";
	std::string open_pose_filepath = "C:\\Users\\bala\\Documents\\myprojects\\Jsonreader\\Dataset\\Yawar\\open_pose_results\\";
	//std::string filepath = "C:\\Users\\bala\\Documents\\myprojects\\Jsonreader\\Dataset\\Yawar\\depth\\frame000197.pgm";
	fs::create_directories(filepath);
	std::string file,json_file;
	
	for (auto& p : fs::directory_iterator(filepath))
	{	
		std::cout << p.path().filename() << std::endl;
		file = p.path().filename().string();
		std::cout << p.path() << '\n';
		sensor.ProcessFrames(p);
		sensor.GenerateXYZ();
		//sensor.Bilinear_interpolation_joint(JointsUV);
		sensor.preprocess_point_cloud();
		sensor.compute_normal_image();
		 
		//read the json file
		json_file = open_pose_filepath + file.substr(0, file.find(".pgm")) + "_keypoints.json";
		std::cout << " json file path " << json_file << std::endl;
		ReadJson(json_file, JointsUV);
		//for (int i = 0; i < JointsUV.size(); i++)
			//std::cout << JointsUV.at(i) << std::endl;
		std::cout <<" number of points = "  <<JointsUV.size() << std::endl;
		
		sensor.Bilinear_interpolation_joint(JointsUV);

		sensor.Dump_pointcloud_off();
		//sensor.Dump_pointcloud_obj();
		sensor.Dump_normals_off();
		//sensor.Dump_normals_obj();
		break;
	}

	//for (int i = start_frame; i < start_frame+1; i++) {

	//	//filepath = "C:\\Users\\bala\\Documents\\myprojects\\openpose\\openpose\\out\\frame_84_keypoints.json";
	//	 filepath = "C:\\Users\\bala\\Documents\\myprojects\\BodyFuViconDataset\\szq\\joint_2d\\frame_"+ std::to_string(i) +"_keypoints.json";
	//	ReadJson(filepath, JointsUV);
	//	//for (int i = 0; i < JointsUV.size(); i++)
	//		//std::cout << JointsUV.at(i) << std::endl;
	//	std::cout <<" number of points = "  <<JointsUV.size() << std::endl;

	;
	//sensor.Bilinear_interpolation_joint(JointsUV);
				
	//for (int i = 0; i < 1; i++)
	//{
	//	sensor.ProcessFrames(filepath);
	//	sensor.GenerateXYZ();
	//	//sensor.Bilinear_interpolation_joint(JointsUV);
	//	sensor.preprocess_point_cloud();
	//	sensor.Dump_obj();
	//}
		

	//}


	std::getchar();

}

