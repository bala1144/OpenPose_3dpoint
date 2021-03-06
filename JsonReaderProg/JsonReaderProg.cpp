// JsonReaderProg.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <fstream>
#include <iostream>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <algorithm>

#include "Eigen.h"
#include "FreeImageHelper.h"
#include "ReadJson.h"
#include "ProcessDepth.h"
#include "Definitons.h"
#include "Visualize_normal_images.h"

namespace fs = std::experimental::filesystem;


//void handle_MINF_pixels(Eigen::Vector4f& b, std::vector<Vector4f>& Q_in_3D, Eigen::Vector4f& interpolated_3d) {
//
//	//convert to array
//	constexpr int SIZE = 4;
//	std::pair<float, int> arr[SIZE] = { { b[0], 0 }, { b[1], 1 }, { b[2], 2 }, { b[3], 3} };
//	//float arr[SIZE] = {b[0],b[1],b[2],b[3]};
//	std::sort(arr, arr + SIZE);
//	Vector4f curr_vec;
//
//	for (auto& v : arr)
//	{
//		curr_vec = Q_in_3D[v.second];
//		if (curr_vec[0] != MINF && curr_vec[2] != MINF && curr_vec[2] != MINF)
//		{
//			interpolated_3d = curr_vec;
//			std::cout << " nearest vector is " << interpolated_3d[0] << " " << interpolated_3d[1] << " " << interpolated_3d[2] << " " << std::endl;
//			break;
//		}
//	}
//}

void main()
{
    std::cout << "Hello World!\n";  

	DepthProcess sensor;

	//synthetic config
	/*sensor.m_3dframes_Dir ="C:\\Users\\bala\\Documents\\myprojects\\Jsonreader\\Dataset\\IdealPose\\3D_Data\\";
	std::string DepthFilePath = "C:\\Users\\bala\\Documents\\myprojects\\Jsonreader\\Dataset\\IdealPose\\syntheticDepthFromMesh\\depth";
	std::string open_pose_DepthFilePath = "not needed";*/
	//fs::create_directories(sensor.m_3dframes_Dir);
	
	//yawar data config
	sensor.m_3dframes_Dir ="C:\\Users\\bala\\Documents\\myprojects\\Jsonreader\\Dataset\\Yawar\\3d_point_cloud\\";
	std::string DepthFilePath = "C:\\Users\\bala\\Documents\\myprojects\\Jsonreader\\Dataset\\Yawar\\depth";
	std::string open_pose_DepthFilePath = "C:\\Users\\bala\\Documents\\myprojects\\Jsonreader\\Dataset\\Yawar\\open_pose_results\\";

	std::vector<float> JointsUV;//joints is a vector of size of 75, which is 25 x 3 = (u,v,confidence)
	std::string file,json_file;
	int no_file = 0;

	for (auto& p : fs::directory_iterator(DepthFilePath))
	{	
		
		//file = p.path().filename().string();
		file = "frame000055.pgm";
		std::cout << file << std::endl;
		std::cout << DepthFilePath << '\n';
		sensor.ProcessFrames(DepthFilePath, file);
		sensor.GenerateXYZ();
		sensor.preprocess_point_cloud();
		sensor.compute_normal_image();
		 
		//ignoring this for the synthetic data generation
		////read the json file and generating the coco joints in 3D
		json_file = open_pose_DepthFilePath + file.substr(0, file.find(".pgm")) + "_keypoints.json";
		std::cout << " json file path " << json_file << std::endl;
		JointsUV.clear();
		ReadJson(json_file, JointsUV);
		std::cout <<" number of points = "  <<JointsUV.size() << std::endl;
		sensor.Bilinear_interpolation_joint(JointsUV);

		sensor.Dump_pointcloud_obj();
		sensor.Dump_normals_obj();
		sensor.Dump_open_pose_joints_obj();

		if (no_file+1 < 1)
			no_file++;
		else
			break;
	}

	/*Vector4f b{Vector4f(0.2, 0.4, 0.25, 0.15)};
	Vector4f interpolated_3d;
	std::vector<Vector4f> Q_in_3D;
	Q_in_3D.push_back(Vector4f(MINF,MINF,MINF,MINF));
	Q_in_3D.push_back(Vector4f(0.1,0.2,0.3,0.6));
	Q_in_3D.push_back(Vector4f(MINF,MINF,MINF,MINF));
	Q_in_3D.push_back(Vector4f(1, 3, 10, 15));

	handle_MINF_pixels(b, Q_in_3D, interpolated_3d);
*/
	std::getchar();

}

