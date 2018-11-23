// JsonReaderProg.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>

#include "Eigen.h"
#include "FreeImageHelper.h"

int m_currentIdx = 99;
int m_increment = 1;
std::string m_baseDir = "C:\\Users\\bala\\Documents\\myprojects\\BodyFuViconDataset\\szq\\depth\\";
int m_depthImageWidth = 640;
int m_depthImageHeight = 480;
float* m_depthFrame = new float[m_depthImageWidth*m_depthImageHeight];

bool ReadJson(const std::string& inputFileName,std::vector<float>& vect)
{
	//genereal details
	// need to implement the faulty cases, if keys are not part of the string

	
	// Create an input filestream
	std::ifstream ifs(inputFileName,std::ios::in);
	if (!ifs) {
		std::cout << "*ERROR** Could not read input file " << inputFileName << "\n";
		return false;
	}
	// Read the file stream into a string stream
	std::string line;
	std::string str_pointer;

	while (std::getline(ifs, str_pointer))
	{		
			
		//seperate the key text
		std::string start_key = "pose_keypoints_2d";
		std::string end_key = "],\"face_keypoints_2d";
		std::size_t start = str_pointer.find(start_key)+ start_key.length() + 3;
		std::size_t end = str_pointer.find(end_key);
		std::string val_string = str_pointer.substr(start, end - start);
		std::istringstream ss(val_string);
		//std::vector<float> vect;

		//seperate into floats
		while ( std::getline(ss, line,',') )
		{
			vect.push_back(std::stof(line));
			//std::cout << line << std::endl;

		}

	}
	return true;
}

bool ProcessNextFrame()
{
	if (m_currentIdx == 99)	m_currentIdx = 100;
	else m_currentIdx += m_increment;

	//if ((unsigned int)m_currentIdx >= (unsigned int)m_filenameColorImages.size()) return false;

	//std::cout << "ProcessNextFrame [" << m_currentIdx << " | " << m_filenameColorImages.size() << "]" << std::endl;
	// depth images are scaled by 5000
	FreeImageU16F dImage;
	dImage.LoadImageFromFile(m_baseDir + "frame_"+ std::to_string(m_currentIdx)+".png" );

	for (unsigned int i = 0; i < m_depthImageWidth*m_depthImageHeight; ++i)
	{
		if (dImage.data[i] == 0)
			m_depthFrame[i] = MINF;
		else
		{
			m_depthFrame[i] = dImage.data[i] * 1.0f / 5000.0f;
			//std::cout << m_depthFrame[i] << std::endl;
		}

	}

	return true;
}

bool ReadDepth(const std::string& inputFileName)
{
	 //reading the depth pixels and converting it into raw depth values


	return true;
}


int main()
{
    std::cout << "Hello World!\n";  
	std::vector<float> JointsUV;//joints is a vector of size of 75, which is 25 x 3 = (u,v,confidence)
	std::string filepath = "C:\\Users\\bala\\Documents\\myprojects\\openpose\\openpose\\out\\frame_82_keypoints.json";
	ReadJson(filepath, JointsUV);
	for (int i = 0; i < JointsUV.size(); i++)
		std::cout << JointsUV.at(i) << std::endl;
	std::cout << JointsUV.size() << std::endl;

	ProcessNextFrame();
	
	std::getchar();

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
