#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

bool ReadJson(const std::string& inputFileName, std::vector<float>& vect)
{
	//genereal details
	// need to implement the faulty cases, if keys are not part of the string
	//std::string inputFileName = "";

	// Create an input filestream
	std::ifstream ifs(inputFileName, std::ios::in);
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
		std::size_t start = str_pointer.find(start_key) + start_key.length() + 3;
		std::size_t end = str_pointer.find(end_key);
		std::string val_string = str_pointer.substr(start, end - start);
		std::istringstream ss(val_string);
		//std::vector<float> vect;

		//seperate into floats
		while (std::getline(ss, line, ','))
		{
			vect.push_back(std::stof(line));
			//std::cout << line << std::endl;

		}

	}
	return true;
}
