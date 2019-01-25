#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include "Eigen.h"
using namespace cv;
void toOpenCV(std::vector<Vector3f> &normal_map, unsigned int& m_depthImageWidth,cv::Mat& normals)
{

	unsigned int id = -1;
	unsigned int x, y ,i ;
	i = 0;
	x = y = -1;
	for (auto &v : normal_map)
	{	
		x = i / m_depthImageWidth;
		y = i % m_depthImageWidth;
		std::cout << " x " << x << "  y " << y << std::endl;
		normals.at<cv::Vec3f>(x, y) = cv::Vec3f(v[0],v[1],v[2]);
		++i;

	}

}

void normal_visualize(std::vector<Vector3f>& normal_map, unsigned int& m_depthImageWidth)
{

	unsigned int m_depthImageHeight = normal_map.size() / m_depthImageWidth;
	std::cout << "Image height " << m_depthImageHeight << std::endl;
	//cv::Mat depth = <my_depth_image> of type CV_32FC1
	cv::Mat normals(m_depthImageHeight,m_depthImageWidth, CV_32FC3);
	toOpenCV(normal_map, m_depthImageWidth, normals);
	cv::namedWindow("normals", WINDOW_AUTOSIZE);
	cv::imshow("normals", normals);
}