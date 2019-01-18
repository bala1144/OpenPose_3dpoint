#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>
#include "Eigen.h"
#include "FreeImageHelper.h"

class DepthProcess {
public:
	DepthProcess(){
		for (unsigned int i = 0; i < m_depthImageWidth*m_depthImageHeight; ++i)
		{
			m_normals.push_back(Vector3f(MINF,MINF,MINF));
		}
		
	}

	bool ProcessFrames(const std::experimental::filesystem::v1::directory_entry &p)
	{
		/*if (m_currentIdx == 99)	m_currentIdx = 100;
		else m_currentIdx += m_increment;*/

		//m_file_name = "frame_" + std::to_string(m_currentIdx);
		std::string file_path;
		file_path = p.path().string();
		m_file_name = p.path().filename().string();
		std::ifstream ifs(file_path, std::ios::in);
		if (!ifs) {
			std::cout << "*ERROR** Could not read input file " << file_path << "\n";
			return false;
		}
		FreeImageU16F dImage;
		dImage.LoadImageFromFile(file_path);

		for (unsigned int i = 0; i < m_depthImageWidth*m_depthImageHeight; ++i)
		{
			if (dImage.data[i] == 0)
				m_depthFrame[i] = MINF;
			else
			{
				m_depthFrame[i] = dImage.data[i] * 1.0f / 1000.0f;
				//m_depthFrame[i] = 1.0f / (dImage.data[i] * -0.0030711016 + 3.3309495161);;
				//std::cout << m_depthFrame[i] << std::endl;
			}

		}
		return true;
	}

	bool GenerateXYZ()
	{ 
		//takes the depth frame and generates the point cloud
		float xp, yp, zp;
		unsigned int id;
		for (unsigned int y = 0; y < m_depthImageHeight; ++y)
		{
			for (unsigned int x = 0; x < m_depthImageWidth; ++x)
			{
				id = x + y * m_depthImageWidth;
				if (m_depthFrame[id] == MINF)
				{
					m_vertices.push_back(Vector4f(MINF, MINF, MINF, MINF));
				}
				else
				{
					zp = m_depthFrame[id];
					//std::cout << "zp = " << zp << std::endl;
					xp = (x - m_c_x) * zp / m_f_x;
					yp = (y - m_c_y) * zp / m_f_y;
					m_vertices.push_back(Vector4f(xp, yp, zp, 1.0));
					//std::cout << xp << " " << yp << " " << zp << std::endl;
				}
			}
		}

		return true;

	}
	
	/**/
	/*
	This function  takes the 3d point and preprocess it to remove the background, sides and the floor
	*/
	void preprocess_point_cloud()
	{
		//filtering the  background
		for (auto& v : m_vertices)
		{
			if( v[2] < 3.3f /* && v[0] < 0.95f && v[0] > -0.95f && v[1] < 0.92f*/ )
			m_preprocessed_vertices.push_back(v);
		}
			
	}

	/**/
	/*
	This function  takes the 3d point in m_vertices and preprocess it to remove the background, sides and the floor
	*/
	void compute_normal_image()
	{
		
		//float dzdx = (depth.at<float>(x + 1, y) - depth.at<float>(x - 1, y)) / 2.0;
		//float dzdy = (depth.at<float>(x, y + 1) - depth.at<float>(x, y - 1)) / 2.0;

		//Vec3f d(-dzdx, -dzdy, 1.0f);
		//Vec3f n = normalize(d);

		//normals.at<Vec3f>(x, y) = n;
		unsigned int id, R, L, B, T;
		for (unsigned int y = 1; y < m_depthImageHeight - 1; ++y)
		{
			for (unsigned int x = 1; x < m_depthImageWidth - 1; ++x)
			{
				id = x  + y * m_depthImageWidth;
				R = (x+1) + y * m_depthImageWidth;
				L = (x-1) + y * m_depthImageWidth;
				B = x + ( ( y + 1) * m_depthImageWidth);
				T = x + ( (y -1)  *  m_depthImageWidth);

				float dzdx = (m_depthFrame[R] - m_depthFrame[L]) / 2.0;
				float dzdy = (m_depthFrame[B] - m_depthFrame[T]) / 2.0;
				
				Vector3f n = Vector3f(-dzdx, -dzdy, 1.0f);

				m_normals[id] = n.normalized();
			}
		}
	}

	 void Extract3Djoints(float& x,float& y, std::vector<Vector4f>& Q_in_3D)
	{
		//std::vector<Vector4f> Q_in_3D; order-> q11,q12,q21,q22
		unsigned int id;
		id =int( y + x * m_depthImageHeight);//q11
		Q_in_3D.push_back(m_vertices.at(id));
		id =( y+1) + x * m_depthImageHeight;//q12
		Q_in_3D.push_back(m_vertices.at(id));
		id = y + (x+1) * m_depthImageHeight;//q21
		Q_in_3D.push_back(m_vertices.at(id));
		id =( y+1) + (x + 1) * m_depthImageHeight;//q22
		Q_in_3D.push_back(m_vertices.at(id));
		
		
		//for debugging
		for (auto& v : Q_in_3D)
		{
			std::cout << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
		}
		

	}

	//bool Bilinear_interpolation_joint(std::vector<float>& Joint2D)
	//{
	//	std::vector<Vector4f> Q_in_3D;
	//	//2d Point = Joint2D.at(0),Joint2D.at(1)
	//	std::cout << "2D Joints are " << Joint2D.at(0) << " " << Joint2D.at(1) << std::endl;
	//	
	//	float x = Joint2D.at(3);
	//	float y = Joint2D.at(4);
	//	
	//	float q11x = std::floor(Joint2D.at(3));
	//	float q11y = std::floor(Joint2D.at(4));
	//
	//	float q22x = q11x + 1;
	//	float q22y = q11y + 1;
	//	
	//	float q12x = q11x;
	//	float q12y = q11y + 1;
	//	
	//	float q21x = q11x + 1;
	//	float q21y = q11y;

	//	std::cout << "q11 " << q11x << " " << q11y << std::endl;
	//	std::cout << "q12 " << q12x << " " << q12y << std::endl;
	//	std::cout << "q21 " << q21x << " " << q21y << std::endl;
	//	std::cout << "q22 " << q22x << " " << q22y << std::endl;

	//	//extract the corresponding 3d points from the depth 
	//	Extract3Djoints(q11x, q11y, Q_in_3D);

	//	
	//	//using closed form linear solution from https://en.wikipedia.org/wiki/Bilinear_interpolation
	//	Eigen::Vector4f b;
	//	Eigen::Matrix4f A;
	//	A << 1.0,	 q11x,		q11y,		q11x * q11y ,
	//		 1.0,	 q11x,		q11y+1,		q11x * q12y ,
	//		 1.0,	 q11x +1,	q11y,		q12x * q11y ,
	//		 1.0,	 q11x+1,	q11y + 1,	q22x * q22y ;

	//	Eigen::Vector4f X = Vector4f(1.0, x, y, x*y);

	//	b = A.inverse().transpose() * X;

	//	std::cout << " bi  = " << std::endl << b << std::endl;

	//	Eigen::Vector4f interpolated_3d = b[0] * Q_in_3D.at(0) + b[1] * Q_in_3D.at(1) + b[2] * Q_in_3D.at(2) + b[3] * Q_in_3D.at(3);
	//	std::cout << " interpolated 3d  = " << std::endl << interpolated_3d << std::endl;

	//	//Linear interpolation in y direction
	//	

	//	return true;

	//}

	bool Dump_pointcloud_obj()
	{
		++m_currentIdx;
		std::ofstream file( (m_3dframes_Dir + m_file_name.substr(0,-4)+ ".obj") , std::ios::out);
		for (auto& v : m_preprocessed_vertices)
		{
			if (v[0] != MINF) // if you add minf you cant visualize it in mesh lab 
			file << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
		}
		file.close();
		return true;
	}

	bool Dump_normals_obj()
	{
		++m_currentIdx;
		std::ofstream file((m_3dframes_Dir + m_file_name.substr(0, -4) + "_normals.obj"), std::ios::out);
		for (auto& v : m_preprocessed_vertices)
		{
			if (v[0] != MINF) // if you add minf you cant visualize it in mesh lab 
				file << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
		}
		file.close();
		return true;
	}

	bool Dump_off()
	{
		std::ofstream file( (m_3dframes_Dir + m_file_name+ ".off") , std::ios::out);
		for (auto& v : m_vertices)
		{
			file << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
		}
		file.close();
		return true;
	}

	
private:

	int m_currentIdx = 0;
	int m_increment = 1;
	//std::string m_baseDir = "C:\\Users\\bala\\Documents\\myprojects\\BodyFuViconDataset\\szq\\depth\\";
	std::string m_3dframes_Dir = "C:\\Users\\bala\\Documents\\myprojects\\Jsonreader\\Dataset\\Yawar\\3d_point_cloud\\";
	std::string m_file_name;
	unsigned int m_depthImageWidth =  640 ;
	unsigned int m_depthImageHeight = 480 ;
	float* m_depthFrame = new float[m_depthImageWidth*m_depthImageHeight];
	//Eigen::Matrix3f m_depthIntrinsics;
	//Eigen::Matrix4f m_depthExtrinsics;
	float m_f_x = 576.353f;
	float m_f_y = 576.057f;
	float m_c_x = 319.85f;
	float m_c_y = 240.632f;
	std::vector<Vector4f> m_vertices;
	std::vector<Vector4f> m_preprocessed_vertices;
	std::vector<Vector3f> m_normals;
};