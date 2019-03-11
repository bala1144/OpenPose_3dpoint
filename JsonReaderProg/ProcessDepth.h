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
#include "Visualize_normal_images.h"

class DepthProcess {
public:
	DepthProcess(){
		for (unsigned int i = 0; i < m_depthImageWidth*m_depthImageHeight; ++i)
		{
			m_normals.push_back(Vector3f(MINF,MINF,MINF));
		}
		
	}

	//bool ProcessFrames(const std::experimental::filesystem::v1::directory_entry &p)
	bool ProcessFrames(std::string file_path, const std::string &file_name)
	{
		//file_path = p.path().string();
		m_file_name = file_name;
		file_path = file_path + +"\\" + file_name;
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
			if (v[2] < 3.3f /* && v[0] < 0.95f && v[0] > -0.95f && v[1] < 0.92f*/)
				m_preprocessed_vertices.push_back(v);
			else
				m_preprocessed_vertices.push_back(Vector4f(MINF, MINF, MINF, MINF));
		}
			
	}

	/**/
	/*
	This function  takes the 3d point in m_vertices and preprocess it to remove the background, sides and the floor
	*/
	void compute_normal_image()
	{
		unsigned int id, R, L, B, T;
		for (unsigned int y = 1; y < m_depthImageHeight - 1; ++y)
		{
			for (unsigned int x = 1; x < m_depthImageWidth - 1; ++x)
			{
				id = x + y * m_depthImageWidth;
				R = id + 1;
				L = id - 1;
				B = id + m_depthImageWidth;
				T = id - m_depthImageWidth;

				//float dzdx = (m_depthFrame[R] - m_depthFrame[L]) / 2.0;
				//float dzdy = (m_depthFrame[B] - m_depthFrame[T]) / 2.0;
				
				Vector3f dzdx = (m_vertices[L] - m_vertices[R]).head(3);
				Vector3f dzdy = (m_vertices[B] - m_vertices[T]).head(3);
		
				Vector3f n = dzdx.cross(dzdy);
		

				//Vector3f n = Vector3f(-m_f_x * dzdx, -m_f_x * dzdy, 1.0f);
				//Vector3f n = Vector3f(-dzdy, -dzdx, 1.0f);

				m_normals[id] = n.normalized();
				//m_normals[id] = n;

				//if (m_depthFrame[id] == MINF)
				//{
				//	m_normals[id] = Vector3f(MINF, MINF, MINF);
				//}
				//else
				//{
				//	float dzdx = (m_depthFrame[R] - m_depthFrame[L]) / 2.0;
				//	float dzdy = (m_depthFrame[B] - m_depthFrame[T]) / 2.0;
				//	Vector3f n = Vector3f(-m_f_x * dzdx, -m_f_x * dzdy, 1.0f);
				//	m_normals[id] = n.normalized();
				//}
			}
		}
	}

	 void Extract3Djoints(int& x,int& y, std::vector<Vector4f>& Q_in_3D)
	{
		//std::vector<Vector4f> Q_in_3D; order-> q11,q12,q21,q22
		unsigned int id;
		//id = x + y * m_depthImageWidth;
		id = x + y * m_depthImageWidth;//q11
		Q_in_3D.push_back(m_preprocessed_vertices.at(id));
		id = x + ( y +1) * m_depthImageWidth;//q12
		Q_in_3D.push_back(m_preprocessed_vertices.at(id));
		id = ( x + 1)  + y * m_depthImageWidth;;//q21
		Q_in_3D.push_back(m_preprocessed_vertices.at(id));
		id = ( x +1 ) + ( y + 1 ) * m_depthImageWidth;//q22
		Q_in_3D.push_back(m_preprocessed_vertices.at(id));
		 
		
	}

	 Eigen::Vector3f colour_to_depth_plane(float& x, float& y)
	 {
		 m_depthIntrinsics
			 << m_f_x, 0.0f, m_c_x,
			 0.0f, m_f_y, m_c_y,
			 0.0f, 0.0f, 1.0f;

		 m_colourIntrinsics <<	m_col_f_x, 0.0f, m_col_c_x,
								0.0f, m_col_f_y, m_col_c_y,
								0.0f, 0.0f, 1.0f;

		 Eigen::Vector3f depth_index = m_depthIntrinsics * m_colourIntrinsics.inverse() * Vector3f(x ,y , 1.0f);
		 	 
		 return (depth_index);
	 }

	 void handle_MINF_pixels(Eigen::Vector4f& b, std::vector<Vector4f>& Q_in_3D, Eigen::Vector4f& interpolated_3d) {

		 //convert to array
		 constexpr int SIZE = 4;
		 std::pair<float, int> arr[SIZE] = { { b[0], 0 }, { b[1], 1 }, { b[2], 2 }, { b[3], 3} };
		 std::sort(arr, arr + SIZE, std::greater<>() );
		 Vector4f curr_vec;

		 for (auto& v : arr)
		 {
			 curr_vec = Q_in_3D[v.second];
			 if (curr_vec[0] != MINF && curr_vec[2] != MINF && curr_vec[2] != MINF)
			 {
				 interpolated_3d = curr_vec;
				 std::cout << " nearest vector is " << interpolated_3d[0] << " " << interpolated_3d[1] << " " << interpolated_3d[2] << " " << std::endl;
				 break;
			 }
		 }
	 }

	 bool Bilinear_interpolation_joint(std::vector<float>& Joint2D)
	 {
		 std::vector<Vector4f> Q_in_3D;
		 //2d Point = Joint2D.at(0),Joint2D.at(1)
		 float x, y;
		 int q11x, q11y, q22x, q22y, q12x, q12y, q21x, q21y;
		 Eigen::Vector4f b;
		 Eigen::Matrix4f A;
		 Eigen::Vector4f X, interpolated_3d;
		 Eigen::Vector3f v_idx;
		 m_open_pose_joints.clear();
		 
		 //for (int id = 0; id <  5 ; id++)
		 for (int id = 0; id < Joint2D.size() / 3; id++)
		 {
			 x = Joint2D.at(id * 3);
			 y = Joint2D.at((id * 3) + 1);

			 std::cout << "2D Joints idx in colour image " << x << " " << y << std::endl;
			 
			 if (x == 0 && y == 0){
				 std::cout << "case: Joint not detected in open pose "<< std::endl << std::endl;
				 interpolated_3d = Vector4f(MINF, MINF, MINF, MINF);
				 m_open_pose_joints.push_back(interpolated_3d);
				 continue;
			 }
			 else if (x != 0 && y != 0){
				 v_idx = colour_to_depth_plane(x, y);

				 x = v_idx[0] / v_idx[2];
				 y = v_idx[1] / v_idx[2];

				 std::cout << "2D Joints idx in depth image" << x << " " << y << std::endl;


				 q11x = std::floor(x);
				 q11y = std::floor(y);

				 q22x = q11x + 1;
				 q22y = q11y + 1;

				 q12x = q11x;
				 q12y = q11y + 1;

				 q21x = q11x + 1;
				 q21y = q11y;

				 //std::cout << "q11 " << q11x << " " << q11y << std::endl;
				 //std::cout << "q12 " << q12x << " " << q12y << std::endl;
				 //std::cout << "q21 " << q21x << " " << q21y << std::endl;
				 //std::cout << "q22 " << q22x << " " << q22y << std::endl;

				 //extract the corresponding 3d points from the depth 
				 Q_in_3D.clear();
				 if (Q_in_3D.empty())
					 Extract3Djoints(q11x, q11y, Q_in_3D);
				 else {
					 std::cout << " error : vector is not empty" << std::endl;
					 break;
				 }

				 std::cout << "neighbouring points in 3d   = " << std::endl;
				 for (auto& v : Q_in_3D)
				 {
					 //if (v[0] != MINF && v[1] != MINF && v[2] != MINF)// if you add minf you cant visualize it in mesh lab 
					 std::cout << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
					 //m_open_pose_joints.push_back(v);
				 }



				 //using closed form linear solution from https://en.wikipedia.org/wiki/Bilinear_interpolation
				 A << 1.0, q11x, q11y, q11x * q11y,
					 1.0, q11x, q11y + 1, q11x * q12y,
					 1.0, q11x + 1, q11y, q12x * q11y,
					 1.0, q11x + 1, q11y + 1, q22x * q22y;

				 X = Vector4f(1.0, x, y, x*y);
				 b = A.inverse().transpose() * X;

				 //std::cout << " bi  = " << std::endl << b << std::endl;
				 std::cout << "Interpolation coefficients   = " << b[0] << " " << b[1] << " " << b[2] << " " << b[3] << " " << std::endl;
				 interpolated_3d = b[0] * Q_in_3D.at(0) + b[1] * Q_in_3D.at(1) + b[2] * Q_in_3D.at(2) + b[3] * Q_in_3D.at(3);

				 std::cout << " interpolated 3d  = " << interpolated_3d[0] << " " << interpolated_3d[1] << " " << interpolated_3d[2] << std::endl;


				 if (interpolated_3d[0] != MINF && interpolated_3d[1] != MINF && interpolated_3d[2] != MINF &&
					 isnan(interpolated_3d[0]) == false && isnan(interpolated_3d[1]) == false && isnan(interpolated_3d[2]) == false) {
					 //pushing the interpolated joints
					 m_open_pose_joints.push_back(interpolated_3d);
				 }
				 else {
					 interpolated_3d = Vector4f(MINF, MINF, MINF, MINF);
					 handle_MINF_pixels(b, Q_in_3D, interpolated_3d);
					 m_open_pose_joints.push_back(interpolated_3d);
				 }

				 //m_open_pose_joints.push_back(Q_in_3D.at(0));
				 std::cout << "MINF interpolated 3d  = " << interpolated_3d[0] << " " << interpolated_3d[1] << " " << interpolated_3d[2] << std::endl << std::endl;
			 }
		}

		std::cout << " numkber of interpolated 3d joints = " << m_open_pose_joints.size() << std::endl;
		int i = 0;
		for (auto& v : m_open_pose_joints)
		{
			std::cout << "j idx " << i <<"  v " << v[0] << " " << v[1] << " " << v[2] << "\n";
			++i;
		}

		Dump_open_pose_joints_obj();
		//Linear interpolation in y direction
		return true;

	}

	bool Dump_open_pose_joints_obj()
	{
		++m_currentIdx;
		std::ofstream file((m_3dframes_Dir + m_file_name.substr(0, m_file_name.find(".pgm")) + "_joints_.obj"), std::ios::out);
		for (auto& v : m_open_pose_joints)
		{
			if (v[0] != MINF && v[1] != MINF && v[2] != MINF)// if you add minf you cant visualize it in mesh lab 
				file << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
			else
				file << "v " << 0.0f << " " << 0.0f << " " << 0.0f << "\n";
		}
		file.close();
		return true;
	}

	bool Dump_pointcloud_obj()
	{
		++m_currentIdx;
		std::ofstream file( (m_3dframes_Dir + m_file_name.substr(0, m_file_name.find(".pgm")) + ".obj") , std::ios::out);
		for (auto& v : m_preprocessed_vertices)
		{
			if (v[0] != MINF) // if you add minf you cant visualize it in mesh lab 
			file << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
		}
		file.close();
		return true;
	}

	bool Dump_pointcloud_off()
	{
		++m_currentIdx;
		std::ofstream file((m_3dframes_Dir + m_file_name.substr(0, m_file_name.find(".pgm")) + ".off"), std::ios::out);
		for (auto& v : m_preprocessed_vertices)
		{
			if ( (v[0] != MINF && v[1] != MINF && v[2] != MINF) /*||
				( isnan(v[0]) == true && isnan(v[1]) == true && isnan(v[2]) == true)*/ ) // if you add minf you cant visualize it in mesh lab 
			file << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
			else 
				file << "v " << 0.0f << " " << 0.0f << " " << 0.0f << "\n";
		}
		file.close();
		return true;
	}

	bool Dump_normals_obj()
	{
		++m_currentIdx;
		std::ofstream file((m_3dframes_Dir + m_file_name.substr(0, m_file_name.find(".pgm")) + "_normals.obj"), std::ios::out);
		for (auto& v : m_normals)
		{
			if (v[0] != MINF && v[1] != MINF && v[2] != MINF &&
				isnan(v[0]) == false && isnan(v[1]) == false && isnan(v[2]) == false) // if you add minf you cant visualize it in mesh lab 
				file << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
		}
		file.close();
		return true;
	}
	
	bool Dump_normals_off()
	{
		++m_currentIdx;
		std::ofstream file((m_3dframes_Dir + m_file_name.substr(0, m_file_name.find(".pgm")) + "_normals.off"), std::ios::out);
		for (auto& v : m_normals)
		{
			//if (v[0] != MINF) // if you add minf you cant visualize it in mesh lab 
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
	Eigen::Matrix3f m_depthIntrinsics;
	Eigen::Matrix3f m_colourIntrinsics;
	//Eigen::Matrix4f m_depthExtrinsics;

	//depth intrinsics param
	float m_f_x = 576.353f;
	float m_f_y = 576.057f;
	float m_c_x = 319.85f;
	float m_c_y = 240.632f;

	//color intrinsics param
	float m_col_f_x = 1161.04f;
	float m_col_f_y = 1161.72f;
	float m_col_c_x = 648.21f;
	float m_col_c_y = 485.785f;
	

	std::vector<Vector4f> m_vertices;
	std::vector<Vector4f> m_preprocessed_vertices;
	std::vector<Vector3f> m_normals;
	std::vector<Vector4f> m_open_pose_joints;
};