#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/core/eigen.hpp>
// #include "precomp.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/detail/distortion_model.hpp"
#include "opencv2/calib3d/calib3d_c.h"


std::string type2str(int type)
{
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
	}

	r += "C";
	r += (chans+'0');
	std::cout << "Type: " << r << std::endl;
	return r;
}

class Disp_map
{
private:
public:
	cv::Mat disp_img;
	cv::Ptr<cv::StereoSGBM> sgbm;
	int num_disp;
	Disp_map()
	{
			sgbm = cv::StereoSGBM::create(0,16,3);
	}
	
	Disp_map(cv::Mat _disp_img) : disp_img(_disp_img){}
	
	void init_sgbm(const cv::Mat& left,int block_size = 5, int numberOfDisparities = 64)
	{
		// int block_size = 5;
		num_disp = numberOfDisparities;
		std::cout << "line 39 \n";
		sgbm->setPreFilterCap(63);
		std::cout << "line 41 \n";
		int sgbmWinSize = block_size > 0 ? block_size : 3;
		// int numberOfDisparities = 64;
		sgbm->setBlockSize(sgbmWinSize);

		
		int cn = left.channels();

		cv::Size img_size = left.size();
		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
		
		sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
		sgbm->setMinDisparity(0);
		sgbm->setNumDisparities(numberOfDisparities);
		sgbm->setUniquenessRatio(10);
		sgbm->setSpeckleWindowSize(100);
		sgbm->setSpeckleRange(32);
		sgbm->setDisp12MaxDiff(1);

		int alg = 2;
		if(alg==0)
			sgbm->setMode(cv::StereoSGBM::MODE_HH);
		else if(alg==1)
			sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
		else if(alg==2)
			sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
	}

	void compute_disp(const cv::Mat& left_img,const cv::Mat& right_img)
	{
		std::cout << "line 61 \n";
		this->init_sgbm(left_img);
		std::cout << "line 63 \n";
		sgbm->compute(left_img, right_img, disp_img);
		// cv::namedWindow("disp1");
		// cv::imshow("disp1", disp_img);
		disp_img.convertTo(disp_img, CV_32FC1,1.0/(num_disp*16.));
		// std::cout << "line 76 Disp_img \n" << disp_img << std::endl;
		// cv::namedWindow("disp2");
		// cv::imshow("disp2", disp_img);
		// cv::waitKey();
	}

	void save_disp_img(std::string file_name)
	{
		cv::Mat img_to_save;
		disp_img.convertTo(img_to_save,CV_8U,255);
		cv::imwrite(file_name.c_str(),img_to_save);
	}
};


class Depth_map
{
private:
public:
	cv::Mat depth_img;
	pcl::PointCloud<pcl::PointXYZRGB> pc;
	pcl::PointCloud<pcl::PointXYZRGB> pc2;
	Depth_map(){};
	Depth_map(cv::Mat _depth_img):depth_img(_depth_img){}
	void generate_z_map(const Disp_map &d,const cv::Mat& Q, cv::Mat& Z_mat)
	{
		std::cout << "line 93\n";		
		type2str(Q.type());

		cv::Mat_<float> Qf = Q;
		// std::cout << "line 97 \n" << Qf << std::endl; 
		cv::Mat_<float> disp_f = d.disp_img; 
		// std::cout << "line 99 disp_f\n" << disp_f << std::endl;
		// cv::Mat_<cv::Vec3f> out_pc(disp_f.rows, disp_f.cols);
		cv::Mat_<float> Z(disp_f.rows, disp_f.cols);
		cv::Mat_<float> temp_vec1(4,1);
		cv::Mat_<float> temp_vec2(4,1);
		std::vector<cv::Mat_<float> > vector_t2;
		for(int y = 0; y < disp_f.rows; ++y)
		{
			for(int x = 0; x < disp_f.cols; ++x)
			{
				temp_vec1(0) = x;
				temp_vec1(1) = y;
				temp_vec1(2) = disp_f.at<float>(y,x);
				temp_vec1(3) = 1;
				temp_vec2 = Qf * temp_vec1;
				
				temp_vec2 = temp_vec2/temp_vec2(3);
				vector_t2.push_back(temp_vec2);
				Z.at<float>(y,x) = temp_vec2(2);
				// if(disp_f.at<float>(y,x) != 0.0)
				// {
				// 	Z.at<float>(y,x) = temp_vec2(2);
				// }
				// else
				// {
				// 	Z.at<float>(y,x) = FLT_MAX;
				// }
			}
		}
		Z.convertTo(Z_mat, CV_32FC1);

		// std::cout << "line 116 v2 \n" << vector_t2[0] << std::endl; 
		// std::cout << "line 117 Z_mat\n" << Z_mat << "line 114 end \n";
	}
	void generate_point_cloud(cv::Mat Z_mat, cv::Mat left_img)
	{

		std::cout << "line 153 size of Z_mat \n" << Z_mat.size() << std::endl;
		std::cout << "line 154 size of left_img \n" << left_img.size() << std::endl;
		for(int y = 0; y < left_img.rows; y++)
		{
			for (int x = 0; x < left_img.cols; x++)
			{
				if (Z_mat.at<float>(y,x) != FLT_MAX)
				{
					pcl::PointXYZRGB p;
					p.x = x;
					p.y = y;
					p.z = Z_mat.at<float>(y,x);
					cv::Vec3b bgr(left_img.at<cv::Vec3b>(y, x));
					p.b = bgr[0];
					p.g = bgr[1];
					p.r = bgr[2];
					pc.push_back(p);
				}
			}
		}
		std::cout << "line 133 \n";
	}

	void reproject_to_3d_opencv(const Disp_map& d,cv::Mat Q,const cv::Mat& left)
	{
		cv::Mat three_d_img;
		cv::Mat_<float> disp_f = d.disp_img;
		// std::cout << "line 187 \n" << Q << std::endl;
		cv::Mat_<float> Qf = Q;
		// std::cout << "line 189 \n" << Qf << std::endl;
		cv::reprojectImageTo3D(disp_f, three_d_img,Qf);

		for(int y = 0; y < three_d_img.rows; y++)
		{
			for(int x = 0; x < three_d_img.cols; x++)
			{
				// std::cout << "line 185 \n";
				cv::Vec3f point = three_d_img.at<cv::Vec3f>(y,x);
				// std::cout << "line 187 \n";
				pcl::PointXYZRGB p;
				p.x = point[0];
				p.y = point[1];
				p.z = point[2];
				cv::Vec3b bgr(left.at<cv::Vec3b>(y, x));
				p.b = bgr[0];
				p.g = bgr[1];
				p.r = bgr[2];
				pc2.push_back(p);				
			}
		}
	}

	void write_point_cloud_to_file(std::string file_name)
	{
		pcl::PCDWriter w;
		w.writeBinaryCompressed(file_name.c_str(), pc);
		std::cout << "line 140 \n";
	}
	
	void write_point_cloud2_to_file(std::string file_name)
	{
		pcl::PCDWriter w;
		w.writeBinaryCompressed(file_name.c_str(), pc2);
		std::cout << "line 215 \n";
	}
	void write_pc_to_ply(std::string file_name)
	{
		pcl::PLYWriter w;
		w.write(file_name,pc);
		std::cout << "line 216 \n";
	}

	void transform_point_cloud(const Eigen::Affine3f& aff)
	{
		pcl::transformPointCloud(pc,pc,aff);
	}

	void transform_point_cloud2(const Eigen::Affine3f &aff)
	{
		pcl::transformPointCloud(pc2,pc2,aff);
	}

};
