#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

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


void store_point_cloud_from_mat(pcl::PointCloud<pcl::PointXYZRGB> &pc,const cv::Mat img)
{
	for (int i=0; i < img.rows; ++i) {
		for (int j=0; j < img.cols; ++j) {
			pcl::PointXYZRGB p;
			p.x = j;
			p.y = img.rows - i;
			p.z = 1;
			cv::Vec3b bgr(img.at<cv::Vec3b>(i, j));
			p.b = bgr[0];
			p.g = bgr[1];
			p.r = bgr[2];
			pc.push_back( p );
		}
	}
}
void store_point_cloud_from_disp_mat(pcl::PointCloud<pcl::PointXYZI> &pc,const cv::Mat img)
{
	for (int i=0; i < img.rows; ++i) {
		for (int j=0; j < img.cols; ++j) {
			pcl::PointXYZI p;
			p.x = j;
			p.y = img.rows - i;
			p.z = 1;
			float i1 = img.at<float>(i,j);
			p.intensity = i1;
			pc.push_back( p );
		}
	}
}
std::string type2str(int type) {
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
void create_skew_symm_mat(Eigen::Vector3d t, Eigen::Matrix3d &m)
{
	// Eigen::Matrix3f m(3,3);
	m << 0, -t[2], t[1],
		t[2], 0, -t[0],
		-t[1], t[0],0;
}

void compute_essential_matrix(Eigen::Vector3d t, Eigen::Quaterniond q, cv::Mat &essential_mat)
{
	Eigen::Matrix3d S_mat;
	create_skew_symm_mat(t, S_mat);
	Eigen::Matrix3d es;
	Eigen::Matrix3d R_mat;
	R_mat = q.matrix();
	// std::cout << "line 44 " << q.matrix() << std::endl;
	// std::cout << "line 45 " << S_mat << std::endl;
	es = R_mat * S_mat;
	cv::eigen2cv(es, essential_mat);
	std::cout << "line 51 "<< essential_mat << std::endl; 
}

void compute_fund_mat(cv::Mat &essential_mat,const cv::Mat& left_K,const cv::Mat& right_K, cv::Mat &fund_mat)
{
	cv::Mat inv_left;
	cv::invert(left_K, inv_left);
	cv::Mat inv_right;
	cv::invert(right_K, inv_right);
	std::cout << "line 60 "<< inv_left << std::endl;
	inv_left.convertTo(inv_left,CV_32FC1);
	inv_right.convertTo(inv_right,CV_32FC1);
	essential_mat.convertTo(essential_mat,CV_32FC1);
	fund_mat = inv_right * essential_mat * inv_left;
	std::cout << "line 65 " << fund_mat << std::endl;
}

// void sterero_rectification(cv::Mat left_k, cv::Mat right_k, cv::Mat left_d, cv::Mat right_d, Size img_size,
// 						   cv::Mat rot_mat, cv::Mat tr_mat, cv::Mat R1, cv::Mat R2, cv::Mat P1, cv::Mat P2, )
// {
	
// }

double compute_fc(const cv::Mat* left_k,const cv::Mat* right_k,const cv::Mat* left_d,const cv::Mat* right_d, double img_w, double img_h)
{
	double fc_new = DBL_MAX;
	for (int k = 0; k < 2; k++)
	{
		const cv::Mat* A = k==0 ? left_k : right_k;
		const cv::Mat* dk = k==0 ? left_d : right_d;
		double dk1 = dk->at<float>(0,0);
		std::cout << "line 82 " << dk1;
		// std::cout << "line 87 " << left_k->at<float>(0,0);
		double fc = A->at<float>(0,0);
		if (dk1 < 0)
		{
			fc *= 1 + dk1*(img_w * img_w + img_h * img_h)/(4 * fc * fc);
		}
		fc_new = MIN(fc_new, fc);
	}
	return fc_new;
}

void compute_Q(float cx, float cy, float cx_r, float fc, float tx, cv::Mat& Q)
{
	float q[] =
	{
		1,0,0,-cx,
		0,1,0,-cy,
		0,0,0,fc,
		0,0,-1/tx,(cx - cx_r)/tx
	};
	cv::Mat Q1;
	Q1 = cv::Mat(4,4,CV_32FC1,q);
	Q1.convertTo(Q, CV_32FC1);
	std::cout << "line 108 Q \n" << Q << std::endl;
}
