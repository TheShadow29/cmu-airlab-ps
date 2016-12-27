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
#include "map_classes.cpp"

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

double compute_fc(const cv::Mat* left_k,const cv::Mat* right_k,const cv::Mat* left_d,const cv::Mat* right_d, float img_w, float img_h)
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

void create_pcd_one_pair(const cv::Mat &left, const cv::Mat &right, const Eigen::Vector3d& t_vec, const Eigen::Quaterniond& q_vec, const cv::Mat& left_K,const cv::Mat& right_K, const cv::Mat left_D, const cv::Mat right_D, float left_w, float left_h, int iter, std::string img_folder,Eigen::Affine3d pose)
{
	Disp_map d;
	d.compute_disp(left,right);


	std::stringstream ss_o;
	ss_o << img_folder << "/disp_imgs/disp0" << iter << ".png";
	d.save_disp_img(ss_o.str());
	
	cv::Mat essential_mat;
	cv::Mat fund_mat;
	compute_essential_matrix(t_vec, q_vec, essential_mat);
	compute_fund_mat(essential_mat, left_K, right_K, fund_mat);

	double fc = compute_fc(&left_K, &right_K, &left_D,&right_D, left_w, left_h);
	cv::Mat Q;
	double tx = t_vec[0];
	compute_Q(left_K.at<float>(0,2), left_K.at<float>(1,2), right_K.at<float>(0,2), fc, tx, Q);

	Depth_map dm;
	cv::Mat Z_mat;
	std::stringstream ss_l;
	std::stringstream ss_r;
	ss_l << img_folder << "/pcd_files/pair0" << iter << "_left.pcd";
	ss_r << img_folder << "/pcd_files/pair0" << iter << "_right.pcd";
	std::string out_file_pcd_l = ss_l.str();
	std::string out_file_pcd_r = ss_r.str();
    
	dm.generate_z_map(d,Q,Z_mat);

	dm.generate_point_cloud(Z_mat, left);
	Eigen::Affine3f pose_new = pose.cast<float>();
	dm.transform_point_cloud(pose_new);
	std::stringstream ss2;
	ss2 << img_folder << "/pcd_files/pair0" << iter <<"_new_left.pcd";
	dm.write_point_cloud_to_file(ss2.str());

	// dm.generate_point_cloud(Z_mat, right);
	// dm.write_point_cloud_to_file(out_file_pcd_r);

	// std::stringstream ss1;
	// ss1 << img_folder << "/pair0" << iter << ".pcd";
	// dm.reproject_to_3d_opencv(d,Q,left);
	// dm.write_point_cloud2_to_file(ss1.str());
	
	std::cout << "saving a pc to " << out_file_pcd_l << std::endl;
	
}


