#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>



#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <iostream>
#include "elas/elas.h"
#include "elas/image.h"

#include "extra_functions.cpp"
/**
 * Load data for this assignment.
 * @param fname The JSON input filename.
 * @param left_fnames The output left images of the stereo pair.
 * @param right_fnames The output right images of the stereo pair.
 * @param poses The 6D poses of the camera when the images were taken.
 *
 * This will probably throw an exception if there's something wrong
 * with the json file.
 */

void LoadMetadata(const std::string& fname,
                  std::vector<std::string>& left_fnames,
                  std::vector<std::string>& right_fnames,
                  std::vector<Eigen::Affine3d>& poses,
				  std::vector<Eigen::Vector3d>& t_vec,
				  std::vector<Eigen::Quaterniond>& q_vec,
				  std::string img_folder) {
	namespace bpt = boost::property_tree;
	bpt::ptree pt;
	bpt::read_json(fname, pt);
	for (bpt::ptree::iterator itr=pt.begin(); itr != pt.end(); ++itr)
	{
		bpt::ptree::value_type v(*itr);
		bpt::ptree entry(v.second);
		std::string left_fname( entry.get<std::string>("left") );
		std::string right_fname( entry.get<std::string>("right") );

		left_fnames.push_back(img_folder+"/"+left_fname);
		right_fnames.push_back(img_folder+"/"+right_fname);
		Eigen::Vector3d t(entry.get<double>("pose.translation.x"),
						  entry.get<double>("pose.translation.y"),
						  entry.get<double>("pose.translation.z"));
		t_vec.push_back(t);
		Eigen::Quaterniond q(entry.get<double>("pose.rotation.w"),
							 entry.get<double>("pose.rotation.x"),
							 entry.get<double>("pose.rotation.y"),
							 entry.get<double>("pose.rotation.z"));
		q_vec.push_back(q);
		Eigen::Affine3d aff = Eigen::Translation3d(t) * q;
		poses.push_back(aff);
	}
}

/**
 * Load calibration data.
 * Note this is basically the ROS CameraInfo message.
 * See
 * http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 * http://wiki.ros.org/image_pipeline/CameraInfo
 * for reference.
 *
 * Note: you probably don't need all the parameters ;)
 */
void LoadCalibration(const std::string& fname,
                     int &width,
                     int &height,
                     cv::Mat& D,
                     cv::Mat& K,
                     cv::Mat& R,
                     cv::Mat& P) {
	namespace bpt = boost::property_tree;
	bpt::ptree pt;
	bpt::read_json(fname, pt);
	width = pt.get<int>("width");
	height = pt.get<int>("height");
	{
		bpt::ptree &spt(pt.get_child("D"));
		D.create(5, 1, CV_32FC1);
		int i=0;
		for (bpt::ptree::iterator itr=spt.begin(); itr != spt.end(); ++itr, ++i) {
			D.at<float>(i,0) = itr->second.get<float>("");
		}
	}
	{
		bpt::ptree &spt(pt.get_child("K"));
		K.create(3, 3, CV_32FC1);
		int ix=0;
		for (bpt::ptree::iterator itr=spt.begin(); itr != spt.end(); ++itr, ++ix) {
			int i=ix/3, j=ix%3;
			K.at<float>(i,j) = itr->second.get<float>("");
		}
	}
	{
		bpt::ptree &spt(pt.get_child("R"));
		R.create(3, 3, CV_32FC1);
		int ix=0;
		for (bpt::ptree::iterator itr=spt.begin(); itr != spt.end(); ++itr, ++ix) {
			int i=ix/3, j=ix%3;
			R.at<float>(i,j) = itr->second.get<float>("");
		}
	}
	{
		bpt::ptree &spt(pt.get_child("P"));
		P.create(3, 4, CV_32FC1);
		int ix=0;
		for (bpt::ptree::iterator itr=spt.begin(); itr != spt.end(); ++itr, ++ix) {
			int i=ix/4, j=ix%4;
			P.at<float>(i,j) = itr->second.get<float>("");
		}
	}
}

/**
 * this is just a suggestion, you can
 * organize your program anyway you like.
 */
void process (const char* file_1,const char* file_2) {
	using namespace std;

	cout << "Processing: " << file_1 << ", " << file_2 << endl;

	// load images
	image<uchar> *I1,*I2;
	I1 = loadPGM(file_1);
	I2 = loadPGM(file_2);
	// I1 = cv::imread(file_1);
	// I2 = cv::imread(file_2);

	// check for correct size
	if (I1->width()<=0 || I1->height() <=0 || I2->width()<=0 || I2->height() <=0 ||
		I1->width()!=I2->width() || I1->height()!=I2->height()) {
		cout << "ERROR: Images must be of same size, but" << endl;
		cout << "       I1: " << I1->width() <<  " x " << I1->height() << 
			", I2: " << I2->width() <<  " x " << I2->height() << endl;
		delete I1;
		delete I2;
		return;    
	}

	// get image width and height
	int32_t width  = I1->width();
	int32_t height = I1->height();

	// allocate memory for disparity images
	const int32_t dims[3] = {width,height,width}; // bytes per line = width
	float* D1_data = (float*)malloc(width*height*sizeof(float));
	float* D2_data = (float*)malloc(width*height*sizeof(float));

	// process
	Elas::parameters param;
	param.postprocess_only_left = false;
	Elas elas(param);
	elas.process(I1->data,I2->data,D1_data,D2_data,dims);

	// find maximum disparity for scaling output disparity images to [0..255]
	float disp_max = 0;
	for (int32_t i=0; i<width*height; i++) {
		if (D1_data[i]>disp_max) disp_max = D1_data[i];
		if (D2_data[i]>disp_max) disp_max = D2_data[i];
	}

	// copy float to uchar
	image<uchar> *D1 = new image<uchar>(width,height);
	image<uchar> *D2 = new image<uchar>(width,height);
	for (int32_t i=0; i<width*height; i++) {
		D1->data[i] = (uint8_t)max(255.0*D1_data[i]/disp_max,0.0);
		D2->data[i] = (uint8_t)max(255.0*D2_data[i]/disp_max,0.0);
	}

	// save disparity images
	char output_1[1024];
	char output_2[1024];
	strncpy(output_1,file_1,strlen(file_1)-4);
	strncpy(output_2,file_2,strlen(file_2)-4);
	output_1[strlen(file_1)-4] = '\0';
	output_2[strlen(file_2)-4] = '\0';
	strcat(output_1,"_disp.pgm");
	strcat(output_2,"_disp.pgm");
	savePGM(D1,output_1);
	// savePGM(D2,output_2);

	// free memory
	delete I1;
	delete I2;
	delete D1;
	delete D2;
	free(D1_data);
	free(D2_data);
}

void elas_compute_disp(const cv::Mat& left, const cv::Mat& right, cv::Mat& disp, std::string img_folder)
{
	std::vector<int> compression_params;
	std::string f1 = img_folder + "/x1000_left.pgm";
	std::string f2 = img_folder + "/x1000_right.pgm";
	cv::Mat im1, im2;
	cv::cvtColor(left, im1,cv::COLOR_BGR2GRAY);
	cv::cvtColor(right, im2,cv::COLOR_BGR2GRAY);	
    compression_params.push_back(CV_IMWRITE_PXM_BINARY);
    compression_params.push_back(1);
	cv::imwrite(f1.c_str(), im1, compression_params);
	cv::imwrite(f2.c_str(), im2, compression_params);
	
	process(f1.c_str(), f2.c_str());
}


void store_point_cloud_from_mat3d(pcl::PointCloud<pcl::PointXYZRGB> &pc,const cv::Mat img_3d, const cv::Mat orig_img_left)
{
	for (int i=0; i < img_3d.rows; ++i)
	{
		for (int j=0; j < img_3d.cols; ++j)
		{
			pcl::PointXYZRGB p;
			cv::Vec3b ptxyz(img_3d.at<cv::Vec3b>(i,j));
			p.x = ptxyz[0];
			p.y = ptxyz[1];
			p.z = ptxyz[2];
			cv::Vec3b bgr(orig_img_left.at<cv::Vec3b>(i, j));
			p.b = bgr[0];
			p.g = bgr[1];
			p.r = bgr[2];
			pc.push_back( p );
		}
	}
}

int main(int argc, char *argv[]) {

	if (argc < 4) {
		std::cerr << "Usage: " << argv[0] << " JSON_DATA_FILE JSON_LEFT_CALIB_FILE JSON_RIGHT_CALIB_FILE\n";
		return -1;
	}
	for(int i = 0; i < argc; i++)
	{
		std::cout << "Argc" << i << " " << argv[i] << std::endl;
	}

	std:: string img_folder(argv[4]);
	// load metadata
	std::vector<std::string> left_fnames, right_fnames;
	std::vector<Eigen::Affine3d> poses;
	std::vector<Eigen::Vector3d>t_vec;
	std::vector<Eigen::Quaterniond>q_vec;
	LoadMetadata(argv[1], left_fnames, right_fnames, poses,t_vec, q_vec,img_folder);

	std::cout << "line 345\n " << poses[0].matrix() << std::endl;

	// load calibration info.
	// note: you should load right as well
	int left_w, left_h;
	cv::Mat left_D, left_K, left_R, left_P;
	LoadCalibration(argv[2], left_w, left_h, left_D, left_K, left_R, left_P);

	int right_w, right_h;
	cv::Mat right_D, right_K, right_R, right_P;
	LoadCalibration(argv[3], right_w, right_h, right_D, right_K, right_R, right_P);

	// std::cout << "line 349 right_D" << right_D << " right_K " << right_K << " right_R " << right_R << " right_P " << right_P << std::endl;
	// here you should load the images from their filenames

	// NOTE: make sure you run the program from the data/ directory
	// for the paths to work.
	// alternatively feel free to modify the input json file or the image
	// filenames at runtime so the images are found.

	// load one of the images as an example.
	std::cout << "loading " << left_fnames[0] << " ... ";
	std::vector<cv::Mat> left_imgs;
	std::vector<cv::Mat> right_imgs;
	for(int i = 0; i < left_fnames.size(); i++)
	{
		left_imgs.push_back(cv::imread(left_fnames[i]));
		if(left_imgs[i].empty()){std::cout << "No left image at index " << i;return -1;}
		right_imgs.push_back(cv::imread(right_fnames[i]));
		if(right_imgs[i].empty()){std::cout << "No right image at index " << i;return -1;}
	}

	// elas_compute_disp(left_imgs[0],right_imgs[0],disp,img_folder);
	std::cout << "line 389 " << left_imgs.size() << std::endl;
	// cv::Mat equl;		   
	// equalize_illumination(left_imgs[0],equl);
	// std::string out_f1 = img_folder + "/clahe.png";
	// cv::imwrite(out_f1.c_str(),equl);
	// apply_log(left_imgs[0], lap);
	Disp_map d1;
	// d1.use_census_transform(left_imgs[0],right_imgs[0]);
	// d1.post_process_disp_map(left_imgs[0], right_imgs[0]);
	// d1.compute_disp(left_imgs[0],right_imgs[0]);
	// d1.post_process_discard_noise();
	Depth_map final_map;
	for(int i = 0; i < left_imgs.size(); i++)
	{
	    create2_pcd_one_pair(left_imgs[i], right_imgs[i], t_vec[i],q_vec[i], left_K, right_K, left_D, right_D, left_w, left_h, i, img_folder,poses[i], final_map);
	}
	std::string out_f = img_folder + "/aligned_direct/aligned.pcd";
	final_map.write_point_cloud2_to_file(out_f);
	std::cout << "All done" << std::endl;
	return 0;
}
