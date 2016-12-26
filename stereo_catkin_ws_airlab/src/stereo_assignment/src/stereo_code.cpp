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
int P1_slide_val = 600;

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
void ComputeDisparity(const cv::Mat& left, const cv::Mat& right, cv::Mat& disp, cv::Ptr<cv::StereoSGBM> &sgbm)
{
//	disp = left;
	/*
	  minDisparity	Minimum possible disparity value. Normally, it is zero but sometimes 
	  rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.

	  numDisparities	Maximum disparity minus minimum disparity. The value is always greater than zero.
	  In the current implementation, this parameter must be divisible by 16. 

	  blockSize	Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
	  
	  P1	The first parameter controlling the disparity smoothness. See below.
	  
	  P2	The second parameter controlling the disparity smoothness. The larger the values are, the 
	  smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between 
	  neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels.
	  The algorithm requires P2 > P1 . See stereo_match.cpp sample where some reasonably good P1 and P2 values
	  are shown (like 8*number_of_image_channels*SADWindowSize*SADWindowSize and 
	  32*number_of_image_channels*SADWindowSize*SADWindowSize , respectively).
	  
	  disp12MaxDiff	Maximum allowed difference (in integer pixel units) in the left-right disparity check. 
	  Set it to a non-positive value to disable the check.
	  
	  preFilterCap	Truncation value for the prefiltered image pixels. The algorithm first computes 
	  x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values 
	  are passed to the Birchfield-Tomasi pixel cost function.
	  
	  uniquenessRatio	Margin in percentage by which the best (minimum) computed cost function value should "win" 
	  the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
	  
	  speckleWindowSize	Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set 
	  it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
	  
	  speckleRange	Maximum disparity variation within each connected component. If you do speckle filtering, 
	  set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.
	  
	  mode	Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm. 
	  It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size 
	  pictures. By default, it is set to false .

	*/
	
	// cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity,numberOfDisparities,blockSize);
	int block_size = 5;
	
	sgbm->setPreFilterCap(63);
	int sgbmWinSize = block_size > 0 ? block_size : 3;
	int numberOfDisparities = 64;
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

	cv::Mat disp1;
	cv::Mat disp2;
	sgbm -> compute(left,right,disp1);
	disp1.convertTo(disp2,CV_8U,255/(numberOfDisparities*16.));

	disp1.convertTo(disp, CV_32FC1,255/(numberOfDisparities*256*16.));
	if(false)
	{
		// cv::namedWindow("left", 1);
		// cv::imshow("left", left);
		// cv::namedWindow("right", 1);
		// cv::imshow("right", right);
		cv::namedWindow("disp1",0);
		cv::imshow("disp1",disp2);
		cv::namedWindow("disparity", 0);
		cv::imshow("disparity", disp);
		cv::waitKey(0);
	}

}
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
	// std::vector<cv::Mat> disp_mat_vec;
	// cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);
	// elas_compute_disp(left_imgs[0],right_imgs[0],disp,img_folder);
	// std::vector<Disp_map> disp_mat_vec;
	std::cout << "line 389 " << left_imgs.size() << std::endl;
	for(int i = 0; i < left_imgs.size(); i++)
	{
	    create_pcd_one_pair(left_imgs[i], right_imgs[i], t_vec[i],q_vec[i], left_K, right_K, left_D, right_D, left_w, left_h, i, img_folder);
	}
	// Disp_map d;
	// std::cout << "line 389 \n";
	// d.compute_disp(left_imgs[0],right_imgs[0]);
	// disp_mat_vec.push_back(d.disp_img);

	// for(int i=0; i < left_imgs.size(); i++)
	// {
	// 	cv::Mat disp_temp;
	// 	ComputeDisparity(left_imgs[i], right_imgs[i], disp_temp,sgbm);
	// 	disp_mat_vec.push_back(disp_temp);
	// 	// cv::namedWindow("disparity", 0);
	// 	// cv::imshow("disparity", disp_mat_vec[i]);
	// 	// cv::waitKey(0);
	// }

	// std::cout << "line 400 \n";
	// cv::namedWindow("disp",0);
	// cv::imshow("disp",d.disp_img);
	// cv::waitKey(0);
	// cv::imshow("Disp",disp);
	// cv::waitKey(0);

	// int typ1 = left_imgs[0].type();
	// std::string s1 = type2str(typ1);
// etc.

	// finally compute the output point cloud from one or more stereo pairs.
	//
	// This is just a silly example of creating a colorized XYZ RGB point cloud.
	// open it with pcl_viewer. then press 'r' and '5' to see the rgb.

	// disp2 = left_imgs[0];

	// pcl::PointCloud<pcl::PointXYZRGB> pc;
	// pcl::PointCloud<pcl::PointXYZI>pc;
	
	// cv::Mat three_d_img;
	
	// cv::reprojectImageTo3D(disp_mat_vec[0],three_d_img,Q,true);

/****************
	cv::Mat essential_mat;
	cv::Mat fund_mat;
	compute_essential_matrix(t_vec[0], q_vec[0], essential_mat);
	compute_fund_mat(essential_mat, left_K, right_K, fund_mat);

	
	double fc = compute_fc(&left_K, &right_K, &left_D, &right_D, left_w, left_h);
	std::cout << "line 422 " << fc << std::endl;
	// cv::Mat_<float> Q;
	cv::Mat Q;
	double tx = t_vec[0][0];
	compute_Q(left_K.at<float>(0,2), left_K.at<float>(1,2), right_K.at<float>(0,2), fc, tx, Q);
	std::cout << "line 435 Q\n" << Q << std::endl;
	Depth_map dm;
	cv::Mat Z_mat;
	std::string out_file_pcd = img_folder + "/out.pcd";
	dm.generate_z_map(d,Q,Z_mat);
	dm.generate_point_cloud(Z_mat, left_imgs[0]);
	dm.write_point_cloud_to_file(out_file_pcd);
	
	// cv::reprojectImageTo3D(disp_mat_vec[0],three_d_img,Q,true);
	// std::cout << "line 428 three_d\n " << three_d_img << std::endl;
	// cv::namedWindow("3dI");
	// cv::imshow("3dI",three_d_img);
	// cv::waitKey(0);
	
	// store_point_cloud_from_mat(pc,disp_mat_vec[0]);
	// store_point_cloud_from_mat(pc,left_imgs[0]);
	// store_point_cloud_from_mat3d(pc, three_d_img,left_imgs[0]);
	// store_point_cloud_from_disp_mat(pc,disp_mat_vec[0]);
	// pcl::PCDWriter w;	
	*****************/	
	// std::cout << "saving a pointcloud to " << out_file_pcd << std::endl;
	std::cout << "All done" << std::endl;
	// w.writeBinaryCompressed(out_file_pcd.c_str(), pc);
	//pcl::io::savePCDBinaryCompressed("out.pcd", pc);
	// std::cout << s1;
	return 0;
}
