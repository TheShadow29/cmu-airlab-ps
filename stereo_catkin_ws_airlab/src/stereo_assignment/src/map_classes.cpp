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
#include "opencv2/stereo.hpp"
#include "opencv2/photo.hpp"

// #include "precomp.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/detail/distortion_model.hpp"
#include "opencv2/calib3d/calib3d_c.h"
#include "opencv2/ximgproc/disparity_filter.hpp"


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

void equalize_illumination(const cv::Mat& inp_img, cv::Mat& equalized_img_out)
{
	cv::Mat clahe_img;
	cv::cvtColor(inp_img,clahe_img, CV_BGR2Lab);
	std::vector<cv::Mat> channels(3);
	cv::split(clahe_img,channels);

	cv::Ptr<cv::CLAHE> clahe_ptr = cv::createCLAHE();
	clahe_ptr->setClipLimit(4);
	cv::Mat dst;
	clahe_ptr->apply(channels[0],dst);
	dst.copyTo(channels[0]);
	cv::merge(channels, clahe_img);

	cv::cvtColor(clahe_img, equalized_img_out, CV_Lab2BGR);

	// cv::namedWindow("Clahe");
	// cv::imshow("Clahe", equalized_img_out);
	// cv::waitKey(0);
	
}

void apply_log(const cv::Mat& inp_img, cv::Mat& log_out)
{
	cv::Mat out_img_gauss_rgb;
	cv::GaussianBlur(inp_img, out_img_gauss_rgb, cv::Size(3,3),0,0, cv::BORDER_DEFAULT);
	cv::Mat out_img_gauss_gray;
	cv::cvtColor(out_img_gauss_rgb,out_img_gauss_gray,cv::COLOR_RGB2GRAY);
	cv::Mat out_img_lap;
	int kernel_size = 3;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	cv::Laplacian(out_img_gauss_gray,out_img_lap, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(out_img_lap, log_out);

	// cv::namedWindow("Lap");
	// cv::imshow("Lap", log_out);
	// cv::waitKey(0);
}

class Disp_map
{
private:
public:
	cv::Mat disp_img;
	cv::Ptr<cv::StereoSGBM> sgbm;
	// cv::Ptr<cv::stereo::StereoBinarySGBM> sgbm;
	int num_disp;
	int wsize;
	cv::Mat disp_mask;
	Disp_map()
	{
		sgbm = cv::StereoSGBM::create(0,16,3);
		// sgbm = cv::stereo::StereoBinarySGBM::create(0,16,3);
	}
	
	Disp_map(cv::Mat _disp_img) : disp_img(_disp_img){}

	void init_disp_mask(int min_l)
	{
		disp_mask = cv::Mat::zeros(disp_img.rows, disp_img.cols,CV_8UC1);
		cv::Mat_<float> disp_f = disp_img;
		disp_f -= min_l;
		
		for(int y = 0; y < disp_f.rows; y++)
		{
			for (int x = 0; x < disp_f.cols; x++)
			{
				if( std::abs(disp_f.at<float>(y,x)) <= .001)
				{
					disp_mask.at<int>(y,x) = 255;
				}
			}
		}
		// std::cout << "line 121 "<< FLT_EPSILON << " disp_mask \n" << disp_mask << std::endl; 
	}

	
	void preprocess_img(const cv::Mat& left,const cv::Mat& right, cv::Mat& left_pre, cv::Mat& right_pre)
	{
		// cv::Mat left_ill;
		// cv::Mat right_ill;
		equalize_illumination(left, left_pre);
		equalize_illumination(right,right_pre);
		// cv::cvtColor(left_pre, left_pre, CV_BGR2GRAY);
		// cv::cvtColor(right_pre, right_pre, CV_BGR2GRAY);
		// left_pre.convertTo(left_pre,CV_8UC1);
		// right_pre.convertTo(right_pre, CV_8UC1);
		// apply_log(left, left_pre);
		// apply_log(right, right_pre);
		// std::cout << "line 114" <<type2str(left_pre.type()) << std::endl;
	}
	
	void init_sgbm(const cv::Mat& left,int block_size = 5, int numberOfDisparities = 64)
	{
		// int block_size = 5;
		num_disp = numberOfDisparities;
		wsize = block_size;
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

		// sgbm -> setBinaryKernelType(1);
		
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

	void use_census_transform(const cv::Mat& left,const cv::Mat& right)
	{
		int h = left.rows;
		int w = left.cols;
		cv::Mat temp(left.size(), CV_32FC1);
		unsigned int census = 0;
		unsigned int bit = 0;
		int m=3;
		int n=3; //window size
		int i,j,x,y;
		int shiftCount = 0;
   
		for (x = m/2; x < h - m/2; x++)
		{
			for(y = n/2; y < w - n/2; y++)
			{
				census = 0;
				shiftCount = 0;
				for (i = x - m/2; i <= x + m/2; i++)
				{
					for (j = y - n/2; j <= y + n/2; j++)
					{
 
						if( shiftCount != m*n/2 )//skip the center pixel
						{
							census <<= 1;
							if( left.at<float>(i,j) < left.at<float>(x,y) )//compare pixel values in the neighborhood
								bit = 1;
							else
								bit = 0;
							census = census + bit;
							//cout<<census<<" ";*/
 
						}
						shiftCount ++;
					}	
				}
          
				temp.ptr<float>(x)[y] = census;
			}
		}
		temp.copyTo(disp_img);
    }

	void post_process_disp_map(const cv::Mat& left, const cv::Mat& right)
	{
		cv::Ptr<cv::ximgproc::DisparityWLSFilter>wls_filter;
		wls_filter = cv::ximgproc::createDisparityWLSFilter(sgbm);
		
		cv::Mat filtered_disp;
		int lambda = 8000*0.5;
		int sigma = 1;
		wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);

		Disp_map ld;
		ld.compute_disp(left,right);
		Disp_map rd;
		rd.compute_disp(right,left);
		std::cout << "line 253 \n";
		// cv::namedWindow("ldb");
		// cv::imshow("ldb",ld.disp_img);
		double min_l, max_l;
		cv::Point min_loc, max_loc;
		cv::minMaxLoc( ld.disp_img, &min_l, &max_l, &min_loc, &max_loc);
		// std::cout << "line 259 " << "min_l " << min_l << " max_l " << max_l << std::endl;
		ld.disp_img -= min_l;
		int scaling_fac = 256 * 128 - 1;
		ld.disp_img.convertTo(ld.disp_img,CV_16S,scaling_fac);
		cv::minMaxLoc( ld.disp_img, &min_l, &max_l, &min_loc, &max_loc);
		std::cout << "line 262 " << "min_l " << min_l << " max_l " << max_l << std::endl;

		cv::minMaxLoc( rd.disp_img, &min_l, &max_l, &min_loc, &max_loc);
		rd.disp_img -= min_l;
		rd.disp_img.convertTo(rd.disp_img,CV_16S,scaling_fac);
		
		// cv::namedWindow("lda");
		// cv::imshow("lda",ld.disp_img);
		// cv::waitKey(0);
		wls_filter->filter(ld.disp_img,left,filtered_disp,rd.disp_img);
		filtered_disp.convertTo(disp_img,CV_32FC1,1.0/(256*128));
		std::cout << "line 255 \n";
		int vis_mult = 1;
		cv::Mat raw_disp_vis;

		cv::ximgproc::getDisparityVis(ld.disp_img,raw_disp_vis,vis_mult);
		cv::namedWindow("raw disparity");
		cv::imshow("raw disparity", raw_disp_vis);
		cv::Mat filtered_disp_vis;
		cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
		cv::namedWindow("filtered disparity");
		cv::imshow("filtered disparity", filtered_disp_vis);
		cv::waitKey(0);

	}

	void post_process_discard_noise()
	{
		
		cv::Mat_<float> disp_f = disp_img;
		double min_l, max_l;
		cv::Point min_loc, max_loc;
		cv::minMaxLoc(disp_f, &min_l, &max_l, &min_loc, &max_loc);
		std::cout << "line 295 " << "min_l " << min_l << " max_l " << max_l << std::endl; 
		this->init_disp_mask(min_l);
		cv::Mat disp_8U;
		disp_f.convertTo(disp_8U,CV_8U,255);
		cv::Mat disp_out_8U;
		// cv::inpaint(disp_8U,disp_mask, disp_out_8U, 20,cv::INPAINT_TELEA);
		disp_f -= min_l;
		disp_f.convertTo(disp_img, CV_32FC1,255);
		// cv::namedWindow("mask");
		// cv::imshow("mask",disp_mask);
		// cv::namedWindow("8U");
		// cv::imshow("8U", disp_out_8U);
		// disp_out_8U.convertTo(disp_img,CV_32FC1,1.0/255);
		// cv::waitKey(0);
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
	// cv::Mat depth_img;
	pcl::PointCloud<pcl::PointXYZRGB> pc;
	pcl::PointCloud<pcl::PointXYZRGB> pc2;
	// cv::Mat disp_mask;
	Depth_map()
	{
		
	};
	
	// Depth_map(cv::Mat _depth_img):depth_img(_depth_img){}
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
				// if (disp_f.at<float>(y,x) == 0)
				// {
				// 	disp_mask.at<short>(y,x) = 1;
				// }
			}
		}
		// cv::Mat Z_mat_temp;
		Z.convertTo(Z_mat, CV_32FC1); // 
		// cv::Mat mask = (disp_f != 0);
		// Z_mat = cv::Mat(Z_mat_temp,mask);
		// std::cout << "line 116 v2 \n" << vector_t2[0] << std::endl; 
		// std::cout << "line 117 Z_mat\n" << Z_mat << "line 114 end \n";
	}
	void generate_point_cloud(cv::Mat Z_mat, cv::Mat left_img)
	{

		std::cout << "line 153 size of Z_mat \n" << Z_mat.size() << std::endl;
		std::cout << "line 154 size of left_img \n" << left_img.size() << std::endl;

		// std::cout << "line 345 disp_mask \n" << disp_mask << std::endl;
		for(int y = 0; y < left_img.rows; y++)
		{
			for (int x = 0; x < left_img.cols; x++)
			{
				// if (disp_mask.at<int>(y,x) == 0)
				if(1)
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
		// this->init_disp_mask(d);
		cv::Mat three_d_img;
		cv::Mat_<float> disp_f = d.disp_img;
		// std::cout << "line 187 disp_f \n" << disp_f << std::endl;
		cv::Mat_<float> Qf = Q;
		// std::cout << "line 189 \n" << Qf << std::endl;
		double min_l, max_l;
		cv::Point min_loc, max_loc;
		cv::minMaxLoc(disp_f, &min_l, &max_l, &min_loc, &max_loc);
		disp_f -= min_l;
		disp_f.convertTo(disp_f, CV_32FC1, 255);

		cv::reprojectImageTo3D(disp_f, three_d_img,Qf);
		std::vector<cv::Mat> points_xyz;
		cv::split(three_d_img,points_xyz);
 
		
		std::cout << "line 425 points_xyz[2] \n " << points_xyz[2] << std::endl;
		// int counter = 0;
		std::vector<cv::Point> idxy;
		cv::Point tp;
		// pcl::PointCloud<pcl::PointXYZ> temp_pc;
		for(int y = 0; y < three_d_img.rows; y++)
		{
			for(int x = 0; x < three_d_img.cols; x++)
			{
				// std::cout << "line 185 \n";
				// if(d.disp_mask.at<int>(y,x) == 0)
				// if(1)
				{
					cv::Vec3f point = three_d_img.at<cv::Vec3f>(y,x);
					// std::cout << "line 187 \n";

					// pcl::PointXYZRGB p;
					// p.x = point[0];
					// p.y = point[1];
					// p.z = point[2];

					// cv::Vec3b bgr(left.at<cv::Vec3b>(y, x));
					// p.b = bgr[0];
					// p.g = bgr[1];
					// p.r = bgr[2];
					if(point[2] > 0)
					{
						tp.x = x;
						tp.y = y;
						idxy.push_back(tp);
					}
					// counter++;
				}
			}
		}
		std::cout << "line 463 \n";
		disp_f.convertTo(disp_f, CV_32FC1,1.0/255);
		
		cv::reprojectImageTo3D(disp_f, three_d_img,Qf);

		for(int k=0; k < idxy.size(); k++)
		{
			cv::Vec3f point = three_d_img.at<cv::Vec3f>(idxy[k].y,idxy[k].x);
			pcl::PointXYZRGB p;
			p.x = point[0];
			p.y = point[1];
			p.z = point[2];

			cv::Vec3b bgr(left.at<cv::Vec3b>(idxy[k].y, idxy[k].x));
			p.b = bgr[0];
			p.g = bgr[1];
			p.r = bgr[2];

			pc2.push_back(p);
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
		w.write(file_name,pc2);
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
