#define _CRT_SECURE_NO_WARNINGS

#include"cfg.h"


void merge4(const cv::Mat& tl, const cv::Mat& tr, const cv::Mat& bl, const cv::Mat& br, cv::Mat& merged);
void merge6(const cv::Mat& p11, const cv::Mat& p12, const cv::Mat& p13, const cv::Mat& p21, const cv::Mat& p22, const cv::Mat& p23, cv::Mat& merged);

// two way for generating disparity
void _SGBM(Mat left_img, Mat right_img, Mat & disparity);
void _BM(Mat left_img, Mat right_img, Mat & disparity);

// fill disp img
void fill_disp(Mat& dispMap);

// from disp to depth
void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K1, Vec3d T);

// get_depth_in_region
void get_depth_in_region(Mat& depth, vector<Point2f> dist_points, vector<Point2f> &undist_points, Mat K1, Vec4d D1);

void check_rectify(Mat K1, Vec4d D1, Mat R1, Mat K2, Vec4d D2, Mat R2) {
	printf("Drawing line for checking coefficient...\n\n");
	cv::Mat lmapx, lmapy, rmapx, rmapy;
	/** if you reset the value of (0,0) (1,1) with a smaller number ¡ª¡ª reduce the fx, fy. Then get remap with
	cv::fisheye::initUndistortRectifyMap(K1, D1, R1, K1, distorted_left.size(), CV_16SC2, lmapx, lmapy);
	you can get the complete picture recovered without trim.
	cv::Matx33d newK1 = K1;
	newK1(0, 0) = 100;
	newK1(1, 1) = 100;
	cv::Matx33d newK2 = K2;
	newK2(0, 0) = 100;
	newK2(1, 1) = 100;*/
	cv::fisheye::initUndistortRectifyMap(K1, D1, R1, K1, Size(w, h), CV_16SC2, lmapx, lmapy);
	//recover the right_img and move the center point to that of left_img.
	Mat new_K2 = K2.clone();
	new_K2.at<double>(1, 2) = K1.at<double>(1, 2);
	cv::fisheye::initUndistortRectifyMap(K2, D2, R2, K1, Size(w, h), CV_16SC2, rmapx, rmapy);

	cv::Mat sample_left, sample_right, undist_left, undist_right;
    // draw line for check the result of retifation
	for (int i = 1; i <= num_imgs; ++i)
	{
		char left_img[100], right_img[100];
		sprintf(left_img, "%s%d%s", img_dir, i, leftimg_postfix);
		sprintf(right_img, "%s%d%s", img_dir, i, rightimg_postfix);
		sample_left = imread(left_img, cv::IMREAD_COLOR);
		sample_right = imread(right_img, cv::IMREAD_COLOR);

		cv::remap(sample_left, undist_left, lmapx, lmapy, cv::INTER_LINEAR);
		cv::remap(sample_right, undist_right, rmapx, rmapy, cv::INTER_LINEAR);

		for (int line = 0; line < h; line += 20)
		{
			cv::line(undist_left, cv::Point(0, line), cv::Point(w, line), cv::Scalar(0, 255, 0));
			cv::line(undist_right, cv::Point(0, line), cv::Point(w, line), cv::Scalar(0, 255, 0));
		}
		for (int line = 0; line < w; line += 20)
		{
			cv::line(undist_left, cv::Point(line, 0), cv::Point(line, h), cv::Scalar(0, 255, 0));
			cv::line(undist_right, cv::Point(line, 0), cv::Point(line, h), cv::Scalar(0, 255, 0));
		}

		cv::Mat rectification;
		merge4(sample_left, sample_right, undist_left, undist_right, rectification);
		cv::imwrite(cv::format("%scombine_%d.jpg", img_dir, i), rectification);
	}
}


void test_depth(Mat K1, Vec4d D1, Mat R1, Mat K2, Vec4d D2, Mat R2, Vec3d T) {
	printf("Testing...\n\n");
	Mat orig_left, orig_right;
	orig_left = imread(test_left_path, cv::IMREAD_COLOR);
	orig_right = imread(test_right_path, cv::IMREAD_COLOR);

	cv::Mat lmapx, lmapy, rmapx, rmapy;
	cv::fisheye::initUndistortRectifyMap(K1, D1, R1, K1, Size(w, h), CV_16SC2, lmapx, lmapy);
	//recover the right_img and move the center point to that of left_img. you can use other new camera_metrix like P1(it would show complete picture, I prefer K1)
	//if you use P1, change fx from  K1.at<double>(0, 0) to P1.at<double>(0, 0) in util.cpp 
	cv::fisheye::initUndistortRectifyMap(K2, D2, R2, K1, Size(w, h), CV_16SC2, rmapx, rmapy);
	Mat undist_left, undist_right;
	cv::remap(orig_left, undist_left, lmapx, lmapy, cv::INTER_LINEAR);
	cv::remap(orig_right, undist_right, rmapx, rmapy, cv::INTER_LINEAR);

	//============================== get disparity and fill it ==============================================
	Mat disp, disp_fill;
	_SGBM(undist_left, undist_right, disp);
	disp_fill = disp.clone();
	fill_disp(disp_fill);
	// save file
	Mat merge_disp;
	cvtColor(disp, disp, CV_GRAY2RGB);
	cvtColor(disp_fill, disp_fill, CV_GRAY2RGB);
	merge6(orig_left, undist_left, disp, orig_right, undist_right, disp_fill, merge_disp);
	cvtColor(disp, disp, CV_RGB2GRAY);
	cvtColor(disp_fill, disp_fill, CV_RGB2GRAY);
	imshow("merge_disp", merge_disp);
	//imwrite("C:\\Users\\cly\\Desktop\\disp.png", merge_disp);

	//========================================================================================================
	

	//============================== get depth  ==============================================
	Mat depth;
	disp2Depth(disp, depth, K1, T);
	// get depth of region
	vector<Point2f> dist_points, undist_points;
	// set the region for getting depth
	dist_points.push_back(point_leftTop);
	dist_points.push_back(point_rightBottom);
	get_depth_in_region(depth, dist_points, undist_points, K1, D1);
	rectangle(orig_left, dist_points[0], dist_points[1], CV_RGB(0, 255, 0), 2);
	rectangle(undist_left, undist_points[0], undist_points[1], CV_RGB(0, 255, 0), 2);
	Mat merge_depth;
	cvtColor(disp, disp, CV_GRAY2RGB);
	cvtColor(depth, depth, CV_GRAY2RGB);
	merge4(orig_left, undist_left, disp, depth, merge_depth);
	cvtColor(disp, disp, CV_RGB2GRAY);
	cvtColor(depth, depth, CV_RGB2GRAY);
	imshow("merge_depth", merge_depth);
	//imwrite("C:\\Users\\cly\\Desktop\\depth.png", merge_depth);
	//========================================================================================
	waitKey(1000000);
}


void test_getdeep() {
	// read the coefficient calculated in main.cpp
	cv::Mat K1, K2, R;
	cv::Vec3d T;
	cv::Vec4d D1, D2;
	cv::Mat R1, R2, P1, P2, Q;
	cv::FileStorage fs(out_file, cv::FileStorage::READ);
	fs["K1"] >> K1;
	fs["K2"] >> K2;
	fs["D1"] >> D1;
	fs["D2"] >> D2;
	fs["R"] >> R;
	fs["T"] >> T;

	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs["Q"] >> Q;
	fs.release();
	// draw line for each train data
	//check_rectify(K1, D1, R1, K2, D2, R2);
	// show the disp and depth of test img.
	// Then get the pixel value of test img with a given box
	test_depth(K1, D1, R1, K2, D2, R2, T);
}
