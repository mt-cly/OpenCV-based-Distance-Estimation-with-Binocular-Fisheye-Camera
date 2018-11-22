#include "cfg.h"

void merge4(const cv::Mat& tl, const cv::Mat& tr, const cv::Mat& bl, const cv::Mat& br, cv::Mat& merged)
{
	int type = tl.type();
	cv::Size sz = tl.size();

	merged.create(cv::Size(sz.width * 2, sz.height * 2), type);
	tl.copyTo(merged(cv::Rect(0, 0, sz.width, sz.height)));
	tr.copyTo(merged(cv::Rect(sz.width, 0, sz.width, sz.height)));
	bl.copyTo(merged(cv::Rect(0, sz.height, sz.width, sz.height)));
	br.copyTo(merged(cv::Rect(sz.width, sz.height, sz.width, sz.height)));
}

void merge6(const cv::Mat& p11, const cv::Mat& p12, const cv::Mat& p13, const cv::Mat& p21, const cv::Mat& p22, const cv::Mat& p23, cv::Mat& merged)
{
	int type = p11.type();
	cv::Size sz = p11.size();
	merged.create(cv::Size(sz.width * 3, sz.height * 2), type);
	p11.copyTo(merged(cv::Rect(0, 0, sz.width, sz.height)));
	p12.copyTo(merged(cv::Rect(sz.width, 0, sz.width, sz.height)));
	p13.copyTo(merged(cv::Rect(sz.width * 2, 0, sz.width, sz.height)));
	p21.copyTo(merged(cv::Rect(0, sz.height, sz.width, sz.height)));
	p22.copyTo(merged(cv::Rect(sz.width, sz.height, sz.width, sz.height)));
	p23.copyTo(merged(cv::Rect(sz.width * 2, sz.height, sz.width, sz.height)));
}

void _SGBM(Mat left_img, Mat right_img, Mat & disparity) {
	int SADWindowSize = 5;
	int numberOfDisparities = 0;
	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((w / 8) + 15) & -16;

	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);
	sgbm->setPreFilterCap(31);
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);
	int cn = left_img.channels();
	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setMode(StereoSGBM::MODE_SGBM);
	sgbm->compute(left_img, right_img, disparity);
	//disparity.convertTo(disparity, CV_8U, 255 / (numberOfDisparities*16.));
	disparity.convertTo(disparity, CV_8U, 1.0 / 16);                //除以16得到真实视差值
	//normalize(disparity, disparity, 0, 255, NORM_MINMAX, CV_8UC3);
}

void _BM(Mat left_img, Mat right_img, Mat & disparity) {
	int SADWindowSize = 5;
	int numberOfDisparities = 0;
	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((w / 8) + 15) & -16;
	Ptr<StereoBM> bm = StereoBM::create(16, 9);
	bm->setROI1(Rect());
	bm->setROI2(Rect());
	bm->setPreFilterCap(31);
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setNumDisparities(numberOfDisparities);
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(15);
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);
	bm->compute(left_img, right_img, disparity);
}

void fill_disp(Mat& dispMap) {
	// fill
	uchar * data = (uchar *)dispMap.data;
	uchar * left_data = new uchar[w*h];
	uchar * right_data = new uchar[w*h];
	int threshold = 25;
	for (int i = 0; i < h; i++) {
		uchar left_value = 0, right_value = 0;
		for (int j = i*w; j < (i + 1)*w; j++) {
			left_data[j] = left_value = data[j] > threshold ? data[j] : left_value;
		}
		for (int j = (i + 1)* w - 1; j >= 0; j--) {
			right_data[j] = right_value = data[j] > threshold ? data[j] : right_value;
		}
	}
	for (int i = 0; i < w*h; i++) {
		data[i] = min(left_data[i], right_data[i]);
	}

	/* smooth with filter
	Mat disp_mask = dispMap.clone();
	uchar* data_ = disp_mask.data;
	for (int core_size = 25; core_size >= 1; core_size -= 8) {
		blur(dispMap, disp_mask, Size(core_size, core_size));
		for (int i = 0; i < w*h; i++) {
			data[i] = data[i] > threshold * 2 ? data[i] : data_[i];
		}
	}*/
}

void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K1, Vec3d T)
{
	depthMap = dispMap.clone();

	float fx = K1.at<double>(0, 0);
	float baseline = abs(T[0]);


	uchar* dispData = dispMap.data;
	uchar* depthData = depthMap.data;
	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < h; j++)
		{
			int id = i*h + j;
			if (dispData[id] == 0)  continue; 
			// from \m to \cm by multiply 100
			// attension, the range of uchar is (0, 255), control the coefficient(100) carfully.
			depthData[id] = (uchar)(fx * baseline / dispData[id] * 100); 
		}
	}
}

void get_depth_in_region(Mat &depth, vector<Point2f> dist_points, vector<Point2f> &undist_points, Mat K1, Vec4d D1) {
	int value_cnt[256] = { 0 };
	uchar* data = depth.data;
	fisheye::undistortPoints(dist_points, undist_points, K1, D1);
	float fx = K1.at<double>(0, 0);
	float cx_left = K1.at<double>(0, 2);
	float fy = K1.at<double>(1, 1);
	float cy_left = K1.at<double>(1, 2);
	for (int i = 0; i < undist_points.size(); i++) {
		undist_points[i].x = undist_points[i].x * fx + cx_left;
		undist_points[i].y = undist_points[i].y * fy + cy_left;
		undist_points[i].x = undist_points[i].x <0 ? 0 : undist_points[i].x>w ? w - 1 : undist_points[i].x;
		undist_points[i].y = undist_points[i].y <0 ? 0 : undist_points[i].y>h ? h - 1 : undist_points[i].y;
	}
	printf("(%f,%f)->(%f,%f)\n", dist_points[0].x, dist_points[0].y, undist_points[0].x, undist_points[0].y);
	printf("(%f,%f)->(%f,%f)\n", dist_points[1].x, dist_points[1].y, undist_points[1].x, undist_points[1].y);
	for (int i = undist_points[0].y; i < undist_points[1].y; i++) {
		for (int j = undist_points[0].x;j< undist_points[1].x; j++) {
			value_cnt[data[i*w + h]]++;
		}
	}
	double depth_value = 0, cnt_value = 0;
	for (int i = 1; i < 256; i++) {
		depth_value += value_cnt[i] * i;
		cnt_value += value_cnt[i];
	}
	depth_value /= cnt_value;
	rectangle(depth, undist_points[0], undist_points[1], CV_RGB(0, 255, 0), 2);
	putText(depth, to_string(depth_value), Point(undist_points[0].x, undist_points[1].y), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 2, 8, 0);
} 