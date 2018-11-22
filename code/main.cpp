#define _CRT_SECURE_NO_WARNINGS
#include"cfg.h"



void test_getdeep();

void load_image_points(vector< vector< Point2d > >& left_img_points, vector< vector< Point2d > >& right_img_points, vector< vector< Point3d > >& object_points) {
	printf("Finding chessboard corners...\n");

	vector< vector< Point2f > > imagePoints1, imagePoints2;
	vector< Point2f > corners1, corners2;
	Size board_size = Size(board_width, board_height);
	int board_n = board_width * board_height;

	// findChessboardCorners in each img 
	for (int i = 1; i <= num_imgs; i++) {
		char left_img[100], right_img[100];
		sprintf(left_img, "%s%d%s", img_dir, i, leftimg_postfix);
		sprintf(right_img, "%s%d%s", img_dir, i, rightimg_postfix);
		Mat img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
		Mat img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);

		bool found1 = false, found2 = false;

		found1 = cv::findChessboardCorners(img1, board_size, corners1,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		found2 = cv::findChessboardCorners(img2, board_size, corners2,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		/* show the corners in img
		Mat gray1, gray2;
		cv::cvtColor(img1, gray1, CV_BGR2GRAY);
		cv::cvtColor(img2, gray2, CV_BGR2GRAY);
		if (found1)
		{
			cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(gray1, board_size, corners1, found1);
		}
		if (found2)
		{
			cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(gray2, board_size, corners2, found2);
		}*/

		vector<cv::Point3d> obj;
		for (int i = 0; i < board_height; ++i)
			for (int j = 0; j < board_width; ++j)
				obj.push_back(Point3d(double((float)j * square_size), double((float)i * square_size), 0));
		// find corners in both left_img and right_img. save the corners in imagePoints
		if (found1 && found2) {
			cout << i << ". found corners successfully." << endl;
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);
			object_points.push_back(obj);
		}
		else {
			cout << i << ". failed to find corners, you can delete it." << endl;
		}
	}
	cout << endl;
	//save the message of corners as vector< vector< Point2d > >
	for (int i = 0; i < imagePoints1.size(); i++) {
		vector< Point2d> v1, v2;
		for (int j = 0; j < imagePoints1[i].size(); j++) {
			v1.push_back(Point2d((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
			v2.push_back(Point2d((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
		}
		left_img_points.push_back(v1);
		right_img_points.push_back(v2);
	}
}


void calibration(FileStorage fs, vector< vector< Point2d > > left_img_points, vector< vector< Point2d > > right_img_points, vector< vector< Point3d > > object_points,
	Matx33d& K1, Vec4d& D1, Matx33d& K2, Vec4d& D2, Matx33d& R, Vec3d& T) {
	printf("Calibrating...\n\n");
	int flag = 0;
	flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	flag |= cv::fisheye::CALIB_CHECK_COND;
	flag |= cv::fisheye::CALIB_FIX_SKEW;
	//flag |= cv::fisheye::CALIB_FIX_K2;
	//flag |= cv::fisheye::CALIB_FIX_K3;
	//flag |= cv::fisheye::CALIB_FIX_K4;
	cv::fisheye::stereoCalibrate(object_points, left_img_points, right_img_points,
		K1, D1, K2, D2, Size(w, h), R, T, flag,
		cv::TermCriteria(3, 12, 0));

	fs << "K1" << Mat(K1);
	fs << "K2" << Mat(K2);
	fs << "D1" << D1;
	fs << "D2" << D2;
	fs << "R" << Mat(R);
	fs << "T" << T;
}


void stereo(FileStorage fs, Matx33d K1, Vec4d D1, Matx33d K2, Vec4d D2, Matx33d R, Vec3d T) {
	printf("Stereorectifing...\n\n");

	// stereorectify related
	cv::Mat R1, R2, P1, P2, Q;
	cv::fisheye::stereoRectify(K1, D1, K2, D2, Size(w,h), R, T, R1, R2, P1, P2,
		Q, CV_CALIB_ZERO_DISPARITY, Size(w, h), 0.0, 1.1);

	fs << "R1" << R1;
	fs << "R2" << R2;
	fs << "P1" << P1;
	fs << "P2" << P2;
	fs << "Q" << Q;

}


int main(int argc, char const *argv[])
{

	// chessboard point positions related
	vector< vector< Point2d > > left_img_points, right_img_points;
	vector< vector< Point3d > > object_points;
	// calibration related
	cv::Matx33d K1, K2, R;
	cv::Vec3d T;
	cv::Vec4d D1, D2;

	// find corners and get 'left_img_points', 'right_img_points', 'object_points'
	load_image_points(left_img_points, right_img_points, object_points); 

	// calibrate left/right and stereo rectify
	cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
	calibration(fs, left_img_points, right_img_points, object_points, K1, D1, K2, D2, R, T);
	stereo(fs, K1, D1, K2, D2, R, T);
	fs.release();

	//test result
	test_getdeep();

	return 0;
}
