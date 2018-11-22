#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

// base message for calibration and rectification
#define w 480
#define h 640
#define board_width 7
#define board_height 5
#define num_imgs 21
#define square_size 0.04233
// the path to chessboard imgs
#define img_dir "C:\\Users\\cly\\Desktop\\data2\\imgs\\"
// the chessborad img named "i_left.png" or "i_right_png" for i = 1,2,3,...,num_imgs
#define leftimg_postfix "_left.png"
#define rightimg_postfix "_right.png"
// the path for saving the result of calibration and strereorectify
#define out_file "C:\\Users\\cly\\Desktop\\data2\\result.yml"
// the region in original img for getting the depth of that
// the message would be used in function 'test_depth' of 'test.cpp'
#define point_leftTop  Point2f(170, 373)
#define point_rightBottom Point2f(257, 435)
//base message for test
#define test_left_path "C:\\Users\\cly\\Desktop\\data2\\test\\7_left.png"
#define test_right_path "C:\\Users\\cly\\Desktop\\data2\\test\\7_right.png"
