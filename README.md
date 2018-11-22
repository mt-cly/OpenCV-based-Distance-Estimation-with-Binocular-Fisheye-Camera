# Fisheye_getDistance_baseOn_OPENCV
Measure binocular distance with opencv
## Version of package
|package|version|
|------|------|
|opencv| 3.4.3|
|opencv-contrib| |

## Code part
### main.cpp 
　　We calibrate each camera with their chessboard images and get the calibration param (K1, D1, K2, D2, R, T). Then, get the stereo param (R1, R2, P1, P2, Q) with the param above.
### test.cpp
　　We firstly check the correction of param by function 'check_rectify' which would draw lines for checking the alignment of undistorted left_img and undistorted right_img.
　　Then, undistort the test_img with function 'initUndistortRectifyMap' and 'remap'. After that, we can get disparity with SGBM or other algorithm. The last step getting depth translated from disparity is easy.
 

