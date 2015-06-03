#ifndef ELASWRAPPER_H_
#define ELASWRAPPER_H_

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>

// OpenCV
#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "elas.h"

using namespace cv;
using namespace std;

class elasWrapper {

	int32_t width, height;
	float *dispL_data, *dispR_data;
	bool disp_data_allocated;

public:

	Elas::parameters *param;
	Elas *elas;

	elasWrapper();

	int64 workBegin();
	double workEnd(int64 work_begin);

	void init_elas(string s, bool elas_subsampling);

	double compute_disparity(cv::Mat &imL, cv::Mat &imR, cv::Mat &dispL);

	void release_elas();

};

#endif /* ELASWRAPPER_H_ */
