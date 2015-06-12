   	/* 'scaling_factor'
    	 * You can set it to 'true' to resize the left and right images
    	 * before computing the disparity map (and finally resize again
    	 * the resulting map to the original size). It is a multiplicative factor.
    	 * For example, if a low-resolution (also less accurate!) disparity map
    	 * is sufficient, but needed at high rate, you can set this parameter to 1/N
    	 * so that the images width and height are divided by a factor N,
    	 * ELAS computes the disparity of the downsampled images,
    	 * and finally the disparity is rescaled by N before returning.
    	 *
    	 * 'elas_setting'
    	 * Two settings of parameters are defined in elas.h: MIDDLEBURY and ROBOTICS.
    	 * This wrapper gives the possibility of modifying (eventually tuning to individual needs)
    	 * those which differ between the two settings, plus a couple of others
    	 * ('subsampling' and 'add_corners') related to computational time.
    	 * The other parameters are supposed to be fixed to the values proposed by the authors of ELAS.
    	 *
    	 * 'elas_subsampling'
    	 * Set to 'true' to speedup the computation inside ELAS
    	 * (at the expenses of accuracy).
    	 *
    	 * 'elas_add_corners'
         * Set to 'false' to discard corners, 'true' to consider them.
    	 *
    	 *
    	 * We leave the possibility of modifying the following parameters also,
    	 * described in ELAS documentation (and in elas.h).
    	 *
    	 * 'elas_ipol_gap_width'
    	 * It can vary between 3 (ROBOTICS) and 5000 (MIDDLEBURY).
    	 *
    	 * 'elas_support_threshold'
    	 * Set to 0.95 in MIDDLEBURY and 0.85 in ROBOTICS.
    	 *
    	 * 'elas_gamma'
    	 * Set to 5 in MIDDLEBURY and 3 in ROBOTICS.
    	 *
    	 * 'elas_sradius'
    	 * Set to 3 inMIDDLEBURY and 2 in ROBOTICS.
    	 *
    	 * 'elas_match_texture'
    	 * Set to 0 in MIDDLEBURY and 1 in ROBOTICS.
    	 *
    	 * 'elas_filter_median'
    	 * Set to 1 in MIDDLEBURY and 0 in ROBOTICS.
    	 *
    	 * 'elas_filter_adaptive_mean'
    	 * Set to 0 in MIDDLEBURY and 1 in ROBOTICS.
    	 *
    	 */
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

class elasWrapper : public Elas {

	double io_scaling_factor;

public:

	int64 workBegin();
	double workEnd(int64 work_begin);
    
	elasWrapper();
	elasWrapper(double scaling_factor, string elas_setting);

	double compute_disparity(cv::Mat &imL, cv::Mat &imR, cv::Mat &dispL, int num_disparities);

};

#endif /* ELASWRAPPER_H_ */
