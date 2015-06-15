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

	enum param_field {
	  		disp_min,
	  		disp_max,
	  		support_threshold,
	  		support_texture,
	  		candidate_stepsize,
	  		incon_window_size,
	  		incon_threshold,
	  		incon_min_support,
	  		add_corners,
	  		grid_size,
	  		beta,
	  		gamma,
	  		sigma,
	  		sradius,
	  		match_texture,
	  		lr_threshold,
	  		speckle_size,
	  		speckle_sim_threshold,
	  		ipol_gap_width,
	  		filter_median,
	  		filter_adaptive_mean,
	  		postprocess_only_left,
	  		subsampling
	};

	int64 workBegin();
	double workEnd(int64 work_begin);
    
	elasWrapper();
	elasWrapper(double scaling_factor, string elas_setting);

	double compute_disparity(cv::Mat &imL, cv::Mat &imR, cv::Mat &dispL, int num_disparities);

	template <typename T>
	T get_param(param_field param_name)
	{
		switch (param_name) {
		case add_corners:
			return param.add_corners;
			break;
		case filter_median:
			return param.filter_median;
			break;
		case filter_adaptive_mean:
			return param.filter_adaptive_mean;
			break;
		case postprocess_only_left:
			return param.postprocess_only_left;
			break;
		case subsampling:
			return param.subsampling;
			break;
		case disp_min:
			return param.disp_min;
			break;
		case disp_max:
			return param.disp_max;
			break;
		case support_texture:
			return param.support_texture;
			break;
		case candidate_stepsize:
			return param.candidate_stepsize;
			break;
		case incon_window_size:
			return param.incon_window_size;
			break;
		case incon_threshold:
			return param.incon_threshold;
			break;
		case incon_min_support:
			return param.incon_min_support;
			break;
		case grid_size:
			return param.grid_size;
			break;
		case match_texture:
			return param.match_texture;
			break;
		case lr_threshold:
			return param.lr_threshold;
			break;
		case speckle_size:
			return param.speckle_size;
			break;
		case ipol_gap_width:
			return param.ipol_gap_width;
			break;
		case support_threshold:
			return param.support_threshold;
			break;
		case beta:
			return param.beta;
			break;
		case gamma:
			return param.gamma;
			break;
		case sigma:
			return param.sigma;
			break;
		case sradius:
			return param.sradius;
			break;
		case speckle_sim_threshold:
			return param.speckle_sim_threshold;
			break;
		default:
			std::cout << "param_set failed: " << param_name << std::endl;
			return 0;
			break;
		};
	}

	template <typename T>
	void set_param(param_field param_name, T param_value)
	{
		switch (param_name) {
		case add_corners:
			param.add_corners = param_value;
			break;
		case filter_median:
			param.filter_median  = param_value;;
			break;
		case filter_adaptive_mean:
			param.filter_adaptive_mean = param_value;
			break;
		case postprocess_only_left:
			param.postprocess_only_left = param_value;
			break;
		case subsampling:
			param.subsampling = param_value;
			break;
		case disp_min:
			param.disp_min = param_value;
			break;
		case disp_max:
			param.disp_max = param_value;
			break;
		case support_texture:
			param.support_texture = param_value;
			break;
		case candidate_stepsize:
			param.candidate_stepsize = param_value;
			break;
		case incon_window_size:
			param.incon_window_size = param_value;
			break;
		case incon_threshold:
			param.incon_threshold = param_value;
			break;
		case incon_min_support:
			param.incon_min_support = param_value;
			break;
		case grid_size:
			param.grid_size = param_value;
			break;
		case match_texture:
			param.match_texture = param_value;
			break;
		case lr_threshold:
			param.lr_threshold = param_value;
			break;
		case speckle_size:
			param.speckle_size = param_value;
			break;
		case ipol_gap_width:
			param.ipol_gap_width = param_value;
			break;
		case support_threshold:
			param.support_threshold = param_value;
			break;
		case beta:
			param.beta = param_value;
			break;
		case gamma:
			param.gamma = param_value;
			break;
		case sigma:
			param.sigma = param_value;
			break;
		case sradius:
			param.sradius = param_value;
			break;
		case speckle_sim_threshold:
			param.speckle_sim_threshold = param_value;
			break;
		default:
			break;
		};
	}

};

#endif /* ELASWRAPPER_H_ */
