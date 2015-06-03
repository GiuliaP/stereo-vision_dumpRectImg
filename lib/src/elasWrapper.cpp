#include <iCub/stereoVision/elasWrapper.h>

#include "elas.h"
//#include <iostream.h>

int64 elasWrapper::workBegin()
{
	return getTickCount();
}

double elasWrapper::workEnd(int64 work_begin)
{
    int64 d = getTickCount() - work_begin;
    double f = getTickFrequency();
    double work_time = d / f;
    return work_time;
}

elasWrapper::elasWrapper()
{

	width = 0;
	height = 0;

	disp_data_allocated = false;
}

void elasWrapper::init_elas(string _s, bool elas_subsampling)
{

	Elas::setting s;
	s = Elas::MIDDLEBURY;

	if (_s == "ROBOTICS")
		s = Elas::ROBOTICS;

	if (_s == "MIDDLEBURY")
		s = Elas::MIDDLEBURY;

	param = new Elas::parameters(s);

	param->postprocess_only_left = true;
	param->subsampling = elas_subsampling;

	param->disp_max = (width<=320)?95:127;

	elas = new Elas(*param);

    std::cout << "elas_setting " << _s << std::endl;
 
}

void elasWrapper::release_elas()
{

	if (param!=NULL)
	{
		delete param;
		param = NULL;
	}

	if (elas!=NULL)
	{
		delete elas;
		elas = NULL;
	}

	width = 0;
    height = 0;

	if (dispL_data != NULL)
	{
		free(dispL_data);
	}

	if (dispR_data != NULL)
	{
		free(dispR_data);
	}

	disp_data_allocated = false;

}

double elasWrapper::compute_disparity(cv::Mat &imL, cv::Mat &imR, cv::Mat &dispL)
{

	// check for correct size
	if (imL.cols <= 0 || imL.rows <= 0 || imR.cols <= 0 || imR.rows <= 0
			|| imL.cols != imR.cols || imL.rows != imR.rows)
	{
		cout << "ERROR: Images must be of same size, but" << endl;
		cout << "       I1: " << imL.cols << " x " << imL.rows << ", I2: "
				<< imR.cols << " x " << imR.rows << endl;
		return -1;
	}

	// start timer
	int64 start = workBegin();

	// if image size is changed, free disp memory
	/*if ( (disp_data_allocated == true  && imL.cols != width ) || ( disp_data_allocated = true && imL.rows != height) )
	{
		width = 0;
	    height = 0;

		if (dispL_data != NULL)
		{
			free(dispL_data);
		}

		if (dispR_data != NULL)
		{
			free(dispR_data);
		}

		disp_data_allocated = false;
	}*/

	int width_disp_data;
    int height_disp_data;

	// if disp memory is free, allocate it
	if (disp_data_allocated == false)
	{
		width = imL.cols;
		height = imL.rows;
		width_disp_data = param->subsampling ? floor(width / 2) : width;
		height_disp_data = param->subsampling ? floor(height / 2) : height;

	    dispL_data = (float*) malloc(width_disp_data * height_disp_data * sizeof(float));
	    dispR_data = (float*) malloc(width_disp_data * height_disp_data * sizeof(float));

	    disp_data_allocated == true;

	} else
	{
		width_disp_data = param->subsampling ? floor(width / 2) : width;
	    height_disp_data = param->subsampling ? floor(height / 2) : height;
	}

	// prepare input images
	if (imL.channels() == 3)
	{
		cv::cvtColor(imL, imL, CV_BGR2GRAY);
		cv::cvtColor(imR, imR, CV_BGR2GRAY);
	}
	if (!imL.isContinuous())
		imL = imL.clone();
	if (!imR.isContinuous())
		imR = imR.clone();

    // compute disparity
	const int32_t dims[3] = {width,height,width}; // bytes per line = width

	elas->Elas::process((unsigned char*)imL.data,(unsigned char*)imR.data, dispL_data, dispR_data, dims);

	dispL = Mat(height_disp_data, width_disp_data, CV_32FC1, dispL_data);
	//dispR = Mat(height_disp_data, width_disp_data, CV_32FC1, dispR_data);

	return workEnd(start);

}
