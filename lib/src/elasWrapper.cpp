#include <iCub/stereoVision/elasWrapper.h>

#include "elas.h"

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

void elasWrapper::init_elas(string _s, double _io_scaling_factor, bool elas_subsampling, bool _add_corners, int _ipol_gap_width)
{

	Elas::setting s;
	s = Elas::MIDDLEBURY;

	if (_s == "ROBOTICS")
		s = Elas::ROBOTICS;

	if (_s == "MIDDLEBURY")
		s = Elas::MIDDLEBURY;

	param = new Elas::parameters(s);

	param->postprocess_only_left = true;

    io_scaling_factor = _io_scaling_factor;

	param->subsampling = elas_subsampling;

    param->add_corners = _add_corners;
	param->ipol_gap_width = _ipol_gap_width;

	elas = new Elas(*param);

    std::cout << "elas_setting " << s << std::endl;
    std::cout << "io_scaling_factor " <<  io_scaling_factor << std::endl;
    std::cout << "elas_subsampling " << param->subsampling << std::endl;
    std::cout << "add_corners " << param->add_corners << std::endl;
    std::cout << "ipol_gap_width " << param->ipol_gap_width << std::endl;
 
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

double elasWrapper::compute_disparity(cv::Mat &imL, cv::Mat &imR, cv::Mat &dispL, int num_disparities)
{

    int64 start = workBegin();

	// check for correct size
	if (imL.cols <= 0 || imL.rows <= 0 || imR.cols <= 0 || imR.rows <= 0
			|| imL.cols != imR.cols || imL.rows != imR.rows)
	{
		cout << "ERROR: Images must be of same size, but" << endl;
		cout << "       I1: " << imL.cols << " x " << imL.rows << ", I2: "
				<< imR.cols << " x " << imR.rows << endl;
		return -1;
	}

	Size im_size = imL.size();

    param->disp_max = num_disparities - 1;

    Mat imR_scaled, imL_scaled;

    if (io_scaling_factor!=1.0)
    {
        resize(imR, imR_scaled, Size(), io_scaling_factor, io_scaling_factor);
        resize(imL, imL_scaled, Size(), io_scaling_factor, io_scaling_factor);
    } else
    {
        imR_scaled = imR;
        imL_scaled = imL;
    }
    width = imL_scaled.cols;
    height = imL_scaled.rows;

	int width_disp_data = param->subsampling ? floor(width / 2) : width;
	int height_disp_data = param->subsampling ? floor(height / 2) : height;

	// if disp memory is free, allocate it
	if (disp_data_allocated == false)
	{
	    dispL_data = (float*) malloc(width_disp_data * height_disp_data * sizeof(float));
	    dispR_data = (float*) malloc(width_disp_data * height_disp_data * sizeof(float));
	    disp_data_allocated == true;
    }

	// prepare input images
	if (imL_scaled.channels() == 3)
	{
		cv::cvtColor(imL_scaled, imL_scaled, CV_BGR2GRAY);
		cv::cvtColor(imR_scaled, imR_scaled, CV_BGR2GRAY);
	}
	if (!imL_scaled.isContinuous())
		imL_scaled = imL_scaled.clone();
	if (!imR_scaled.isContinuous())
		imR_scaled = imR_scaled.clone();

    // compute disparity
	const int32_t dims[3] = {width,height,width}; // bytes per line = width

	elas->Elas::process((unsigned char*)imL_scaled.data,(unsigned char*)imR_scaled.data, dispL_data, dispR_data, dims);

	dispL = Mat(height_disp_data, width_disp_data, CV_32FC1, dispL_data);

	if (io_scaling_factor!=1.0)
	   resize(dispL, dispL, im_size);

	return workEnd(start);

}
