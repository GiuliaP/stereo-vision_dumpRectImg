#include <iCub/stereoVision/elasWrapper.h>

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

elasWrapper::elasWrapper(double _io_scaling_factor, parameters &p) : Elas(p)
{
	io_scaling_factor = _io_scaling_factor;

	cout << "disp_scaling_factor" << io_scaling_factor << endl;

	cout << "subsampling" << get_param<bool>(subsampling) << endl;
	cout << "add_corners" << get_param<bool>(add_corners) << endl;

	cout << "ipol_gap_width" << get_param<int>(ipol_gap_width) << endl;

	cout << "support_threshold" << get_param<float>(support_threshold) << endl;
	cout << "gamma" << get_param<float>(gamma) << endl;
	cout << "sradius" << get_param<float>(sradius) << endl;

	cout << "match_texture" << get_param<int>(match_texture) << endl;

	cout << "filter_median" << get_param<bool>(filter_median) << endl;
	cout << "filter_adaptive_mean" << get_param<bool>(filter_adaptive_mean) << endl;

	cout << "disp_max" << get_param<int>(disp_max) << endl;
}

elasWrapper::elasWrapper() : Elas(parameters(ROBOTICS))
{
	io_scaling_factor = 1.0;
}


double elasWrapper::compute_disparity(cv::Mat &imL, cv::Mat &imR, cv::Mat &dispL)
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

    Mat imR_scaled, imL_scaled;
    if (io_scaling_factor!=1.0)
    {
    	resize(imR, imR_scaled, Size(), io_scaling_factor, io_scaling_factor);
    	resize(imL, imL_scaled, Size(), io_scaling_factor, io_scaling_factor);
    } else
    {
    	imR_scaled = imR.clone();
    	imL_scaled = imL.clone();
    }
    int width = imL_scaled.cols;
    int height = imL_scaled.rows;

	int width_disp_data = get_param<bool>(subsampling) ? width>>1 : width;
	int height_disp_data = get_param<bool>(subsampling) ? height>>1 : height;

	float *dispL_data = (float*) malloc(width_disp_data * height_disp_data * sizeof(float));
	float *dispR_data = (float*) malloc(width_disp_data * height_disp_data * sizeof(float));

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

	process((unsigned char*)imL_scaled.data,(unsigned char*)imR_scaled.data, dispL_data, dispR_data, dims);

	Mat dispL_scaled = Mat(height_disp_data, width_disp_data, CV_32FC1, dispL_data);

    if (io_scaling_factor!=1.0 || get_param<bool>(subsampling)==true)
    	resize(dispL_scaled, dispL, im_size);
    else
    	dispL = dispL_scaled.clone();

	free(dispL_data);
	free(dispR_data);

	return workEnd(start);

}
