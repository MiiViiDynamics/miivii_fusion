#ifndef GENCOLORS_H_
#define GENCOLORS_H_


#include "opencv2/core/core.hpp"
//#include "precomp.hpp"
#include <opencv2/opencv.hpp>

#include <iostream>

using namespace cv;

static void downsamplePoints(const Mat& src, Mat& dst, size_t count);

void generateColors(std::vector<Scalar>& colors, size_t count, size_t factor = 100);


#endif  // GENCOLORS_CPP
