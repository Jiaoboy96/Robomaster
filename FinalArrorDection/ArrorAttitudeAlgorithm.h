#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
class ArrorAttitudeAlgorithm
{
public:
	ArrorAttitudeAlgorithm();
	~ArrorAttitudeAlgorithm();//X:Yaw,Y:Pitch
	int actualDist;
	double focalLenth;
	double angleSover(Mat src, RotatedRect arrorRect, double& yaw, double& pitch);
private:
	double inchPerPixWidth;
	double inchPerPixHeight;
	double ledHeight;
};
