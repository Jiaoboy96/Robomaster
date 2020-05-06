#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class Graph
{
public:
	Graph(uint cols,uint rows);
	~Graph();
	void coordinateSystem(double yaw, double pitch);
	uint coordRows;
	uint coordCols;
private:
	uint yMapRatio;
	uint xMapRatio;
	uint coordPosition;
	vector <Point2f> yawPointBuffer;
	vector <Point2f> pitchPointBuffer;
	Mat coordImg;
	void coordInit(Mat &src);
};

