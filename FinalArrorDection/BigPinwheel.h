#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>  
#include <iostream>
using namespace cv;
using namespace ml;
using namespace std;

//#define pinWheelDebugging
//@@@上场注释
#ifdef pinWheelDebugging

		/*滑动条*/
#define PinWheelTrackbar
		/*二值化*/
#define MainPinWheelGrayBinary
		/*Step0*/
//#define Step01FlabellumRect
		/*Step1*/
//#define Step10BinaryMask
#define Step10FloodFill
#define Step11BigPinwheelArror
		/*Step2*/
#define Step20BigPinWheelCenter
#define Step21PredictArror

#endif // pinWheelDebugging

		
class BigPinwheel
{
public:
	BigPinwheel();
	~BigPinwheel();
//调参准备
	int pinWheelGrayThre;
	int flabellumMinArea;
	int centerRadiusMin;
	int centerRadiusMax;
	int centerAreaMin;
	int antiAngleDividend;
	int obeyAngleDividend;
	double centerLengthWidthRatioMin;

//实现函数
	RotatedRect flabellumRect;														//目标扇叶
	vector<vector<Point>> step00Contours;
	vector<Vec4i> step00Hierarchy;
	int rotateArrorNum;
	double rotateArrorAngle[10];
	bool getCotyledonArror(Mat src, Mat& binary,vector<RotatedRect>& finalArror);		//找扇叶
	bool getFinalArror(Mat src,Mat& binary, vector< RotatedRect> primaryArror
		, vector<RotatedRect>& finalArror);											//漫水填充框装甲
	bool predictFinalArror(Mat src, vector<RotatedRect>& finalArror);
	void clear();
//调试函数
	void drawRect(Mat src, RotatedRect roi, Scalar color, int thickness=2);				//绘制矩形
};

