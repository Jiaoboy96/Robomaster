#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>  
#include <iostream>
using namespace cv;
using namespace ml;
using namespace std;

//#define pinWheelDebugging
//@@@�ϳ�ע��
#ifdef pinWheelDebugging

		/*������*/
#define PinWheelTrackbar
		/*��ֵ��*/
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
//����׼��
	int pinWheelGrayThre;
	int flabellumMinArea;
	int centerRadiusMin;
	int centerRadiusMax;
	int centerAreaMin;
	int antiAngleDividend;
	int obeyAngleDividend;
	double centerLengthWidthRatioMin;

//ʵ�ֺ���
	RotatedRect flabellumRect;														//Ŀ����Ҷ
	vector<vector<Point>> step00Contours;
	vector<Vec4i> step00Hierarchy;
	int rotateArrorNum;
	double rotateArrorAngle[10];
	bool getCotyledonArror(Mat src, Mat& binary,vector<RotatedRect>& finalArror);		//����Ҷ
	bool getFinalArror(Mat src,Mat& binary, vector< RotatedRect> primaryArror
		, vector<RotatedRect>& finalArror);											//��ˮ����װ��
	bool predictFinalArror(Mat src, vector<RotatedRect>& finalArror);
	void clear();
//���Ժ���
	void drawRect(Mat src, RotatedRect roi, Scalar color, int thickness=2);				//���ƾ���
};

