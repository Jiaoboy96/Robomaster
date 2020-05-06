#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>  
#include <iostream>
#include <fstream>

using namespace cv;
using namespace ml;
using namespace std;

#define DEBUG
/*
@@@调试打开，上场注释
*/
#ifdef DEBUG

#define ImgClone
/*滑动条调试*/
#define MainTrackbar

/*感兴趣掩码*/
//#define MainSrcMask						

/*二值化*/
//*
#define MainGrayBinary
#define MainColorBinary
#define MainMergeBinary
//*/

/*检测LED*/
//#define DetectionContourDrawStep00
//#define DetectionContourDrawStep01

/*拟合LED*/
//#define DetectionContourDrawStep10

/*机器学习筛选装甲*/
//#define MLStep20
#define MLStep21
#define MLStep22

/*非机器学习筛选装甲*/
#define NonMLStep20
#endif // DEBUG                                                                                                                                                                                                                                                                                                                                                                                                               

/*
@@@上场打开，调试注释
*/
/*操作系统选择*/
#define MLWriter
#define WIN10
//#define LINUX

class ArrorRecognition
{
public:
	ArrorRecognition();
	~ArrorRecognition();
//调参准备
	uchar ImgWriter;
	int fileHeaderNum;

	uchar mainColor;
	int mainColorThre;
	int mainGrayThre;
	int neighborGrayThre;
	int svmWidth;
	int svmHeight;

//Step0
	int ledMinAngle;
	int ledMaxAngle;
	int ledMinArea;
	int ledMaxArea;	
	double ledWitdhHeightRatio;
//Step1
	int arrorMinDiffX;
	int arrorMaxDiffX;
	int arrorMaxDiffY;
	int arrorMinAngle;
	int arrorMaxAngle;
//Step2
	uchar mlGrayOrThre;
	int mlArrorNum;
//实现函数
	int otsuThre(Mat image);

	void getArrorNumBinary(Mat src, Mat& finalNum);

	void getColorBinary(Mat src, Mat& dst, uchar colorFlag, uchar colorThre);										//RGB二值

	void mergeGrayColor(Mat colorBinary,Mat grayBinary,Mat& mergeBinary);

	void hogDetector(Mat src, Mat& featureMat);																		//Hog标准化尺寸
	
	int detectionContour(Mat src, Mat& mergeBinary, vector<RotatedRect>& arrorRect, Rect srcRoi);					//二值，提取装甲轮廓,调参
	
	int finalMachineLearn(Mat src, Mat mergeBinary, vector<RotatedRect>& arrorRect,									
		vector<RotatedRect>& finalArror, int& finalNum, Ptr<SVM> svmLoad);                                                                  
	int nonML(Mat src, vector<RotatedRect>& arrorRect, vector<RotatedRect>& finalArror);							//不加机器学习

	int mlNumNonMl(Mat src, Mat mergeBinary, vector<RotatedRect>& arrorRect,	
		vector<RotatedRect>& finalArror, int& finalNum, Ptr<SVM> svmLoad);
																													//调试函数
	static void drawRect(Mat src, RotatedRect roi, Scalar color = Scalar(0, 255, 255), int thickness = 2);
private:
	char stringBuffer[100];
	uchar fileHeader;
	uint fileNum;
};
