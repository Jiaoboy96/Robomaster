#include "BigPinwheel.h"

//初始化调参函数2
BigPinwheel::BigPinwheel()
{
	/*小符参数*/
	pinWheelGrayThre = 200;		//灰度阈值
	flabellumMinArea = 500;		//目标扇叶最小面积
	/*大符参数*/
	//找中心
	centerRadiusMin = 100;
	centerRadiusMax = 200;
	centerAreaMin = 50;
	centerLengthWidthRatioMin = 0.9;
	//做预测的角度补偿
	antiAngleDividend = 6;
	obeyAngleDividend = 3;
	/*初始化*/
	rotateArrorNum = 0;
}
BigPinwheel::~BigPinwheel()
{
}
//操作函数
bool BigPinwheel::getCotyledonArror(Mat src,Mat & binary, vector<RotatedRect>& finalArror)
{
	int areaBuffer;
	int minArea=1000000;

	vector< RotatedRect> roiArea;
	//step00，选则有内轮廓且面积最小的子叶
	findContours(binary, step00Contours, step00Hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE); //在二值图像中寻找轮廓
	for (int i = 0; i < step00Contours.size(); i++)
	{
		if (step00Contours[i].size() > 4)
		{
			areaBuffer = contourArea(step00Contours[i],false);	//暂存面积大小
			if (step00Hierarchy[i][2] != -1 && step00Hierarchy[i][3] == -1&& areaBuffer>flabellumMinArea)// 
			{
				if(minArea> areaBuffer)
					minArea = areaBuffer;
			}
		}
	}
	
	/*step01,选择目标子叶,进行感兴趣处理*/
	for (int i = 0; i < step00Contours.size(); i++)
	{
		if (step00Contours[i].size() > 4)
		{
			areaBuffer = contourArea(step00Contours[i], false);	//暂存面积大小
			flabellumRect = minAreaRect(step00Contours[i]);		//暂存大风车子叶
			if (areaBuffer == minArea)
			{
				roiArea.push_back(flabellumRect);
#ifdef Step01FlabellumRect
				drawRect(src, flabellumRect, Scalar(0, 255, 255));
				imshow("大风车子叶", src);
#endif // Step01FlabellumRect
				//开始获取最终装甲板
				getFinalArror(src, binary, roiArea, finalArror);
				
				return true;
			}
		}
	}
	return false;
}
bool BigPinwheel::getFinalArror(Mat src, Mat & binary, vector< RotatedRect> primaryArror, vector<RotatedRect>& finalArror)
{
	if (primaryArror.empty())
	{
		printf("primaryArror.empty\n");
		return false;
	}
	/*Step10,漫水填充*/
	Mat binaryMask(src.size(), CV_8UC1, Scalar(0));
	Point2f point[4];
	primaryArror[0].points(point);

	Rect floodFillRect;
	Rect roiRect = primaryArror[0].boundingRect();	//外接矩形
	if (roiRect.x >= src.cols || roiRect.x < 0 ||
		roiRect.x + roiRect.width >= src.cols || roiRect.x + roiRect.width<0 ||
		roiRect.y>src.rows || roiRect.y<0 ||
		roiRect.y + roiRect.height>src.rows || roiRect.y + roiRect.height < 0
		)
		return false;

	Mat roiDistrict = binary(roiRect);				//感兴趣区
	roiDistrict.copyTo(binaryMask(roiRect));
#ifdef Step10BinaryMask
	imshow("扣", binaryMask);
#endif // Step10BinaryMask

	floodFill(binaryMask, Point(0, 0), Scalar(255), &floodFillRect, Scalar(150), Scalar(0));
	dilate(binaryMask, binaryMask, Mat(), Point(-1, -1), 2); //图像膨胀
	erode(binaryMask, binaryMask, Mat(), Point(-1, -1), 3);
#ifdef Step10FloodFill
	imshow("floodFill", binaryMask);
#endif // Step10FloodFill
	
	/*Step11，轮廓重找*/
	RotatedRect rectBuffer;
	vector<Vec4i> hierarchy;				   //轮廓等级
	vector<vector<Point> > contours;		   //轮廓容器

	findContours(binaryMask, contours, hierarchy, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE); //在binaryMpridectPointk中寻找轮廓

	for (int i = 0; i < contours.size(); i++)
	{
		if (hierarchy[i][3] == 0)			   //只检测内轮廓
		{
			if (contours[i].size() > 4)
			{
				rectBuffer = minAreaRect(contours[i]);
				finalArror.push_back(rectBuffer);
			}
		}
	}
	if (finalArror.size() > 1)
		finalArror.clear();
#ifdef Step11BigPinwheelArror
	if (!finalArror.empty())
	{
		drawRect(src, finalArror[0], Scalar(255, 0, 0));
		imshow("BigPinwheelArror", src);
	}
#endif

	return true;
}
bool BigPinwheel::predictFinalArror(Mat src, vector<RotatedRect>& finalArror)
{
	if (finalArror.empty())
		return false;
	/*step20开始查找中心点*/
	RotatedRect rectBuffer;
	RotatedRect bigPinWheelCenter;
	double radius=0; 
	double radiusBuffer=0;
	double LengthWidthRatioBuffer=0;

	for (int i = 0; i < step00Contours.size(); i++)
	{
		if (step00Contours[i].size() > 4)
		{
			rectBuffer = minAreaRect(step00Contours[i]);
			if (step00Hierarchy[i][2] == -1 && step00Hierarchy[i][3] == -1)
			{
				/*drawRect(src, rectBuffer, Scalar(255, 0, 255));*/
				//距离目标装甲的距离
				radiusBuffer = powf((finalArror[0].center.x - rectBuffer.center.x), 2)
					+ powf((finalArror[0].center.y - rectBuffer.center.y), 2);
				radiusBuffer = sqrtf(radiusBuffer);
#ifdef Step20BigPinWheelCenter
				printf("radiusBuffer=%f\n", radiusBuffer);
#endif
				//大风车中心框的长宽比
				if (rectBuffer.size.width <= rectBuffer.size.height)
					LengthWidthRatioBuffer = rectBuffer.size.width / rectBuffer.size.height;
				else
					LengthWidthRatioBuffer = rectBuffer.size.height / rectBuffer.size.width;

				if (radiusBuffer > centerRadiusMin && radiusBuffer < centerRadiusMax
					&& LengthWidthRatioBuffer> centerLengthWidthRatioMin
					&& rectBuffer.size.area()> centerAreaMin)
				{
						radius = radiusBuffer;
						bigPinWheelCenter = rectBuffer;
				}
			}
		}
	}
#ifdef Step20BigPinWheelCenter
	circle(src, bigPinWheelCenter.center, radius, Scalar(0, 255, 0));
	drawRect(src, bigPinWheelCenter, Scalar(0, 0, 255));
	imshow("bigPinWheelCenter", src);
#endif // Step20BigPinWheelCenter

	/*Step21,开始预测*/
	long double finalAngle = 0, finalX = 0, finalY = 0;

	finalAngle = asin((bigPinWheelCenter.center.x - finalArror[0].center.x) / radius);
	if (radius == 0)
		return false;
	if (bigPinWheelCenter.center.x - finalArror[0].center.x > 0 && bigPinWheelCenter.center.y - finalArror[0].center.y > 0)
	{
		finalAngle = 3.1415 / 2 + finalAngle;
	}
	if (bigPinWheelCenter.center.x - finalArror[0].center.x <= 0 && bigPinWheelCenter.center.y - finalArror[0].center.y > 0)
	{
		finalAngle = 3.1415 / 2 + finalAngle;
	}
	if (bigPinWheelCenter.center.x - finalArror[0].center.x <= 0 && bigPinWheelCenter.center.y - finalArror[0].center.y <= 0)
	{
		finalAngle = 3.1415 / 2 * 3 - finalAngle;
	}
	if (bigPinWheelCenter.center.x - finalArror[0].center.x > 0 && bigPinWheelCenter.center.y - finalArror[0].center.y <= 0)
	{
		finalAngle = 3.1415 / 2 * 3 - finalAngle;
	}
	//预测大风车旋转方向，取前10帧

	if (rotateArrorNum < 10)
	{
		rotateArrorAngle[rotateArrorNum] = finalAngle;
		rotateArrorNum++;
		finalArror.clear();
	}
	else if (rotateArrorNum >= 10)
	{
		/*for (int i = 0; i < 10; i++)
			cout << rotateArrorAngle[i] << " ";
		cout << endl;*/
		//顺时针
		if ((rotateArrorAngle[4] < rotateArrorAngle[0]) && (rotateArrorAngle[9] < rotateArrorAngle[5]))
		{
			if (finalAngle < 6.29)
			{
				finalX = bigPinWheelCenter.center.x
					+ radius * sin(3.14-3.14 / obeyAngleDividend - finalAngle);//
				finalY = bigPinWheelCenter.center.y + radius * -cos(3.14-3.14/ obeyAngleDividend - finalAngle);//3.14/7	
				finalArror[0].center.x = finalX;
				finalArror[0].center.y = finalY;
			}
		}
		//逆时针
		else if ((rotateArrorAngle[4] >= rotateArrorAngle[0]) && (rotateArrorAngle[9] >= rotateArrorAngle[5]))
		{
			if (finalAngle < 6.29)
			{
				finalX = bigPinWheelCenter.center.x + radius * cos(finalAngle + 3.14 / antiAngleDividend);//
				finalY = bigPinWheelCenter.center.y + radius * -sin(finalAngle + 3.14 / antiAngleDividend);//3.14/7	
				finalArror[0].center.x = finalX;
				finalArror[0].center.y = finalY;
			}
		}
#ifdef Step21PredictArror
		circle(src, finalArror[0].center, 5, Scalar(0, 0, 255), -1);
		imshow("PredictArror", src);
#endif // Step21PredictArror
	}
	return true;
}
void BigPinwheel::clear()
{
	rotateArrorNum = 0;
}
//调试函数
void BigPinwheel::drawRect(Mat src, RotatedRect roi, Scalar color, int thickness)
{
	Point2f P[4];
	roi.points(P);//将二维点集存入图像中
	for (int j = 0; j <= 3; j++)
	{
		line(src, P[j], P[(j + 1) % 4], color, thickness);
	}
}