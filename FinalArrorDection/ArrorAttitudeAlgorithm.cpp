#include "ArrorAttitudeAlgorithm.h"

//#define FOCALLENGTH
ArrorAttitudeAlgorithm::ArrorAttitudeAlgorithm()
{
#ifdef FOCALLENGTH
	actualDist=2100;
#endif
	ledHeight=60;
	focalLenth = 7.72;
	inchPerPixWidth = (4.86 / 728.0);//(ccdWidth/imgWidth)
	inchPerPixHeight = (3.66 / 544.0);//(ccdHeight/imgHeight)
}
ArrorAttitudeAlgorithm::~ArrorAttitudeAlgorithm()
{
}

double ArrorAttitudeAlgorithm::angleSover(Mat src, RotatedRect arrorRect, double & yaw, double & pitch)
{
#ifdef FOCALLENGTH
	//printf("LED物理尺寸=%f\n",arrorRect.size.height*inchPerPixHeight);
	printf("实际焦距（实测参数代入）=%f\n",(arrorRect.size.height*inchPerPixHeight)*actualDist/ledHeight);
#endif
	double diffX, diffY, distance;
	diffX = arrorRect.center.x - src.size().width / 2.0;
	diffY = arrorRect.center.y - src.size().height / 2.0;
	yaw = atan(diffX * (inchPerPixWidth / focalLenth)) * 57.2958;//0.0075 4.8/640
	pitch = atan(diffY * (inchPerPixHeight / focalLenth)) * 57.2958;//0.0075 3.6/480
	//double degree = abs(tan(yaw * 3.14159265 / 180 * 0.25));

	distance = ((ledHeight * focalLenth) / (arrorRect.size.height * inchPerPixHeight) / 10.0);
	return distance;
}
