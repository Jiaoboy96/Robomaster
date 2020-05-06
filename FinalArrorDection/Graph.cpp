#include "Graph.h"

Graph::Graph(uint cols, uint rows)
{
	yMapRatio = 15;
	xMapRatio = 2;
	coordPosition = 0;
	coordRows = rows;
	coordCols = cols;
	coordInit(coordImg);
	yawPointBuffer.push_back(Point2f(25, coordRows / 2.0));
	pitchPointBuffer.push_back(Point2f(25, coordRows / 2.0));
}


Graph::~Graph()
{
}

void Graph::coordinateSystem(double yaw, double pitch)
{
	Point2f yawPoint;
	Point2f pitchPoint;

	if (coordPosition < (coordCols - 25) / (xMapRatio*1.0))
	{
		/*��ֱ����*/
		yawPoint.y = coordRows/2.0 - yaw * yMapRatio;
		pitchPoint.y = coordRows / 2.0 - pitch * yMapRatio;

		/*ˮƽ����*/
		yawPoint.x = pitchPoint.x = 25 + coordPosition * xMapRatio;

		/*��ʼ�ݴ�*/
		yawPointBuffer.push_back(yawPoint);
		pitchPointBuffer.push_back(pitchPoint);
		coordPosition++;

		/*��ʼ����*/
		line(coordImg, yawPointBuffer[coordPosition-1], yawPointBuffer[coordPosition],Scalar(0, 255, 0), 2);
		line(coordImg, pitchPointBuffer[coordPosition - 1], pitchPointBuffer[coordPosition], Scalar(0, 0, 127), 2);
	}
	else
	{
		coordPosition = 0;
		//ˢ������ϵ,���vector
		coordInit(coordImg);
		yawPointBuffer.clear();
		pitchPointBuffer.clear();
		yawPointBuffer.push_back(Point2f(25, coordRows / 2.0));
		pitchPointBuffer.push_back(Point2f(25, coordRows / 2.0));
	}

	imshow("CoordImg", coordImg);
	uchar key = uchar(waitKey(1));
	if (key == 'p')
		waitKey(0);		
}

void Graph::coordInit( Mat & src)
{
	int rowsCenter = coordRows / 2.0;
	int colsCenter = coordCols / 2.0;
	Mat buffer(Size(coordCols,coordRows), CV_8UC3, Scalar(255,255,255));
	src=buffer.clone();

	/*������ϵ����*/
	line(src, Point(0, rowsCenter), Point(src.cols, rowsCenter), Scalar(255,0,0));
	line(src, Point(25, 0), Point(25, src.rows), Scalar(255, 0, 0));


	/*��ʼ�̶�*/
	char stringBuffer[25];
	//��ֱ�ᣬ�Ƕ�
		//������
	for (uint nI = 0; nI < 20; nI++)
	{
		sprintf_s(stringBuffer, "%d", nI);
		putText(src, stringBuffer, Point(0, rowsCenter - nI * yMapRatio), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 0, 255), 1, 4);
		line(src, Point(25, rowsCenter - nI * yMapRatio), Point(coordCols, rowsCenter - nI * yMapRatio), Scalar(127, 127, 127), 1);

	}
		//������
	for (uint nI = 1; nI < 20; nI++)
	{
		
		sprintf_s(stringBuffer, "-%d", nI);
		putText(src, stringBuffer, Point(0, rowsCenter + nI * yMapRatio), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 0, 255), 1, 4);
		line(src, Point(25, rowsCenter + nI * yMapRatio), Point(coordCols, rowsCenter + nI * yMapRatio), Scalar(127, 127, 127), 1);
	}
	//ˮƽ�ᣬ֡��
	for (uint nI = 1; nI < (coordCols-25)/(xMapRatio*1.0); nI++)
	{
		if (nI % 50 == 0)
		{
			sprintf_s(stringBuffer, "%d", nI);
			putText(src, stringBuffer, Point(25 + nI * xMapRatio, rowsCenter), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 0, 255), 1, 4);
		}

		line(src, Point(25+nI*xMapRatio, rowsCenter), Point(25 + nI * xMapRatio, rowsCenter), Scalar(0, 0, 255), 2);
	}

	/*Yaw��Pitch����ɫ*/
	putText(src, "Yaw-color", Point(coordCols-100, 12), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 255, 0), 1, 4);
	putText(src, "Pitch-color", Point(coordCols - 100, 30), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 0, 127), 1, 4);
}
