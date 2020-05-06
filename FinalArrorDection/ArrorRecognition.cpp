#include "ArrorRecognition.h"
//初始化调参函数1
ArrorRecognition::ArrorRecognition()
{
	mainColor = 'b';
	mainColorThre =	40;
	mainGrayThre = 130;
	neighborGrayThre = 120;

	ImgWriter = 'N';

//Step0,led灯柱
	ledMinAngle = 50;
	ledMaxAngle = 130;
	ledMinArea = 80;
	ledMaxArea = 3000;
	ledWitdhHeightRatio = 5.9 / 6;
//Step1，装甲匹配
	arrorMinDiffX = 10;
	arrorMaxDiffX = 350;
	arrorMaxDiffY = 25;
	arrorMinAngle = 80;
	arrorMaxAngle = 100;
//Step2，机器学习滤波
	mlArrorNum = 0;
	mlGrayOrThre = 'Y';
	svmWidth = 48;
	svmHeight = 48;
	fileNum = 0;

	fileHeader = 'w';
	fileHeaderNum = 10;
}
ArrorRecognition::~ArrorRecognition()
{
}

int ArrorRecognition::otsuThre(Mat image)
{
	int width = image.cols;
	int height = image.rows;
	int x = 0, y = 0;
	int pixelCount[256];
	float pixelPro[256];
	int pixelSum = width * height, threshold = 0;


	//初始化  
	for (uint nI = 0; nI < 256; nI++)
	{
		pixelCount[nI] = 0;
		pixelPro[nI] = 0;
	}

	//统计灰度级中每个像素在整幅图像中的个数  
	for (uint nI = y; nI < height; nI++)
	{
		uchar* pixData = image.ptr<uchar>(nI);
		for (uint nJ = x; nJ < width; nJ++)
		{
			pixelCount[pixData[nJ]] = pixData[nJ];
		}
	}


	//计算每个像素在整幅图像中的比例  
	for (uint nI = 0; nI < 256; nI++)
	{
		pixelPro[nI] = (float)(pixelCount[nI]) / (float)(pixelSum);
	}

	//经典ostu算法,得到前景和背景的分割  
	//遍历灰度级[0,255],计算出方差最大的灰度值,为最佳阈值  
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (uint nI = 0; nI < 256; nI++)//i为阈值
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

		for (uint nJ = 0; nJ < 256; nJ++)//遍历阈值数组当i=0，1，2，3...255
		{
			if (nJ <= nI) //背景部分  
			{
				//以i为阈值分类，第一类总的概率  
				w0 += pixelPro[nJ];
				u0tmp += nJ * pixelPro[nJ];
			}
			else       //前景部分  
			{
				//以i为阈值分类，第二类总的概率  
				w1 += pixelPro[nJ];
				u1tmp += nJ * pixelPro[nJ];
			}
		}

		u0 = u0tmp / w0;        //第一类的平均灰度  
		u1 = u1tmp / w1;        //第二类的平均灰度  
		u = u0tmp + u1tmp;      //整幅图像的平均灰度  
		//计算类间方差  
		deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);
		//找出最大类间方差以及对应的阈值  
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			threshold = nI;
		}
	}
	return threshold;
}
void ArrorRecognition::getArrorNumBinary(Mat src, Mat & finalNum)
{
	//@@@直方图均衡化
	equalizeHist(src, src);
	int thre= otsuThre(src);
#ifdef MLStep20
	printf("thre=%d\n", thre);
	imshow("equalizeHist", src);
#endif // MLStep20
	//@@@大津二值化
	threshold(src, finalNum, thre, 255, CV_THRESH_BINARY);
	//@@@去除椒盐噪点
	medianBlur(finalNum, finalNum, 5);
}
//实现函数
void ArrorRecognition::getColorBinary(Mat src, Mat& dst, uchar colorFlag, uchar colorThre)
{
	switch (colorFlag)
	{
	case 'b':
	case 'B':
		//*
		for (int nI = 0; nI < src.rows; nI++)
		{
			uchar* bgr = src.ptr<uchar>(nI);
			uchar* pix = dst.ptr<uchar>(nI);
			for (int nJ = 0, nK = 0; nJ < src.cols * 3; nJ += 3, nK++)//B:bgr[nJ],G:bgr[nJ+1],R:bgr[nJ+2]
			{
				if ((bgr[nJ] - bgr[nJ+2]) >= colorThre)//B-R
					pix[nK] = 255;
				else
					pix[nK] = 0;
			}
		}
		break;
	case 'r':
	case 'R':
		for (int nI = 0; nI < src.rows; nI++)
		{
			uchar* bgr = src.ptr<uchar>(nI);
			uchar* pix = dst.ptr<uchar>(nI);
			for (int nJ = 0, nK = 0; nJ < src.cols * 3; nJ += 3, nK++)//B:bgr[nJ],G:bgr[nJ+1],R:bgr[nJ+2]
			{
				if ((bgr[nJ + 2] - bgr[nJ]) >= colorThre)//R-B
					pix[nK] = 255;
				else
					pix[nK] = 0;
			}
		}
		break;
	}
	dilate(dst, dst, Mat(), Point(-1, -1), 5); //图像膨胀
}
void ArrorRecognition::mergeGrayColor(Mat colorBinary, Mat grayBinary, Mat & mergeBinary)
{
	for (int nI = 0; nI < colorBinary.rows; nI++)
	{
		uchar* colorPix = colorBinary.ptr<uchar>(nI);
		uchar* grayPix = grayBinary.ptr<uchar>(nI);
		uchar* mergePix= mergeBinary.ptr<uchar>(nI);
		for (int nJ = 0; nJ < colorBinary.cols; nJ ++)//B:bgr[nJ],G:bgr[nJ+1],R:bgr[nJ+2]
		{
			if (grayPix[nJ]==255 && colorPix[nJ] == 255)
				mergePix[nJ] = 255;
			else
				mergePix[nJ] = 0;
		}
	}
}
void ArrorRecognition::hogDetector(Mat src, Mat & featureMat)
{
	Size winSize = Size(svmWidth, svmHeight);
	Size blockSize = Size(8, 8);
	Size blockStride = Size(4, 4);
	Size cellSize = Size(4, 4);
	int nBins = 9;
	HOGDescriptor hog(winSize, blockSize, blockStride, cellSize, nBins);

	vector<float> descriptors;
	hog.compute(src, descriptors);
	featureMat = Mat::zeros(1, descriptors.size(), CV_32FC1);

	for (int j = 0; j < descriptors.size(); j++)
	{
		featureMat.at<float>(0, j) = descriptors[j];
	}
}

int ArrorRecognition::detectionContour(Mat src, Mat& mergeBinary, vector<RotatedRect>& arrorRect,Rect srcRoi)
{
	Point2f P[4];
	vector<Vec4i> mergeHierarchy;
	vector<vector<Point>> mergeContours;
	vector<RotatedRect> roiPoints;

#ifdef ImgClone
	Mat srcClone = src.clone();
#endif // ImgClone
//Step0
	findContours(mergeBinary, mergeContours, mergeHierarchy, CV_RETR_LIST, CHAIN_APPROX_NONE, Point());
	for (int i = 0; i < mergeContours.size(); i++)
	{
		if (mergeContours[i].size() < 6) //进行拟合点至少要大于6，否则报错
			continue;
		RotatedRect mergeContoursArea = fitEllipse(mergeContours[i]);

#ifdef DetectionContourDrawStep00
		ellipse(mergeBinary, mergeContoursArea, cv::Scalar(255, 255, 255), 2, 8);
		line(mergeBinary, mergeContoursArea.center, mergeContoursArea.center, Scalar(255, 255, 255), 2, 8);
#endif // DetectionContourDrawStep00

		if ((mergeContoursArea.angle < ledMinAngle || mergeContoursArea.angle>ledMaxAngle) &&
			(mergeContoursArea.size.area() > ledMinArea && mergeContoursArea.size.area() < ledMaxArea)&&
			(mergeContoursArea.size.width/mergeContoursArea.size.height)<=ledWitdhHeightRatio)
		{

#ifdef DetectionContourDrawStep01
			mergeContoursArea.center.x += srcRoi.x;
			mergeContoursArea.center.y += srcRoi.y;
			ellipse(srcClone, mergeContoursArea, Scalar(0, 255, 255), 2, 8);
			line(srcClone, mergeContoursArea.center, mergeContoursArea.center, Scalar(0, 255, 255), 2, 8);
			mergeContoursArea.center.x -= srcRoi.x;
			mergeContoursArea.center.y -= srcRoi.y;
#endif // DetectionContourDrawStep01

			roiPoints.push_back(mergeContoursArea);//Step0筛选存入
		}
	}



//Step1
	if (roiPoints.size() < 2) //如果检测到的旋转矩形个数小于2，则直接返回
	{
		arrorRect.clear();
	}
	else
	{
		for (unsigned int nI = 0; nI < roiPoints.size() - 1; nI++) //求任意两个旋转矩形的夹角
		{
			uchar count = 0;
			RotatedRect roiMin;
			//ellipse(srcClone, roiPoints[nI], Scalar(0, 255, 255), 2, 8);
			for (unsigned int nJ = nI + 1; nJ < roiPoints.size(); nJ++)
			{
				double areaProp;
				if (roiPoints[nI].size.area() < roiPoints[nJ].size.area())
					areaProp = roiPoints[nI].size.area() / roiPoints[nJ].size.area();
				else
					areaProp = roiPoints[nJ].size.area() / roiPoints[nI].size.area();
				if (areaProp > 0.3)
				{
					//printf("LED面积之比=%f\n", areaProp);
					double dX;
					double dY;
					double dAngle;
					double dWidthJudge;
					RotatedRect roiArror;
					dX = abs(roiPoints[nJ].center.x - roiPoints[nI].center.x);//水平方向偏置
					dY = abs(roiPoints[nJ].center.y - roiPoints[nI].center.y);//垂直方向偏置

					if (roiPoints[nJ].angle > 90)
						roiPoints[nJ].angle = 180 - roiPoints[nJ].angle;
					if (roiPoints[nI].angle > 90)
						roiPoints[nI].angle = 180 - roiPoints[nI].angle;
					dAngle = abs(roiPoints[nJ].angle - roiPoints[nI].angle);
					printf("两灯条的角度差=%f\n", dAngle);

					if (dX >= arrorMinDiffX && dX <= arrorMaxDiffX && dY <= arrorMaxDiffY && dAngle <= arrorMinAngle)
					{
						roiArror.center.x = (roiPoints[nJ].center.x + roiPoints[nI].center.x) / 2.0 + srcRoi.x;
						roiArror.center.y = (roiPoints[nJ].center.y + roiPoints[nI].center.y) / 2.0 + srcRoi.y;
						if(roiPoints[nJ].size.height> roiPoints[nI].size.height)
							dWidthJudge = roiPoints[nJ].size.height;
						else
							dWidthJudge = roiPoints[nI].size.height;
						if (dX > dWidthJudge)
						{
							roiArror.size.width = dX;
							roiArror.size.height = dWidthJudge;
						}
						else
						{
							roiArror.size.width = dWidthJudge;
							roiArror.size.height = dX;
						}
						if ((roiArror.size.height / roiArror.size.width) >= 0.28)
						{
							//printf("装甲板高/宽=%f", roiArror.size.height / roiArror.size.width);
							if (count < 1)
							{
								roiMin = roiArror;
								count = 1;
							}
							if (roiMin.size.width > roiArror.size.width)
							{
								roiMin = roiArror;
							}
						}
					}
				}
			}
			
#ifdef DetectionContourDrawStep10
			drawRect(srcClone, roiMin, Scalar(0, 255, 255));
			line(srcClone, roiMin.center, roiMin.center, Scalar(0, 255, 0), 8, 8);
			line(srcClone, roiMin.center, Point(srcClone.cols / 2.0, srcClone.rows / 2.0), Scalar(0, 255, 0), 1, 8);
			imshow("arrorStep10Src", srcClone);
#endif // DetectionContourDrawStep10
			if(!roiMin.size.empty())
				arrorRect.push_back(roiMin);
		}
	}

#ifdef DetectionContourDrawStep10
	line(srcClone, Point(0, srcClone.rows / 2.0), Point(srcClone.cols, srcClone.rows / 2.0), Scalar(0, 255, 255), 1, 8);
	line(srcClone, Point(srcClone.cols/2.0, 0), Point(srcClone.cols/2.0, srcClone.rows), Scalar(0, 255, 255), 1, 8);
	imshow("arrorStep10Src", srcClone);
#endif //DetectionContourDrawStep10

	return 0;
}

int ArrorRecognition::finalMachineLearn(Mat src, Mat mergeBinary, vector<RotatedRect>& arrorRect, 
	vector<RotatedRect>& finalArror, int & finalNum, Ptr<SVM> svmLoad)
{
	uint dWidth;
	uint dHeight;
	uint countFlag = 0;
	Mat hogImg;
	Mat minBuffer;
	Point2f P[4];
	RotatedRect arrorBuffer;
#ifdef ImgClone	
	Mat srcClone = src.clone();
#endif // ImgClone
	for (uint nI = 0; nI < arrorRect.size(); nI++)
	{
		//初始化领域尺寸
		arrorBuffer = arrorRect[nI];
		arrorBuffer.points(P);
		P[1].x += arrorRect[nI].size.width*((1 - 2 / 3.0) / 2.0);
		P[1].y -= arrorRect[nI].size.height*((5 / 3.0 - 1) / 2.0);			//右上方移动
		dWidth = arrorRect[nI].size.width *(2 / 3.0);						//横向宽减少
		dHeight = arrorRect[nI].size.height *(5 / 3.0);					//纵向高增加

		if (P[1].x <= 0)
			P[1].x = 0;
		if (P[1].y <= 0)
			P[1].y = 0;
		if (P[1].x + dWidth >= src.cols)
			dWidth = src.cols - P[1].x;
		if (P[1].y + dHeight >= src.rows)
			dHeight = src.rows - P[1].y;

		if (P[1].x <= 0 || P[1].x > src.cols ||
			P[1].y <= 0 || P[1].y > src.rows ||
			P[1].x + dWidth >= src.cols || P[1].y + dHeight >= src.rows)//进行越界保护
		{
			continue;
		}
		if (P[1].x <= 0 || P[1].x > src.cols||
			P[1].y <= 0 || P[1].y > src.rows||
			P[1].x + dWidth >= src.cols || P[1].y + dHeight >= src.rows)//进行越界保护
		{
			continue;
		}

		//开始框选
		minBuffer = src(Rect(P[1].x, P[1].y, dWidth, dHeight));
		cvtColor(minBuffer, minBuffer, COLOR_BGR2GRAY); 
		resize(minBuffer, minBuffer, Size(svmWidth, svmHeight));//规范图像像素
		//开始处理
		Mat predictArrorNum(minBuffer.size(), CV_8UC1, Scalar(0));
		if (mlGrayOrThre == 'N')
		{
			getArrorNumBinary(minBuffer, predictArrorNum);
		}
		else if (mlGrayOrThre == 'Y')
		{
			predictArrorNum = minBuffer;
			equalizeHist(predictArrorNum, predictArrorNum);
		}
#ifdef MLStep21
		rectangle(srcClone, Point(P[1].x, P[1].y), Point(P[1].x + dWidth, P[1].y + dHeight), Scalar(255, 255, 255), 2, 8);
		line(srcClone, P[1], P[1], Scalar(0, 255, 0), 4);//确定目标物体左上顶点
		line(srcClone, arrorRect[nI].center, Point(srcClone.cols / 2.0, srcClone.rows / 2.0), Scalar(255, 255, 255), 1, 8);
		line(srcClone, Point(0, srcClone.rows / 2.0), Point(srcClone.cols, srcClone.rows / 2.0), Scalar(255, 255, 255), 1, 8);
		line(srcClone, Point(srcClone.cols / 2.0, 0), Point(srcClone.cols / 2.0, srcClone.rows), Scalar(255, 255, 255), 1, 8);
		imshow("arrorRoiStep20Src", srcClone);
		imshow("finalNum", predictArrorNum);
#endif // MLStep21
		//开始SVM预测
		hogDetector(predictArrorNum, hogImg);
		int response = (int)svmLoad->predict(hogImg);
		cout << "该数字=" << response << endl;
		if (response != 0)
		{
			countFlag++;
			if (countFlag == 1)
			{
				finalArror.push_back(arrorBuffer);
				mlArrorNum = response;
			}
			else
			{
				if (finalArror[0].size.area() < arrorBuffer.size.area())
				{
					finalArror[0] = arrorBuffer;
					mlArrorNum = response;
				}
			}
		}
#ifdef WIN10
#ifdef MLWriter
		switch (response)
		{
		case 0:fileNum++; sprintf_s(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 1:fileNum++; sprintf_s(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 2:fileNum++; sprintf_s(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 3:fileNum++; sprintf_s(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 4:fileNum++; sprintf_s(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 5:fileNum++; sprintf_s(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		}
		imwrite(stringBuffer, minBuffer);
#endif // MLWriter
#endif
#ifdef LINUX
#ifdef MLWriter
		//*
		switch (response)
		{
		case 0:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 1:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 2:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 3:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 4:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 5:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		}
		imwrite(stringBuffer, predictArrorNum);
		/*
		if(response==0)
		{
		fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum);
		imwrite(stringBuffer, predictArrorNum);
		}
		//*/
#endif
#endif // LINUX

	}
#ifdef MLStep22
	if (!finalArror.empty())
	{
		drawRect(src, finalArror[0], Scalar(0, 255, 255));
		//line(srcClone, finalArror[0].center, finalArror[0].center, Scalar(0, 255, 0), 8, 8);
		//line(srcClone, finalArror[0].center, Point(srcClone.cols / 2.0, srcClone.rows / 2.0), Scalar(0, 255, 0), 1, 8);
		imshow("arrorStep21Src", src);
	}
#endif
	return 0;
}

int ArrorRecognition::nonML(Mat src, vector<RotatedRect>& arrorRect, vector<RotatedRect>& finalArror)
{
#ifdef ImgClone	
	Mat srcClone = src.clone();
#endif // ImgClone
	if (arrorRect.empty())
		return 0;
	finalArror.push_back(arrorRect[0]);
	if (arrorRect.size() > 1)
	{
		for (uint nI = 1; nI < arrorRect.size(); nI++)
		{
			if (finalArror[0].size.area() < arrorRect[nI].size.area())
				finalArror[0]=arrorRect[nI];
		}
	}
#ifdef NonMLStep20
	drawRect(srcClone, finalArror[0], Scalar(0, 255, 255));
	imshow("arrorNoMLStep20", srcClone);
#endif // NonMLStep20

	return 0;
}
int ArrorRecognition::mlNumNonMl(Mat src, Mat mergeBinary, vector<RotatedRect>& arrorRect, vector<RotatedRect>& finalArror, int & finalNum, Ptr<SVM> svmLoad)
{
	uint dWidth;
	uint dHeight;
	uint countFlag = 0;
	Mat hogImg;
	Mat minBuffer;
	Point2f P[4];
	RotatedRect arrorBuffer;
#ifdef ImgClone	
	Mat srcClone = src.clone();
#endif // ImgClone
	for (uint nI = 0; nI < arrorRect.size(); nI++)
	{
		//初始化领域尺寸
		arrorBuffer = arrorRect[nI];
		arrorBuffer.points(P);
		P[1].x += arrorRect[nI].size.width*((1 - 2 / 3.0) / 2.0);
		P[1].y -= arrorRect[nI].size.height*((5 / 3.0 - 1) / 2.0);			//右上方移动
		dWidth = arrorRect[nI].size.width *(2 / 3.0);						//横向宽减少
		dHeight = arrorRect[nI].size.height *(5 / 3.0);					//纵向高增加

		if (P[1].x <= 0)
			P[1].x = 0;
		if (P[1].y <= 0)
			P[1].y = 0;
		if (P[1].x + dWidth >= src.cols)
			dWidth = src.cols - P[1].x;
		if (P[1].y + dHeight >= src.rows)
			dHeight = src.rows - P[1].y;

		if (P[1].x <= 0 || P[1].x > src.cols ||
			P[1].y <= 0 || P[1].y > src.rows ||
			P[1].x + dWidth >= src.cols || P[1].y + dHeight >= src.rows)//进行越界保护
		{
			continue;
		}

		//开始框选
		minBuffer = src(Rect(P[1].x, P[1].y, dWidth, dHeight));
		cvtColor(minBuffer, minBuffer, COLOR_BGR2GRAY);
		resize(minBuffer, minBuffer, Size(svmWidth, svmHeight));//规范图像像素
		//开始处理
		Mat predictArrorNum(minBuffer.size(), CV_8UC1, Scalar(0));
		if (mlGrayOrThre == 'N')
		{
			getArrorNumBinary(minBuffer, predictArrorNum);
		}
		else if (mlGrayOrThre == 'Y')
		{
			predictArrorNum = minBuffer;
			equalizeHist(predictArrorNum, predictArrorNum);
		}
#ifdef LINUX
#ifdef MLWriter
		//*
		switch (response)
		{
		case 0:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 1:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 2:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 3:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 4:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		case 5:fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum); break;
		}
		imwrite(stringBuffer, predictArrorNum);
		/*
		if(response==0)
		{
		fileNum++; sprintf(stringBuffer, "./sample/%d/%d%c%d.jpg", response, fileHeaderNum, fileHeader, fileNum);
		imwrite(stringBuffer, predictArrorNum);
		}
		//*/
#endif
#endif // LINUX
		//开始SVM预测
		hogDetector(predictArrorNum, hogImg);
		int response = (int)svmLoad->predict(hogImg);

		countFlag++;
		if (countFlag == 1)
		{
			finalArror.push_back(arrorBuffer);
			mlArrorNum = response;
		}
		else
		{
			if (finalArror[0].size.area() < arrorBuffer.size.area())
			{
				finalArror[0] = arrorBuffer;
				mlArrorNum = response;
			}
		}

	}
#ifdef NonMLStep20
	if (!finalArror.empty())
	{
		drawRect(srcClone, finalArror[0], Scalar(0, 255, 255));
		imshow("arrorNoMLStep20", srcClone);
	}
#endif // NonMLStep20
	return 0;
}
//调试函数
void ArrorRecognition::drawRect(Mat src, RotatedRect roi, Scalar color, int thickness)
{
	Point2f P[4];
	roi.points(P);//将二维点集存入图像中
	for (int j = 0; j <= 3; j++)
	{
		line(src, P[j], P[(j + 1) % 4], color, thickness);
	}
}
