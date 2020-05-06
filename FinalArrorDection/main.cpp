/*###################&&&&&&&&&&&&&&&&#############################
  #						据xml文件调参
  #				Y-89，N-78			r-114,b-98
  ###################@@@@@@@@@@@@@@@@#############################*/
#include "ArrorRecognition.h"				//程序1---装甲识别
#include "BigPinwheel.h"					//程序2---大风车
#include "ArrorAttitudeAlgorithm.h"			//姿态解算
#include "Graph.h"							//打印波形
#include <iomanip>
#include <thread>
#include <mutex>

#define SEND_SHOW

String videoPath0 = "./video/17Arror1.mp4";
String videoPath1 = "./video/19(1280x720)0.avi";
String videoPath2 = "./video/19(1280x960)01.mp4";
String videoPath3 = "./video/rBigPinwheel0.avi";
String videoPath4 = "./video/number31.avi";
String videoPath5 = "./video/video13(800)6000.avi";
String videoPath6 = "./video/bisai5(728).avi";
String videoPath7 = "./video/bisai0(800).avi";

String pinwheelPath0 = "./video/bBigPinwheel0.avi";
String pinwheelPath1 = "./video/rBigPinwheel0.avi";
String pinwheelPath2 = "./video/bBigPinwheel1.mov";
String pinwheelPath3 = "./video/rBigPinwheel1.mov";
 
VideoCapture cap(videoPath6);
VideoCapture cap0(pinwheelPath3); // 能量机关视频

int ExposureValue = 1;
void disposeImg0(VideoCapture cap);

static mutex g_mutex;
static char ReceiveData = 'z';//"1"+"2"

uchar MlFlag = 'N';
Ptr<SVM> SvmLoad ;
FileStorage fsRead; 

bool WriteFlag = false;
Mat WriterImg;

 void main() {
	
	printf("正在加载xml...\n");
	SvmLoad = StatModel::load<SVM>("./xml/svm.xml");
	fsRead.open("./xml/debugging.xml", FileStorage::READ, "utf-8");
	printf("加载完成...\n");
	fsRead["ExposureValue"] >> ExposureValue;
	printf("ExposureValue=%dus		0 \n", ExposureValue);

	thread deal0(disposeImg0,cap);		//此线程进行持续的数据处理
	waitKey(50);
#ifdef ImgWriter
	thread write(writerWriter);
#endif 

	while (1)
	{
		uchar key = getchar();
		switch (key)
		{
		case 'b':cout << "\tExiting..." << endl; exit(0); break;
		case 'z':ReceiveData = 'z'; break;
		case 'n':ReceiveData = 'n'; break;
		default:
			break;
		}
	}
}

void disposeImg0(VideoCapture cap)
{
	//$$$初始化调参参数
	//坐标系初始化
	Graph coord(800,600);
	//两函数共用区
	int finalNum = 0;
	double yaw = 0;
	double pitch = 0;
	double dist = 0;
	vector <RotatedRect> finalArror;
	ArrorAttitudeAlgorithm targetArror;
	//自瞄初始参数
	Mat mainSrc;
	Mat mainGrayBinary;
	Mat srcMask;
	vector<RotatedRect> arrorRect;
	ArrorRecognition targetCar;
	/*感兴趣搜索*/
	g_mutex.lock();
	cap >> mainSrc;
	g_mutex.unlock();
	Point2f roiPoint[4];
	uint roiWidth = mainSrc.cols;
	uint roiHeight = mainSrc.rows;
	roiPoint[1].x = 0;
	roiPoint[1].y = 0;
	Rect srcRoi = Rect(roiPoint[1].x, roiPoint[1].y, roiWidth, roiHeight);
	uint roiCounter = 0;
	//大风车初始化参数
	BigPinwheel targetPinWheel;
	Mat pinWheelSrc;
	Mat pinWheelGrayBinary;

	//读取xml参数
	/*自瞄参数*/
	fsRead["MlFlag"] >> MlFlag;
	fsRead["ImgWriter"] >> targetCar.ImgWriter;
	fsRead["fileHeaderNum"] >> targetCar.fileHeaderNum;
	fsRead["mainColor"] >> targetCar.mainColor;
	if (targetCar.mainColor == 'b')
	{
		printf("蓝场\n");
		fsRead["bMainColorThre"] >> targetCar.mainColorThre;
		fsRead["bMainGrayThre"] >> targetCar.mainGrayThre;
		fsRead["bFocalLenth"] >> targetArror.focalLenth;
	}
	else if (targetCar.mainColor == 'r')
	{
		printf("红场\n");
		fsRead["rMainColorThre"] >> targetCar.mainColorThre;
		fsRead["rMainGrayThre"] >> targetCar.mainGrayThre;
		fsRead["rFocalLenth"] >> targetArror.focalLenth;
	}
	fsRead["actualDist"] >> targetArror.actualDist;
	fsRead["ledMinArea"] >> targetCar.ledMinArea;
	fsRead["ledMaxArea"] >> targetCar.ledMaxArea;
	fsRead["ledMinAngle"] >> targetCar.ledMinAngle;
	fsRead["ledMaxAngle"] >> targetCar.ledMaxAngle;
	fsRead["ledWitdhHeightRatio"] >> targetCar.ledWitdhHeightRatio;

	fsRead["arrorMinDiffX"] >> targetCar.arrorMinDiffX;
	fsRead["arrorMaxDiffX"] >> targetCar.arrorMaxDiffX;
	fsRead["arrorMaxDiffY"] >> targetCar.arrorMaxDiffY;
	fsRead["arrorMinAngle"] >> targetCar.arrorMinAngle;
	fsRead["arrorMaxAngle"] >> targetCar.arrorMaxAngle;
/*
	cout << "自瞄参数:\n";
	cout << "ImgWriter=" << targetCar.ImgWriter << "\t\t\t"
		<<"fileHeaderNum="<<targetCar.fileHeaderNum<<"\t\t\t"

		<< "mainColor=" << targetCar.mainColor << "\t\t\t" << endl

		<< "mainColorThre=" << targetCar.mainColorThre << "\t\t"
		<< "mainGrayThre=" << targetCar.mainGrayThre << "\t\t" 

		<< "neighborGrayThre=" << targetCar.neighborGrayThre << "\t\t\t" << endl

		<< "actualDist=" << targetArror.actualDist << "\t\t\t"
		<< "focalLenth=" << targetArror.focalLenth << "\t\t\t"

		<< "ledMinArea=" << targetCar.ledMinArea << "\t\t\t" << endl
		<< "ledMaxArea=" << targetCar.ledMaxArea << "\t\t\t"
	    << targetCar.ledMinAngle << "\t\t\t"
		<< "ledMaxAngle=" << targetCar.ledMaxAngle << "\t\t\t" << endl
		<< "ledWitdhHeightRatio=" << targetCar.ledWitdhHeightRatio << "\t"

		<< "arrorMinDiffX=" << targetCar.arrorMinDiffX << "\t"
		<< "arrorMaxDiffX=" << targetCar.arrorMaxDiffX << "\t" << endl
		<< "arrorMaxDiffY=" << targetCar.arrorMaxDiffY << "\t\t"
		<< "arrorMinAngle=" << targetCar.arrorMinAngle << "\t\t"
		<< "arrorMaxAngle=" << targetCar.arrorMaxAngle << "\t\t" << endl;
//*/
	/*大风车参数*/
	fsRead["pinWheelGrayThre"] >> targetPinWheel.pinWheelGrayThre;
	fsRead["flabellumMinArea"] >> targetPinWheel.flabellumMinArea;
	fsRead["centerRadiusMin"] >> targetPinWheel.centerRadiusMin;
	fsRead["centerRadiusMax"] >> targetPinWheel.centerRadiusMax;
	fsRead["centerAreaMin"] >> targetPinWheel.centerAreaMin;
	fsRead["centerLengthWidthRatioMin"] >> targetPinWheel.centerLengthWidthRatioMin;
	fsRead["antiAngleDividend"] >> targetPinWheel.antiAngleDividend;
	fsRead["obeyAngleDividend"] >> targetPinWheel.obeyAngleDividend;

/*
	cout << "大风车参数:\n";
	cout << "pinWheelGrayThre=" << targetPinWheel.pinWheelGrayThre << "\t\t\t"
		<< "flabellumMinArea=" << targetPinWheel.flabellumMinArea << "\t\t\t"

		<< "centerRadiusMin=" << targetPinWheel.centerRadiusMin << "\t\t\t" << endl
		<< "centerRadiusMax=" << targetPinWheel.centerRadiusMax << "\t\t\t"
		<< "centerAreaMin=" << targetPinWheel.centerAreaMin << "\t\t\t"

		<< "centerLengthWidthRatioMin=" << targetPinWheel.centerLengthWidthRatioMin << "\t\t\t" << endl
		<< "antiAngleDividend=" << targetPinWheel.antiAngleDividend << "\t\t\t"
		<< "obeyAngleDividend=" << targetPinWheel.obeyAngleDividend << "\t\t\t"
		<< endl;
//*/
#ifdef PinWheelTrackbar
	namedWindow("PinWheelTrackbarDebugging", CV_WINDOW_NORMAL);
#endif // PinWheelTrackbar
#ifdef MainTrackbar
	namedWindow("Debugging", CV_WINDOW_NORMAL);
#endif
	while (1)
	{
		printf("大风车\n");
		finalArror.clear();
		targetPinWheel.clear();
		while (1)
		{
#ifdef PinWheelTrackbar
			createTrackbar("pinWheelGrayThre", "PinWheelTrackbarDebugging", &targetPinWheel.pinWheelGrayThre, 255, 0);
			createTrackbar("半径下限", "PinWheelTrackbarDebugging", &targetPinWheel.centerRadiusMin, 255, 0);
			createTrackbar("半径上限", "PinWheelTrackbarDebugging", &targetPinWheel.centerRadiusMax, 255, 0);
			createTrackbar("中心最小面积", "PinWheelTrackbarDebugging", &targetPinWheel.centerAreaMin, 300, 0);
			createTrackbar("逆预测", "PinWheelTrackbarDebugging", &targetPinWheel.antiAngleDividend, 50, 0);
			createTrackbar("顺预测", "PinWheelTrackbarDebugging", &targetPinWheel.obeyAngleDividend, 50, 0);
			uchar key = uchar(waitKey(1));
			switch (key) {
			case 'q':
				targetPinWheel.pinWheelGrayThre++; break;
			case 'w':
				targetPinWheel.pinWheelGrayThre--; break;
			case 'a':
				targetPinWheel.centerRadiusMin++; break;
			case 's':
				targetPinWheel.centerRadiusMin--; break;
			case 'z':
				targetPinWheel.centerRadiusMax++; break;
			case 'x':
				targetPinWheel.centerRadiusMax--; break;
			case 'e':
				targetPinWheel.centerAreaMin++; break;
			case 'r':
				targetPinWheel.centerAreaMin--; break;
			case 'd':
				targetPinWheel.antiAngleDividend++; break;
			case 'f':
				targetPinWheel.antiAngleDividend--; break;
			case 'c':
				targetPinWheel.obeyAngleDividend++; break;
			case 'v':
				targetPinWheel.obeyAngleDividend--; break;
			case 'p':
			case 'P':waitKey(0); break;
			case 27:
				printf("\tExiting...\n");
				exit(0);
			default:
				break;
			}
#endif
			if (ReceiveData == 'z')
			break;
			cap0 >> pinWheelSrc;
			if (pinWheelSrc.empty())
			{
				printf("0\n");
				continue;
			}
			resize(pinWheelSrc, pinWheelSrc, Size(pinWheelSrc.cols / 2, pinWheelSrc.rows / 2.0));
			/*@@@PinwheelStep0二值化*/
			//***灰度参数准备***
			Mat pinWheelGray;
			cvtColor(pinWheelSrc, pinWheelGray, COLOR_BGR2GRAY);
			//***开始二值化***
			threshold(pinWheelGray, pinWheelGrayBinary, targetPinWheel.pinWheelGrayThre, 255, CV_THRESH_BINARY);
			dilate(pinWheelGrayBinary, pinWheelGrayBinary, Mat(), Point(-1, -1), 3); //图像膨胀
			erode(pinWheelGrayBinary, pinWheelGrayBinary, Mat(), Point(-1, -1), 3);
#ifdef MainPinWheelGrayBinary
			imshow("pinWheelGrayBinary", pinWheelGrayBinary);
			waitKey(1);
#endif // MainPinWheelGrayBinary
			/*@@@PinwheelStep2找扇叶*/
			targetPinWheel.getCotyledonArror(pinWheelSrc, pinWheelGrayBinary, finalArror);
			/*@@@开始预测*/
			targetPinWheel.predictFinalArror(pinWheelSrc, finalArror);
			/*开始发送*/
			if (!finalArror.empty())
			{
				targetArror.angleSover(pinWheelSrc, finalArror[0], yaw, pitch);
				printf("\t[Yaw=%f\tPich=%f\t]\n", yaw, pitch);
			}
			else
			{
				yaw = 0;
				pitch = 0;
				printf("\t[Yaw=%f\tPich=%f\t]\n", yaw, pitch);
			}
			finalArror.clear();

			coord.coordinateSystem(yaw, pitch);
		}
		printf("自瞄\n");
		finalArror.clear();
		while (1)
		{
			if (ReceiveData == 'n')
				break;
#ifdef MainTrackbar
			createTrackbar("mainColorThre", "Debugging", &targetCar.mainColorThre, 255, 0);
			createTrackbar("mainGrayThre", "Debugging", &targetCar.mainGrayThre, 255, 0);
			createTrackbar("neighborGrayThre", "Debugging", &targetCar.neighborGrayThre, 255, 0);
			createTrackbar("ExposureValue", "Debugging", &ExposureValue, 200, 0);
			//srcRoi = Rect(0, 0, mainSrc.cols, mainSrc.rows);
#endif
			g_mutex.lock();
			cap >> mainSrc;
			g_mutex.unlock();

			if (mainSrc.empty()) {
				printf("0\n");
				continue;
			}
#ifdef ImgClone
			imshow("mainSrc", mainSrc);
			waitKey(1);
#endif
			//@@@Step0,准备ROI跟（踪颜色空间、灰度）二值图
					//***颜色空间参数准备***
			srcMask = mainSrc(srcRoi).clone();
			Mat mainColorBinary(srcMask.size(), CV_8UC1, Scalar(0));

			//***灰度参数准备***
			Mat mainGray;
			cvtColor(srcMask, mainGray, COLOR_BGR2GRAY);
			//***开始二值化***
			threshold(mainGray, mainGrayBinary, targetCar.mainGrayThre, 255, CV_THRESH_BINARY);
			dilate(mainGrayBinary, mainGrayBinary, Mat(), Point(-1, -1), 2); //图像膨胀、

			targetCar.getColorBinary(srcMask, mainColorBinary, targetCar.mainColor, targetCar.mainColorThre);

			//@@@Step1。开始颜色空间二值图和灰度二值图的轮廓查找，并进行中心灯条拟合+两两灯条拟合
			Mat mainMergeBinary(srcMask.size(), CV_8UC1, Scalar(0));
			targetCar.mergeGrayColor(mainColorBinary, mainGrayBinary, mainMergeBinary);
#ifdef MainGrayBinary
			imshow("mainGrayBinary", mainGrayBinary);
#endif
#ifdef MainColorBinary
			imshow("mainColorBinary", mainColorBinary);
#endif // MainColorBinary

#ifdef MainMergeBinary
			imshow("mainMergeBinary", mainMergeBinary);
#endif

			targetCar.detectionContour(mainSrc, mainMergeBinary, arrorRect, srcRoi);
			//@@@Step2开始进行最终装甲板的确定
			if (MlFlag == 'Y')
			{
				/*if (srcRoi.width == mainSrc.cols&&srcRoi.height == mainSrc.rows)
				{*/
					printf("机器学习\n");
					targetCar.finalMachineLearn(mainSrc, mainMergeBinary, arrorRect, finalArror, finalNum, SvmLoad);
				/*}
				else
				{
					printf("非机器学习\n");
					targetCar.nonML(mainSrc, arrorRect, finalArror);
				}*/
			}
			else
			{
				targetCar.mlNumNonMl(mainSrc, mainMergeBinary, arrorRect, finalArror, finalNum, SvmLoad);
			}

#ifdef MainTrackbar
			uchar Key = uchar(waitKey(1));
			switch (Key) {
			case 'q':
				targetCar.mainColorThre++; break;
			case 'w':
				targetCar.mainColorThre--; break;
			case 'e':
				targetCar.mainGrayThre++; break;
			case 'r':
				targetCar.mainGrayThre--; break;
			case 'a':
				targetCar.neighborGrayThre++; break;
			case 's':
				targetCar.neighborGrayThre--; break;
			case 'p':
			case 'P':waitKey(0); break;
			case 27:
				printf("\tExiting...\n");
				exit(0);
			default:
				break;
			}
			if (!finalArror.empty()) {
				dist = targetArror.angleSover(mainSrc, finalArror[0], yaw, pitch);
#ifdef SEND_SHOW
				printf("\t[Yaw=%f\tPich=%f\tdist=%f\tarrorNum=%d]\n", yaw, pitch, dist, finalNum);
#endif // SEND_SHOW
			}
			else
			{
#ifdef SEND_SHOW
				yaw = 0;
				pitch = 0;
				printf("\t[Yaw=9999\tPich=9999\tdist=9999\tarrorNum=9999]\n");
#endif // SEND_SHOW
			}

#else
			//@@@Step2，开始角度解算，发送数据
			if (!finalArror.empty()) {
				dist = targetArror.angleSover(mainSrc, finalArror[0], yaw, pitch);

				printf("\t[Yaw=%f,Pich=%f,dist=%f,arrorNum=%d]\n", yaw, pitch, dist, finalNum);

				//@@@开始感兴趣跟踪
				finalArror[0].points(roiPoint);
				roiPoint[1].x -= finalArror[0].size.width*0.5;
				roiPoint[1].y -= finalArror[0].size.height * 0.5;
				roiWidth = finalArror[0].size.width * 2;
				roiHeight = finalArror[0].size.height * 2;

				if (roiPoint[1].x <= 0 || roiPoint[1].x >= mainSrc.cols || roiPoint[1].y <= 0 || roiPoint[1].y >= mainSrc.rows || roiPoint[1].x + roiWidth >= mainSrc.cols || roiPoint[1].y + roiHeight >= mainSrc.rows)
				{
					roiPoint[1].x = 0;
					roiPoint[1].y = 0;
					roiWidth = mainSrc.cols;
					roiHeight = mainSrc.rows;
				}

				srcRoi = Rect(roiPoint[1].x, roiPoint[1].y, roiWidth, roiHeight);
#ifdef MainSrcMask
				line(mainSrc, roiPoint[1], roiPoint[1], Scalar(0, 255, 0), 4);
				rectangle(mainSrc, srcRoi, Scalar(0, 255, 0), 2, 8);
				imshow("mainSrc", mainSrc);
#endif

			}
			else
			{
				yaw = 0;
				pitch = 0;
#ifdef SEND_SHOW
				printf("\t[Yaw=9999\tPich=9999\tdist=9999\tarrorNum=9999]\n");
#endif // SEND_SHOW

#ifdef LINUX
				sendDataId0(9999, 9999, 9999, 9999);
#endif
				//@@@开始感兴趣计数
				srcRoi = Rect(0, 0, mainSrc.cols, mainSrc.rows);
			}
#endif
			coord.coordinateSystem(yaw, pitch);


			arrorRect.clear();
			finalArror.clear();
		}
	}
}