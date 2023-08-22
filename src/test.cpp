#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

using namespace cv;
using namespace std;

//Mat类
void test01()
{
	/*Mat a(3, 3, CV_8UC1);
	Mat b(Size(3, 4), CV_8UC1);
	Mat c1(5, 5, CV_8UC1, Scalar(3, 4, 5));
	Mat c2(5, 5, CV_8UC2, Scalar(3, 4, 5));
	Mat c3(5, 5, CV_8UC3, Scalar(3, 4, 5));
	Mat d = (cv::Mat_<int>(1, 5) << 1, 2, 3, 4, 5);
	Mat e = Mat::diag(d);
	Mat f = Mat(e, Range(2, 4), Range(2, 4));*/

	//Mat a = (cv::Mat_<int>(1, 5) << 1, 2, 3, 4, 5);
	//Mat c1(5, 5, CV_8UC1, Scalar(3, 4, 5));
	//Mat c2(5, 5, CV_8UC2, Scalar(3, 4, 5));
	//Mat c3(5, 5, CV_8UC3, Scalar(3, 4, 5));
	//
	//cout << a.at<int>(0, 0) << endl;
	//Vec2b vc2 = c2.at<Vec2b>(0, 1);
	//cout << vc2 << endl;
	//cout << (int)vc2[0] << endl;
	//cout << c3 << endl;
	//cout << (int)(*(c3.data + c3.step[0] * 0 + c3.step[1] * 1 + 2)) << endl;

	Mat a = (cv::Mat_<int>(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
	Mat b = (cv::Mat_<int>(3, 3) << 1, 2, 3, 7, 8, 9, 4, 5, 6);
	Mat c = (cv::Mat_<double>(3, 3) << 1.1, 2.1, 3.4, 5.2, 6.1, 1.7, 9.3, 3.2, 5.3);
	Mat d = (cv::Mat_<double>(3, 3) << 1.0, 2.0, 3.0, 5.0, 6.0, 1.0, 9.0, 3.0, 5.0);

	cout << "a = \n" << a << endl;
	cout << "b = \n" << b << endl;
	cout << "c = \n" << c << endl;
	cout << "d = \n" << d << endl; 

	cout << "a + b = \n" << a + b << endl;
	cout << "a - b = \n" << a - b << endl;
	cout << "2.0 * c = \n" << 2.0 * c << endl;
	cout << "d / 2.0 = \n" << d / 2.0 << endl;
	cout << "c * d = \n" << c * d << endl;
	cout << "c / d = \n" << c / d << endl;

	cout << "<a, b> = \n" << a.dot(b) << endl;
	cout << "a mul b = \n" << a.mul(b) << endl;

	cout << "min in a & b = \n" << min(a, b) << endl;
}
//图像读取显示与保存
void test02()
{
	Mat a = (Mat_<int>(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg", IMREAD_COLOR);
	Mat gray = imread("F:/study/code/Opencv_C++_Learn/atri.jpg", IMREAD_GRAYSCALE);

	namedWindow("color", WINDOW_AUTOSIZE);
	namedWindow("gray", WINDOW_NORMAL);

	imshow("color", img);
	imshow("gray", gray);

	//vector<int> compression_params;
	//compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	//compression_params.push_back(9);

	imwrite("F:/study/code/Opencv_C++_Learn/newatri.jpg", img);

	waitKey(0);
}
//视频加载与摄像头使用
void test03()
{
	/*VideoCapture video;
	video.open(0);
	if (!video.isOpened())
	{
		cout << "视频打开失败!" << endl;
		return;
	}


	while (true)
	{
		Mat frame;
		video >> frame;
		if (frame.empty())
			break;
		imshow("video", frame);

		double currentFrame = video.get(CAP_PROP_POS_FRAMES);
		double currentTime = video.get(CAP_PROP_POS_MSEC);
		double camera_fps = currentFrame / (currentTime / 1000);

		system("cls");
		cout << "视频帧率=" << camera_fps << endl;
		cout << "视频宽度=" << video.get(CAP_PROP_FRAME_WIDTH) << endl;

		uchar input = waitKey(1000 / camera_fps);
		if (input == 27)
			break;
	}*/

	Mat img;
	VideoCapture video(0);	//调用摄像头
							//读取视频 video.open(filename)
	if (!video.isOpened())	//判断摄像头或图像是否打开
	{
		cout << "摄像头未打开！" << endl;
		return;
	}

	video >> img;	//判断是否获取图象
	if (img.empty())
	{
		cout << "没有获取图像！" << endl;
		return;
	}

	bool isColor = (img.type() == CV_8UC3);	//判断相机是否为彩色
	VideoWriter writer;
	int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');	//选择视频编码格式

	double fps = 10.0;	//设置视频输出帧率
	string filename = "../live.avi";	//保存视频文件名

	writer.open(filename, codec, fps, img.size(), isColor);	//创建保存视频文件的视频流

	if (!writer.isOpened()) //判断视频流是否创建成功
	{
		cout << "视频流创建失败！" << endl;
		return;
	}

	while (true)
	{
		if (!video.read(img))	//判断是否能读出下一帧图像
		{
			cout << "摄像头断开连接或视频读取完毕！" << endl;
			break;
		}
		writer.write(img); //将图像写入视频流//writer << img

		imshow("live", img); //显示图像
		char input = waitKey(50);
		if (input == 27)	//按Esc保存并退出
			break;
	}

	video.release();		//关闭视频流
	writer.release();

}
//颜色空间变换
void test04()
{
	Mat img, img32;
	img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	img.convertTo(img32, CV_32F, 1.0 / 255.0, 0);

	Mat HSV, HSV32;

	cvtColor(img, HSV, COLOR_BGR2HSV);
	cvtColor(img32, HSV32, COLOR_BGR2HSV);

	Mat gray0, gray1;
	cvtColor(img, gray0, COLOR_RGB2GRAY);
	cvtColor(img, gray1, COLOR_BGR2GRAY);
}
//多通道的分离与合并
void test05()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	Mat imgs[3];
	split(img, imgs);

	Mat img0 = imgs[0]; Mat img1 = imgs[1]; Mat img2 = imgs[2];
	Mat img_H;
	merge(imgs, 3, img_H);

	Mat zero = Mat::zeros(Size(img.cols, img.rows), CV_8UC1);

	vector<Mat> imgB;
	imgB.push_back(img0); imgB.push_back(zero); imgB.push_back(zero);

	Mat imgB_H;
	merge(imgB, imgB_H);

	vector<Mat> imgG;
	imgG.push_back(zero); imgG.push_back(img1); imgG.push_back(zero);

	Mat imgG_H;
	merge(imgG, imgG_H);

	vector<Mat> imgR;
	imgR.push_back(zero); imgR.push_back(zero); imgR.push_back(img2);

	Mat imgR_H;
	merge(imgR, imgR_H);
}
//像素比较
void test06()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	Mat white = imread("F:/study/code/Opencv_C++_Learn/white.png");
	Mat black = imread("F:/study/code/Opencv_C++_Learn/black.png");

	Mat Min, Max;
	max(img, white, Min);
	min(img, black, Max);

	Mat gray, gray_black, gray_white;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	cvtColor(black, gray_black, COLOR_BGR2GRAY);
	cvtColor(white, gray_white, COLOR_BGR2GRAY);

	double minVal, maxVal;
	Point minLoc, maxLoc;

	minMaxLoc(gray, &minVal, &maxVal, &minLoc, &maxLoc, gray_black);
	minMaxLoc(gray, &minVal, &maxVal, &minLoc, &maxLoc, gray_white);
}
//逻辑操作
void test07()
{
	//Mat a = (Mat_<uchar>(1, 2) << 0, 5);
	//Mat b = (Mat_<uchar>(1, 2) << 0, 6);

	//Mat Not, And, Or, Xor;
	//
	//Not = ~a;
	//bitwise_and(a, b, And);
	//Or = a | b;
	//Xor = a ^ b;

	//cout << "a Not\n" << Not << endl;
	//cout << "a And b\n" << And << endl;
	//cout << "a Or b\n" << Or << endl;
	//cout << "a Xor b\n" << Xor << endl;

	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	Mat mark_white = imread("F:/study/code/Opencv_C++_Learn/white.png");
	Mat mark_black = imread("F:/study/code/Opencv_C++_Learn/black.png");

	Mat img_in;
	Mat result;

	//cvtColor(mark_black, mark_black, COLOR_BGR2GRAY);

	//bitwise_not(img, img_in, mark_black);
	//result = img & mark_white;
	//result = result + img_in;
	
	//Mat img_in;
	//result = ~img & mark_black;
	//img_in = img & ~mark_black;
	//img_in = result + img_in;

	result = img ^ mark_black;
}
//图像二值化
void test08()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	if (img.empty())
		exit(-1);

	imshow("atri", img);
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	Mat img_B, img_B_I, gray_B, gray_B_I, gray_T, gray_T_I, gray_TRUNC;
	//彩色图像二值化
	threshold(img, img_B, 125, 255, THRESH_BINARY);
	threshold(img, img_B_I, 125, 255, THRESH_BINARY_INV);
	imshow("atriB", img_B);
	imshow("atriBI", img_B_I);
	//灰度图像二值化
	threshold(gray, gray_B, 125, 255, THRESH_BINARY);
	threshold(gray, gray_B_I, 125, 255, THRESH_BINARY_INV);
	imshow("atriGB", gray_B);
	imshow("atriGBI", gray_B_I);
	//灰度图像TOZERO变换
	threshold(gray, gray_T, 125, 255, THRESH_TOZERO);
	threshold(gray, gray_T_I, 125, 255, THRESH_TOZERO_INV);
	imshow("atriT", gray_T);
	imshow("atriTI", gray_T_I);
	//灰度图像TRUNC变换
	threshold(gray, gray_TRUNC, 125, 255, THRESH_TRUNC);
	imshow("atriTR", gray_TRUNC);
	//灰度图像大津法和三角形法二值化
	Mat img_Thr = imread("F:/study/code/Opencv_C++_Learn/sonnet.png", IMREAD_GRAYSCALE);
	Mat img_Thr_O, img_Thr_T;
	threshold(img_Thr, img_Thr_O, 100, 255, THRESH_BINARY | THRESH_OTSU);
	threshold(img_Thr, img_Thr_T, 100, 255, THRESH_BINARY | THRESH_TRIANGLE);
	imshow("img_Thr", img_Thr);
	imshow("img_Thr_O", img_Thr_O);
	imshow("img_Thr_T", img_Thr_T);
	//均值法和高斯法
	Mat adaptive_mean, adaptive_gauss;
	adaptiveThreshold(img_Thr, adaptive_mean, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 55, 0);
	adaptiveThreshold(img_Thr, adaptive_gauss, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 55, 0);
	imshow("adaptive_mean", adaptive_mean);
	imshow("adaptive_gauss", adaptive_gauss);

	waitKey(0);
}
//LUT查找表
void test09()
{
	//LUT查找表第一层
	uchar lutfirst[256] = { 0 };
	for (int i = 0; i < 256; i++)
	{
		if (i <= 100)
			lutfirst[i] = 0;
		if (i > 100 && i <= 200)
			lutfirst[i] = 100;
		if (i > 200)
			lutfirst[i] = 255;
	}
	Mat lutOne(1, 256, CV_8UC1, lutfirst);
	//LUT查找表第二层
	uchar lutsecond[256] = { 0 };
	for (int i = 0; i < 256; i++)
	{
		if (i <= 100)
			lutsecond[i] = 0;
		if (i > 100 && i <= 150)
			lutsecond[i] = 100;
		if (i > 150 && i <= 200)
			lutsecond[i] = 150;
		if (i > 200)
			lutsecond[i] = 255;
	}
	Mat lutTwo(1, 256, CV_8UC1, lutsecond);
	//LUT查找表第三层
	uchar lutthird[256] = { 0 };
	for (int i = 0; i < 256; i++)
	{
		if (i <= 100)
			lutthird[i] = 100;
		if (i > 100 && i <= 200)
			lutthird[i] = 200;
		if (i > 200)
			lutthird[i] = 255;
	}
	Mat lutThree(1, 256, CV_8UC1, lutthird);
	//三通道的LUT查找表
	vector<Mat> Mergelut;
	Mergelut.push_back(lutOne);
	Mergelut.push_back(lutTwo);
	Mergelut.push_back(lutThree);
	Mat lutTree;
	merge(Mergelut, lutTree);
	//计算图像的查找表
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	if (img.empty())
		exit(-1);
	Mat gray, out0, out1, out2;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	LUT(gray, lutOne, out0);
	LUT(img, lutTwo, out1);
	LUT(img, lutTree, out2);
	imshow("out0", out0);
	imshow("out1", out1);
	imshow("out2", out2);

	waitKey(0);
}
//尺寸变换
void test10()
{
	Mat gray = imread("F:/study/code/Opencv_C++_Learn/atri.jpg", IMREAD_GRAYSCALE);
	Mat small, big0, big1, big2;
	resize(gray, small, Size(30, 30), 0, 0, INTER_AREA);	//缩小图像
	resize(small, big0, Size(60, 60), 0, 0, INTER_NEAREST);	//最近邻插值
	resize(small, big1, Size(60, 60), 0, 0, INTER_LINEAR);	//双线性插值
	resize(small, big2, Size(60, 60), 0, 0, INTER_CUBIC);	//双三次插值
	//图像翻转
	Mat img_x, img_y, img_xy;
	flip(gray, img_x, 0);	//x轴对称
	flip(gray, img_y, 1);	//y轴对称
	flip(gray, img_xy, -1);	//x轴对称，y轴对称
	//获取四个子图像
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	Mat img00 = img(Range(0, img.rows / 2), Range(0, img.cols / 2));
	Mat img01 = img(Range(0, img.rows / 2), Range(img.cols / 2, img.cols));
	Mat img10 = img(Range(img.rows / 2, img.rows), Range(0, img.cols / 2));
	Mat img11 = img(Range(img.rows / 2, img.rows), Range(img.cols / 2, img.cols));
	//将子图像拼接
	Mat img_, img0, img1;
	hconcat(img00, img01, img0);	//横向连接
	hconcat(img10, img11, img1);	//横向连接
	vconcat(img0, img1, img_);		//纵向连接

	waitKey(0);
}
//仿射变换
void test11()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	if (img.empty())
		exit(-1);

	Mat rotation0, img_warp0;
	double angle = 30.0;	//设置旋转角度
	Size dst_size(img.cols, img.rows);	//设置设置输出图像大小
	Point2f center(img.cols / 2.0, img.rows / 2.0);	//设置旋转中心
	rotation0 = getRotationMatrix2D(center, angle, 0.5);	//获得仿射变换矩阵
	warpAffine(img, img_warp0, rotation0, dst_size);	//对原图像进行仿射变换
	imshow("img_warp0", img_warp0);

	Point2f src_points[3], dst_points[3];
	//原图像三个点
	src_points[0] = Point2f(0, 0);
	src_points[1] = Point2f(0, img.rows - 1);
	src_points[2] = Point2f(img.cols - 1, img.rows - 1);
	//变换后图像对应的三个点
	dst_points[0] = Point2f(img.cols * 0.71, img.rows * 0.21);
	dst_points[1] = Point2f(img.cols * 0.12, img.rows * 0.01);
	dst_points[2] = Point2f(img.cols * 0.19, img.rows * 0.85);

	Mat rotation1, img_warp1;
	rotation1 = getAffineTransform(src_points, dst_points);	//求得放射变换矩阵
	warpAffine(img, img_warp1, rotation1, dst_size);	//对图像进行仿射变换

	imshow("img_warp1", img_warp1);
	waitKey(0);
}
//透视变换
void test12()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/IMG_20230810_105155.jpg");
	if (img.empty())
		exit(-1);
	
	Point2f src_points[4], dst_points[4];
	//获取原图像中物体四点坐标
	src_points[0] = Point2f(374.0, 555.0);
	src_points[1] = Point2f(2038.0, 587.0);
	src_points[2] = Point2f(56.0, 1361.0);
	src_points[3] = Point2f(2277.0, 1437.0);
	//设置变换后对应四点的坐标
	dst_points[0] = Point2f(0, 0);
	dst_points[1] = Point2f(1000.0, 0);
	dst_points[2] = Point2f(0, 600.0);
	dst_points[3] = Point2f(1000.0, 600.0);
	
	Mat rotation, img_warp;
	//获取透视变换矩阵以及对图像进行透视变换
	rotation = getPerspectiveTransform(src_points, dst_points);
	warpPerspective(img, img_warp, rotation, Size(1000, 600));

	namedWindow("img", WINDOW_NORMAL);
	imshow("img", img);
	imshow("img_warp", img_warp);

	waitKey(0);
}
//绘制基础图形
void test13()
{
	Mat img = Mat::zeros(Size(512, 512), CV_8UC3);	//生成黑色窗口用于绘制图像
	//绘制圆形
	circle(img, Point(21, 31), 15, Scalar(255, 0, 0), -1);	//实心圆
	circle(img, Point(62, 31), 20, Scalar(255, 255, 0), 3); //空心圆
	//绘制直线
	line(img, Point(23, 500), Point(450, 90), Scalar(0, 0, 255), 2, LINE_4, 0);
	//绘制椭圆
	ellipse(img, Point(200, 300), Size(70, 100), 0, 0, 360, Scalar(0, 255, 0)); //实心椭圆
	//绘制矩形
	rectangle(img, Point(100, 150), Point(200, 300), Scalar(0, 255, 255));	//矩形
	//绘制多边形
	Point pp[2][6];
	pp[0][0] = Point(72, 200);
	pp[0][1] = Point(142, 204);
	pp[0][2] = Point(226, 263);
	pp[0][3] = Point(172, 310);
	pp[0][4] = Point(117, 319);
	pp[0][5] = Point(15, 260);
	pp[1][0] = Point(359, 339);
	pp[1][1] = Point(447, 351);
	pp[1][2] = Point(504, 349);
	pp[1][3] = Point(484, 433);
	pp[1][4] = Point(418, 449);
	pp[1][5] = Point(354, 402);
	Point pp2[5];
	pp2[0] = Point(350, 83);
	pp2[1] = Point(463, 90);
	pp2[2] = Point(500, 171);
	pp2[3] = Point(421, 194);
	pp2[4] = Point(338, 141);

	const Point* pts[3] = { pp[0], pp[1], pp2 };	//pts变量生成
	int npts[] = { 6, 6, 5 };	//顶点个数生成
	fillPoly(img, pts, npts, 3, Scalar(125, 125, 125), 8);	//绘制三个多边形
	
	//输出文本
	putText(img, "LearnOpenCV", Point(100, 400), 2, 1, Scalar(255, 200, 124));

	//输出图像
	imshow("", img);
	waitKey(0);
}
//ROI区域截取
void test14()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	Mat noobcv = imread("F:/study/code/Opencv_C++_Learn/black.png");

	Mat ROI1, ROI2, ROI2_copy, mask, img2, img_copy;
	resize(noobcv, mask, Size(200, 200));
	img2 = img;	//浅拷贝
	//深拷贝
	img.copyTo(img_copy);
	//两种图中截取ROI区域的方式
	Rect rect(206, 206, 200, 200);	//定义ROI区域
	ROI1 = img(rect);	//截图

	ROI2 = img(Range(300, 500), Range(300, 500));	//第二种截图方式
	img(Range(300, 500), Range(300, 500)).copyTo(ROI2_copy);	//深拷贝

	mask.copyTo(ROI1);	//在图像中加入部分图像

	imshow("加入noobcv后图像", img);
	imshow("深拷贝的img_copy", img_copy);
	imshow("ROI1对ROI2的影响", ROI2);
	imshow("深拷贝的ROI2", ROI2_copy);

	circle(img, Point(300, 200), 20, Scalar(0, 0, 255), -1);	//绘制一个圆形

	imshow("浅拷贝的img2", img2);
	imshow("画圆对ROI1的影响", ROI1);
	waitKey(0);
}
//高斯&拉普拉斯图像金字塔
void test15()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	vector<Mat> Gauss;
	Gauss.push_back(img);
	
	int i = 0; int level = 3;
	for (i = 0; i < level; i++)
	{
		Mat gauss;
		pyrDown(Gauss[i], gauss);
		Gauss.push_back(gauss);
	}

	vector<Mat> Laplace;
	for (i = Gauss.size() - 1; i > 0; i--)
	{
		Mat laplace, Upgauss;
		if (i == Gauss.size() - 1)
		{
			Mat Downgauss;
			pyrDown(Gauss[i], Downgauss);
			pyrUp(Downgauss, Upgauss, Gauss[i].size());
			laplace = Gauss[i] - Upgauss;
			Laplace.push_back(laplace);
		}
		pyrUp(Gauss[i], Upgauss, Gauss[(size_t)i - 1].size());
		laplace = Gauss[(size_t)i - 1] - Upgauss;
		Laplace.push_back(laplace);
	}

	for (i = 0; i <= level; i++)
	{
		string name = (string)("leve") + to_string(i);
		imshow((string)"G" + name, Gauss[i]);
		imshow((string)"L" + name, Laplace[level - i]);
	}
	waitKey(0);
}
//滑动条
Mat img, imgPoint;	//全局图像
void callBack(int value, void*);

void test16()
{
	img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	namedWindow("img", WINDOW_NORMAL);
	imshow("img", img);

	int value = 100;
	createTrackbar("百分比", "img", &value, 600, callBack);
	waitKey(0);
}
//鼠标响应
Point prePoint;	//前一刻鼠标坐标，用于绘制图像
void mouse(int event, int x, int y, int flags, void*);

void test17()
{
	img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	img.copyTo(imgPoint);
	namedWindow("窗口1", WINDOW_NORMAL);
	namedWindow("窗口2", WINDOW_NORMAL);
	imshow("窗口1", img);
	imshow("窗口2", imgPoint);

	setMouseCallback("窗口1", mouse, 0);	//鼠标影响
	waitKey(0);
}
//直方图绘制
void drawHist(Mat& gray, Mat& hist, string name);

void test18()
{
	Mat gray = imread("F:/study/code/Opencv_C++_Learn/atri.jpg", IMREAD_GRAYSCALE);
	Mat hist;	//用于存放直方图统计结果
	const int channels[1] = { 0 };	//图像的通道数
	const int bins[256] = { 256 };	//直方图的维度即图像的灰度最大值
	float inRanges[2] = { 0, 256 };	//像素灰度范围
	const float* ranges[1] = { inRanges };	//每一张图片的灰度范围
	calcHist(&gray, 1, channels, Mat(), hist, 1, bins, ranges);	//计算图像灰度直方图
	drawHist(gray, hist, "gray");
	waitKey(0);
}
//直方图均衡化
void test19()
{
	Mat gray = imread("F:/study/code/Opencv_C++_Learn/lena.jpg", IMREAD_GRAYSCALE);
	Mat hist, eqhist;	//用于存放直方图统计结果
	Mat equalgray;	//存放均衡化后的灰度图像
	equalizeHist(gray, equalgray);
	const int channels[1] = { 0 };	//图像的通道数
	const int bins[256] = { 256 };	//直方图的维度即图像的灰度最大值
	float inRanges[2] = { 0, 256 };	//像素灰度范围
	const float* ranges[1] = { inRanges };	//每一张图片的灰度范围
	calcHist(&gray, 1, channels, Mat(), hist, 1, bins, ranges);	//计算图像灰度直方图
	calcHist(&equalgray, 1, channels, Mat(), eqhist, 1, bins, ranges);	//计算均衡化后图像灰度直方图
	drawHist(gray, hist, "gray");
	drawHist(equalgray, eqhist, "equalgray");
	imwrite("F:/study/code/Opencv_C++_Learn/eqlena.jpg", equalgray);
	waitKey(0);
}
//直方图匹配
void test20()
{
	Mat gray1 = imread("F:/study/code/Opencv_C++_Learn/R-C.jpg");
	Mat gray2 = imread("F:/study/code/Opencv_C++_Learn/eqlena.jpg");
	Mat hist1, hist2;
	const int channels[1] = { 0 };
	const int* bins = new int[256] { 256 };
	float inRanges[2] = { 0, 256 };
	const float* ranges[1] = { inRanges };
	calcHist(&gray1, 1, channels, Mat(), hist1, 1, bins, ranges);
	calcHist(&gray2, 1, channels, Mat(), hist2, 1, bins, ranges);
	drawHist(gray1, hist1, "gray1");
	drawHist(gray2, hist2, "gray2");
	//构建每个图像的累计概率矩阵
	float* hist1_cdf = new float[256] { hist1.at<float>(0) };
	float* hist2_cdf = new float[256] { hist2.at<float>(0) };
	for (int i = 1; i < 256; i++)
	{
		hist1_cdf[i] = hist1_cdf[i - 1] + hist1.at<float>(i);
		hist2_cdf[i] = hist2_cdf[i - 1] + hist2.at<float>(i);
	}
	//构建累积概率差值矩阵
	float(*diff_cdf)[256] = new float[256][256]{ 0 };
	for (int i = 0; i < 256; i++)
	{
		for (int j = 0; j < 256; j++)
			diff_cdf[i][j] = fabs(hist1_cdf[i] - hist2_cdf[j]);
	}
	//生成LUT映射表
	Mat lut = Mat(1, 256, CV_8U);
	for (int i = 0; i < 256; i++)
	{
		int min = diff_cdf[i][0];
		int idx = 0;
		for (int j = 0; j < 256; j++)
		{
			if (min > diff_cdf[i][j])
			{
				min = diff_cdf[i][j];
				idx = j;
			}
		}
		lut.at<uchar>(i) = (uchar)idx;
	}
	Mat result, hist3;
	LUT(gray1, lut, result);
	calcHist(&result, 1, channels, Mat(), hist3, 1, bins, ranges);
	drawHist(result, hist3, "result");

	waitKey(0);
	delete[] bins;
	delete[] hist1_cdf;
	delete[] hist2_cdf;
	delete[] diff_cdf;
}
//模板匹配
void test21()
{
	Mat img_ = imread("F:/study/code/Opencv_C++_Learn/lena.jpg");
	Mat temp = imread("F:/study/code/Opencv_C++_Learn/lenaface.jpg");
	Mat result;
	matchTemplate(img_, temp, result, TM_CCOEFF_NORMED);

	Point maxloc, minloc;
	double maxval, minval;
	minMaxLoc(result, &minval, &maxval, &minloc, &maxloc);

	rectangle(img_, maxloc, Point(maxloc.x + temp.cols, maxloc.y + temp.rows), Scalar(0, 255, 255), 2);

	imshow("img_", img_);
	imshow("temp", temp);
	imshow("result", result);
	waitKey(0);
}
//图像的卷积
void test22()
{
	uchar points[25] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
						16, 17, 18, 19, 20, 21, 22, 23, 24, 25 };
	//待卷积的矩阵
	Mat matrix = Mat(5, 5, CV_8UC1, points);
	//卷积模板
	Mat kernal = (Mat_<float>(3, 3) << 1, 2, 1,
		2, 0, 2,
		1, 2, 1);
	//归一化
	Mat kernal_norm = kernal / 12.0;
	Mat result, result_norm;
	//对矩阵进行卷积
	filter2D(matrix, result, CV_32F, kernal, Point(-1, -1), 2, BORDER_CONSTANT);
	filter2D(matrix, result_norm, CV_32F, kernal_norm, Point(-1, -1), 2, BORDER_CONSTANT);
	//输出矩阵
	cout << "result:>\n" << result << endl;
	cout << "result_norm:>\n" << result_norm << endl;
	//对图像进行卷积
	Mat lena = imread("../lena.jpg");
	if (lena.empty())
		exit(-1);
	Mat lena_filter;
	filter2D(lena, lena_filter, -1, kernal_norm, Point(-1, -1), 2, BORDER_CONSTANT);
	imshow("lena", lena);
	imshow("lena_filter", lena_filter);
	waitKey(0);
}
//椒盐噪声和高斯噪声
void saltAndPepper(Mat& image, int n);

void test23()
{
	Mat lena = imread("../lena.jpg");
	Mat eqlena = imread("../eqlena.jpg", IMREAD_ANYDEPTH);
	if (lena.empty() || eqlena.empty())
		exit(-1);
	
	Mat lena_G, eqlena_G;
	lena.copyTo(lena_G);
	eqlena.copyTo(eqlena_G);
	
	imshow("原图1", lena);
	imshow("原图2", eqlena);
	//添加椒盐噪声
	saltAndPepper(lena, 10000);
	saltAndPepper(eqlena, 10000);
	//显示处理后的图像
	imshow("椒盐噪声1", lena);
	imshow("椒盐噪声2", eqlena);
	waitKey(0);
	//高斯噪声
	Mat lena_noise = Mat::zeros(lena_G.rows, lena_G.cols, lena_G.type());
	Mat eqlena_noise = Mat::zeros(eqlena_G.rows, eqlena_G.cols, eqlena_G.type());
	
	imshow("原图1", lena_G);
	imshow("原图2", eqlena_G);
	RNG rng;	//创建rng类
	rng.fill(lena_noise, RNG::NORMAL, 10, 20);	//生成三通道高斯分布随机数
	rng.fill(eqlena_noise, RNG::NORMAL, 10, 20);	//生成单通道高斯分布随机数
	
	imshow("三通道高斯噪声", lena_noise);
	imshow("单通道高斯噪声", eqlena_noise);

	lena_G = lena_G + lena_noise; //在彩色图像中添加高斯噪声
	eqlena_G = eqlena_G + eqlena_noise;	//在灰色图像中添加高斯噪声

	imshow("高斯噪声1", lena_G);
	imshow("高斯噪声2", eqlena_G);

	waitKey(0);

	imwrite("../RGBSaltNoise.jpg", lena);
	imwrite("../GRAYSaltNoise.jpg", eqlena);
	imwrite("../RGBGaussNoise.jpg", lena_G);
	imwrite("../GRAYGaussNoise.jpg", eqlena_G);
}

void test24()
{
	Mat eqlena = imread("../eqlena.jpg", IMREAD_ANYDEPTH);
	Mat eqlena_salt = imread("../GRAYSaltNoise.jpg", IMREAD_ANYDEPTH);
	Mat eqlena_gauss = imread("../GRAYGaussNoise.jpg", IMREAD_ANYDEPTH);
	if (eqlena.empty() || eqlena_salt.empty() || eqlena_gauss.empty())
		exit(-1);

	Mat result_3, result_5;	//存放不含有噪声滤波结果，后面数字表示滤波器尺寸
	Mat result_3salt, result_5salt;	//存放含有椒盐噪声滤波结果，后面数字表示滤波器尺寸
	Mat result_3gauss, result_5gauss;	//存放含有高斯噪声滤波结果，后面数字表示滤波器尺寸
	//运用均值滤波函数blur()进行滤波
	blur(eqlena, result_3, Size(3, 3));
	blur(eqlena, result_5, Size(5, 5));
	blur(eqlena_salt, result_3salt, Size(3, 3));
	blur(eqlena_salt, result_5salt, Size(5, 5));
	blur(eqlena_gauss, result_3gauss, Size(3, 3));
	blur(eqlena_gauss, result_5gauss, Size(5, 5));
	//显示不含噪声的图像
	imshow("eqlena", eqlena);
	imshow("result_3", result_3);
	imshow("result_5", result_5);
	//显示含椒盐噪声的图像
	imshow("eqlena_salt", eqlena_salt);
	imshow("result_3salt", result_3salt);
	imshow("result_5salt", result_5salt);
	//显示含高斯噪声的图像
	imshow("eqlena_gauss", eqlena_gauss);
	imshow("result_3gauss", result_3gauss);
	imshow("result_5gauss", result_5gauss);
	waitKey(0);
	//运用方框滤波函数处理图像
	Mat result, result_Norm;
	boxFilter(eqlena, result, -1, Size(3, 3), Point(-1, -1), false);	//不进行归一化
	boxFilter(eqlena, result_Norm, -1, Size(3, 3), Point(-1, -1), true);	//进行归一化
	//显示处理结果
	imshow("result", result);
	imshow("result_Norm", result_Norm);
	waitKey(0);
	//高斯滤波
	Mat result_3_G, result_5_G;	//存放不含有噪声滤波结果，后面数字表示滤波器尺寸
	Mat result_3_G_salt, result_5_G_salt;	//存放含有椒盐噪声滤波结果，后面数字表示滤波器尺寸
	Mat result_3_G_gauss, result_5_G_gauss;	//存放含有高斯噪声滤波结果，后面数字表示滤波器尺寸
	//高斯滤波函数
	GaussianBlur(eqlena, result_3_G, Size(3, 3), 10, 50);
	GaussianBlur(eqlena, result_5_G, Size(5, 5), 10, 20);
	GaussianBlur(eqlena_salt, result_3_G_salt, Size(3, 3), 10, 20);
	GaussianBlur(eqlena_salt, result_5_G_salt, Size(5, 5), 10, 20);
	GaussianBlur(eqlena_gauss, result_3_G_gauss, Size(3, 3), 10, 20);
	GaussianBlur(eqlena_gauss, result_5_G_gauss, Size(5, 5), 10, 20);
	//显示不含噪声的图像
	imshow("eqlena", eqlena);
	imshow("result_3_G", result_3_G);
	imshow("result_5_G", result_5_G);
	//显示含椒盐噪声的图像
	imshow("eqlena_salt", eqlena_salt);
	imshow("result_3_G_salt", result_3_G_salt);
	imshow("result_5_G_salt", result_5_G_salt);
	//显示含高斯噪声的图像
	imshow("eqlena_gauss", eqlena_gauss);
	imshow("result_3_G_gauss", result_3_G_gauss);
	imshow("result_5_G_gauss", result_5_G_gauss);
	waitKey(0);
}
//中值滤波
void test25()
{
	Mat lena = imread("../lena.jpg", IMREAD_ANYCOLOR);
	Mat eqlena = imread("../eqlena.jpg", IMREAD_ANYCOLOR);
	Mat lena_salt = imread("../RGBSaltNoise.jpg", IMREAD_ANYCOLOR);
	Mat eqlena_salt = imread("../GRAYSaltNoise.jpg", IMREAD_ANYCOLOR);
	Mat lena_gauss = imread("../RGBGaussNoise.jpg", IMREAD_ANYCOLOR);
	Mat eqlena_gauss = imread("../GRAYGaussNoise.jpg", IMREAD_ANYCOLOR);

	Mat result_3, result_5, result_3_salt, result_5_salt, result_3_gauss, result_5_gauss;
	Mat eqresult_3, eqresult_5, eqresult_3_salt, eqresult_5_salt, eqresult_3_gauss, eqresult_5_gauss;
	//中值滤波函数
	medianBlur(lena, result_3, 3);
	medianBlur(lena, result_5, 5);
	medianBlur(eqlena, eqresult_3, 3);
	medianBlur(eqlena, eqresult_5, 5);
	medianBlur(lena_salt, result_3_salt, 3);
	medianBlur(lena_salt, result_5_salt, 5);
	medianBlur(eqlena_salt, eqresult_3_salt, 3);
	medianBlur(eqlena_salt, eqresult_5_salt, 5);
	medianBlur(lena_gauss, result_3_gauss, 3);
	medianBlur(lena_gauss, result_5_gauss, 5);
	medianBlur(eqlena_gauss, eqresult_3_gauss, 3);
	medianBlur(eqlena_gauss, eqresult_5_gauss, 5);
	//彩色图片
	imshow("lena", lena);
	imshow("result_3", result_3);
	imshow("result_5", result_5);
	//灰色图片
	imshow("eqlena", eqlena);
	imshow("eqresult_3", eqresult_3);
	imshow("eqresult_5", eqresult_5);
	//含椒盐噪声的彩色图片
	imshow("lena_salt", lena_salt);
	imshow("result_3_salt", result_3_salt);
	imshow("result_5_salt", result_5_salt);
	//含椒盐噪声的灰色图片
	imshow("eqlena_salt", eqlena_salt);
	imshow("eqresult_3_salt", eqresult_3_salt);
	imshow("eqresult_5_salt", eqresult_5_salt);
	//含高斯噪声的彩色图片
	imshow("lena_gauss", lena_gauss);
	imshow("result_3_gauss", result_3_gauss);
	imshow("result_5_gauss", result_5_gauss);
	//含高斯噪声的灰色图片
	imshow("eqlena_gauss", eqlena_gauss);
	imshow("eqresult_3_gauss", eqresult_3_gauss);
	imshow("eqresult_5_gauss", eqresult_5_gauss);

	waitKey(0);
}
//滤波的叠加性
void test26()
{
	float points[25] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
						16, 17, 18, 19, 20, 21, 22, 23, 24, 25 };
	Mat matrix = Mat(5, 5, CV_32FC1, points);
	//X方向，Y方向和联合滤波
	Mat Y = (Mat_<float>(3, 1) << -1, 3, -1);
	Mat X = Y.reshape(1, 1);
	Mat XY = Y * X;
	//验证线性滤波的可分离性
	Mat dataY, dataYX, dataXY, dataXY_sep;
	filter2D(matrix, dataY, -1, Y, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(dataY, dataYX, -1, X, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(matrix, dataXY, -1, XY, Point(-1, -1), 0, BORDER_CONSTANT);

	sepFilter2D(matrix, dataXY_sep, -1, Y, Y, Point(-1, -1), 0, BORDER_CONSTANT);
	//输出滤波结果
	cout << "matrix = \n" << matrix << endl;
	cout << "dataY = \n" << dataY << endl;
	cout << "dataYX = \n" << dataYX << endl;
	cout << "dataXY = \n" << dataXY << endl;
	cout << "dataXY_sep = \n" << dataXY_sep << endl;

	//验证高斯滤波的可分离性
	Mat gaussX = getGaussianKernel(3, 1);
	Mat gaussData, gaussDataXY;
	GaussianBlur(matrix, gaussData, Size(3, 3), 1, 1, BORDER_CONSTANT);
	sepFilter2D(matrix, gaussDataXY, -1, gaussX, gaussX, Point(-1, -1), 0, BORDER_CONSTANT);
	//输出两种滤波结果
	cout << "gaussData = \n" << gaussData << endl;
	cout << "gaussDataXY = \n" << gaussDataXY << endl;

	//图像测试滤波的可分离性
	Mat lena = imread("../lena.jpg");
	Mat lenaY, lenaYX, lenaXY, lena_sep;
	if (lena.empty())
		exit(-1);

	filter2D(lena, lenaY, -1, Y, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(lenaY, lenaYX, -1, X, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(lena, lenaXY, -1, XY, Point(-1, -1), 0, BORDER_CONSTANT);
	
	sepFilter2D(lena, lena_sep, -1, Y, Y, Point(-1, -1), 0, BORDER_CONSTANT);

	imshow("lena", lena);
	imshow("lenaYX", lenaYX);
	imshow("lenaXY", lenaXY);
	imshow("lena_sep", lena_sep);
	waitKey(0);
}
//Soble和Scharr边缘检测算子
void test27()
{
	Mat eqlena = imread("../lena.jpg", IMREAD_ANYDEPTH);
	Mat resultX, resultY, resultXY;
	//Soble边缘检测
	//X边缘检测
	Sobel(eqlena, resultX, CV_16S, 1, 0, 1);
	convertScaleAbs(resultX, resultX);
	//Y边缘检测
	Sobel(eqlena, resultY, CV_16S, 0, 1, 3);
	convertScaleAbs(resultY, resultY);
	//整幅图像的边缘
	resultXY = resultX + resultY;
	//显示图像
	imshow("eqlena", eqlena);
	imshow("resultX", resultX);
	imshow("resultY", resultY);
	imshow("resultXY", resultXY);
	waitKey(0);
	//Scharr边缘检测
	//X边缘检测
	Scharr(eqlena, resultX, CV_16S, 1, 0);
	convertScaleAbs(resultX, resultX);
	//Y边缘检测
	Scharr(eqlena, resultY, CV_16S, 0, 1);
	convertScaleAbs(resultY, resultY);
	//整幅图像的边缘
	resultXY = resultX + resultY;
	//显示图像
	imshow("eqlena", eqlena);
	imshow("resultX", resultX);
	imshow("resultY", resultY);
	imshow("resultXY", resultXY);
	waitKey(0);
	//生成边缘检测器
	Mat soble_x, soble_y, soble_X1;
	Mat scharr_x, scharr_y, scharr_X1;
	//生成一阶x方向soble算子
	getDerivKernels(soble_x, soble_y, 1, 0, 3);
	soble_x = soble_x.reshape(CV_8U, 1);
	//计算滤波器
	soble_X1 = soble_y * soble_x;
	//生成一阶x方向scharr算子
	getDerivKernels(scharr_x, scharr_y, 1, 0, -1);	//-1代表生成scharr算子
	scharr_x = scharr_x.reshape(CV_8U, 1);
	//计算滤波器
	scharr_X1 = scharr_y * scharr_x;
	//输出结果
	cout << "一阶x方向soble算子:\n" << soble_X1 << endl;
	cout << "一阶x方向scharr算子:\n" << scharr_X1 << endl;
	waitKey(0);
}
//拉普拉斯和Canny边缘检测
void test28()
{
	Mat gray = imread("../GRAYGaussNoise.jpg", IMREAD_ANYDEPTH);
	Mat result, result_g, result_G;
	if (gray.empty())
		exit(-1);
	//拉普拉斯边缘检测
	//未进行滤波直接提取边缘
	Laplacian(gray, result, CV_16S, 3, 1, 0);
	convertScaleAbs(result, result);
	//滤波后提取边缘
	GaussianBlur(gray, result_g, Size(3, 3), 5, 0);
	Laplacian(result_g, result_G, CV_16S, 3, 1, 0);
	convertScaleAbs(result_G, result_G);
	//显示结果
	imshow("gray", gray);
	imshow("result", result);
	imshow("result_G", result_G);
	waitKey(0);
	//Canny边缘检测
	Mat resultHigh, resultLow, resultG;
	//大阈值检测图像边缘
	Canny(gray, resultHigh, 100, 200);
	//小阈值检测图像边缘
	Canny(gray, resultLow, 20, 40);
	//高斯模糊后检测图像边缘
	GaussianBlur(gray, resultG, Size(3, 3), 5, 0);
	Canny(resultG, resultG, 100, 200);
	//显示结果
	imshow("resulHigh", resultHigh);
	imshow("resultLow", resultLow);
	imshow("resultG", resultG);
	waitKey(0);
}
//连通域分析
void drawStatus(Mat& image, Mat& stats, Mat& centroids, int number);

void test29()
{
	Mat image = imread("../rice.jpg");
	if (image.empty())
		exit(-1);
	//将图像转换为二值图像，用于统计连通域
	Mat rice, riceB;
	cvtColor(image, rice, COLOR_BGR2GRAY);
	threshold(rice, riceB, 100, 255, THRESH_BINARY);
	//随机生成颜色用于区分不同连通域
	RNG rng((unsigned)time(NULL));
	Mat out;
	vector<Vec3b> colors;
	int number = connectedComponents(riceB, out, 8, CV_16U);	//统计图像中连通域的个数
	for (int i = 0; i < number; i++)
	{
		//用随机数确定颜色
		Vec3b v3 = Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		colors.push_back(v3);
	}
	//以不同颜色标记连通域
	Mat result = Mat::zeros(rice.size(), image.type());
	int w = result.cols, h = result.rows;
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			int label = out.at<uint16_t>(i, j);
			if (label == 0)	//背景为黑色不变
				continue;
			result.at<Vec3b>(i, j) = colors[label];
		}
	}
	//显示结果
	imshow("原图", image);
	imshow("标记后的图像", result);
	waitKey(0);
	//统计连通域的信息
	Mat stats, centroids;
	number = connectedComponentsWithStats(riceB, out, stats, centroids, 8, CV_16U);
	drawStatus(image, stats, centroids, number);
	imshow("标记后的图像", image);
	waitKey(0);
}
//图像距离变化
void test30()
{
	Mat matrix = (Mat_<uchar>(5, 5) << 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 0, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1);
	Mat dist_L1, dist_L2, dist_C;
	//计算街区距离
	distanceTransform(matrix, dist_L1, DIST_L1, 3, CV_8U);
	cout << "街区距离：\n" << dist_L1 << endl;
	//计算欧氏距离
	distanceTransform(matrix, dist_L2, DIST_L2, 5, CV_32F);
	cout << "欧氏距离：\n" << dist_L2 << endl;
	//计算棋盘距离
	distanceTransform(matrix, dist_C, DIST_C, 3, CV_8U);
	cout << "棋盘距离：\n" << dist_C << endl;
	//对图像进行距离变换
	Mat rice = imread("../rice.jpg", IMREAD_GRAYSCALE);
	Mat riceB, riceBI;
	threshold(rice, riceB, 50, 255, THRESH_BINARY);
	threshold(rice, riceBI, 50, 255, THRESH_BINARY_INV);
	//计算图像的街区距离
	Mat dist, dist_INV;
	distanceTransform(riceB, dist, 1, 3, CV_32F);
	distanceTransform(riceBI, dist_INV, 1, 3, CV_8U);

	imshow("rice", rice);
	imshow("riceB", riceB);
	imshow("riceBI", riceBI);
	imshow("dist", dist);
	imshow("dist_INV", dist_INV);
	waitKey(0);
}
//图像腐蚀操作
void test31()
{
	Mat matrix = (Mat_<uchar>(6, 6) << 0, 0, 0, 0, 255, 0,
		0, 255, 255, 255, 255, 255,
		0, 255, 255, 255, 255, 0,
		0, 255, 255, 255, 255, 0,
		0, 255, 255, 255, 255, 0,
		0, 0, 0, 0, 0, 0);
	Mat struct1, struct2;
	struct1 = getStructuringElement(0, Size(3, 3));	//矩形结构元素
	struct2 = getStructuringElement(1, Size(3, 3));	//十字结构元素
	//存放腐蚀后的结果
	Mat eroMatrix;
	erode(matrix, eroMatrix, struct2);
	namedWindow("matrix", WINDOW_GUI_NORMAL);
	namedWindow("eroMatrix", WINDOW_GUI_NORMAL);
	imshow("matrix", matrix);
	imshow("eroMatrix", eroMatrix);
	waitKey(0);
	//文字腐蚀
	Mat learn = imread("../word.png", IMREAD_ANYCOLOR);
	Mat ero1, ero2;
	erode(learn, ero1, struct1);
	erode(learn, ero2, struct2);
	imshow("learn", learn);
	imshow("ero1", ero1);
	imshow("ero2", ero2);
	waitKey(0);
	//图像腐蚀
	Mat rice = imread("../rice.jpg");
	Mat gray;
	//将原图片转换为灰色图片
	cvtColor(rice, gray, COLOR_BGR2GRAY);
	//两份存放二值化图片，拷贝一份后期绘图的图片以及创建一份存放腐蚀图像
	Mat riceB, riceBW, rice_copy, eroRice;
	rice.copyTo(rice_copy);
	//腐蚀原图像
	erode(gray, eroRice, struct2);
	//将原图像二值化
	threshold(gray, riceB, 30, 255, THRESH_BINARY);
	Mat out, stats, centroids;
	int number = connectedComponentsWithStats(riceB, out, stats, centroids);
	drawStatus(rice, stats, centroids, number);
	imshow("rice", rice);
	threshold(eroRice, riceBW, 30, 255, THRESH_BINARY);
	number = connectedComponentsWithStats(riceBW, out, stats, centroids);
	drawStatus(rice_copy, stats, centroids, number);
	imshow("rice_copy", rice_copy);
	waitKey(0);
}
//图像膨胀操作
void test32()
{
	Mat matrix = (Mat_<uchar>(6, 6) << 0, 0, 0, 0, 255, 0,
		0, 255, 255, 255, 255, 255,
		0, 255, 255, 255, 255, 0,
		0, 255, 255, 255, 255, 0,
		0, 255, 255, 255, 255, 0,
		0, 0, 0, 0, 0, 0);
	Mat struct1, struct2;
	struct1 = getStructuringElement(0, Size(3, 3));	//矩形结构元素
	struct2 = getStructuringElement(1, Size(3, 3));	//十字结构元素
	//存放膨胀后的结果
	Mat eroMatrix;
	dilate(matrix, eroMatrix, struct2);
	namedWindow("matrix", WINDOW_GUI_NORMAL);
	namedWindow("eroMatrix", WINDOW_GUI_NORMAL);
	imshow("matrix", matrix);
	imshow("eroMatrix", eroMatrix);
	waitKey(0);
	//文字膨胀
	Mat learn = imread("../word.png", IMREAD_ANYCOLOR);
	Mat ero1, ero2;
	dilate(learn, ero1, struct1);
	dilate(learn, ero2, struct2);
	imshow("learn", learn);
	imshow("ero1", ero1);
	imshow("ero2", ero2);
	waitKey(0);
}
//对图像进行形态学操作
void test33()
{
	//用于验证形态学应用的二值化矩阵
	Mat matrix = (Mat_<uchar>(9, 12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 255, 255, 255, 255, 255, 255, 255, 0, 0, 255, 0,
		0, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0,
		0, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0,
		0, 255, 255, 255, 0, 255, 255, 255, 0, 0, 0, 0,
		0, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0,
		0, 255, 255, 255, 255, 255, 255, 255, 0, 0, 255, 0,
		0, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	namedWindow("matrix", WINDOW_GUI_NORMAL);
	imshow("matrix", matrix);
	//3 * 3的矩形结构元素
	Mat kernal = getStructuringElement(0, Size(3, 3));
	Mat kernelHit = (Mat_<uchar>(3, 3) << 0, 1, 0, 1, -1, 1, 0, 1, 0);
	//对矩阵进行形态学操作
	Mat open, close, gradient, tophat, blackhat, hitmiss;
	//开运算
	morphologyEx(matrix, open, MORPH_OPEN, kernal);
	namedWindow("open", WINDOW_GUI_NORMAL);
	imshow("open", open);
	//闭运算
	morphologyEx(matrix, close, MORPH_CLOSE, kernal);
	namedWindow("close", WINDOW_GUI_NORMAL);
	imshow("close", close);
	//梯度运算
	morphologyEx(matrix, gradient, MORPH_GRADIENT, kernal);
	namedWindow("gradient", WINDOW_GUI_NORMAL);
	imshow("gradient", gradient);
	//顶帽运算
	morphologyEx(matrix, tophat, MORPH_TOPHAT, kernal);
	namedWindow("tophat", WINDOW_GUI_NORMAL);
	imshow("tophat", tophat);
	//黑帽运算
	morphologyEx(matrix, blackhat, MORPH_BLACKHAT, kernal);
	namedWindow("blackhat", WINDOW_GUI_NORMAL);
	imshow("blackhat", blackhat);
	//集中击不中运算
	morphologyEx(matrix, hitmiss, MORPH_HITMISS, kernelHit);
	namedWindow("hitmiss", WINDOW_GUI_NORMAL);
	imshow("hitmiss", hitmiss);
	waitKey(0);
	//对图片进行形态学操作
	Mat key = imread("../key.jpg", IMREAD_GRAYSCALE);
	//输出原图像
	imshow("key", key);
	Mat keyB;
	threshold(key, keyB, 100, 255, THRESH_BINARY);
	//输出二值化后的图像
	imshow("keyB", keyB);
	//5 * 5矩阵结构元素
	Mat kernal_ = getStructuringElement(0, Size(5, 5));
	Mat open_key, close_key, gradient_key, tophat_key, blackhat_key, hitmiss_key;
	//对图像进行运算
	morphologyEx(keyB, open_key, MORPH_OPEN, kernal_);
	imshow("open_key", open_key);
	morphologyEx(keyB, close_key, MORPH_CLOSE, kernal_);
	imshow("close_key", close_key);
	morphologyEx(keyB, gradient_key, MORPH_GRADIENT, kernal_);
	imshow("gradient_key", gradient_key);
	morphologyEx(keyB, tophat_key, MORPH_TOPHAT, kernal_);
	imshow("tophat_key", tophat_key);
	morphologyEx(keyB, blackhat_key, MORPH_BLACKHAT, kernal_);
	imshow("blackhat_key", blackhat_key);
	morphologyEx(keyB, hitmiss_key, MORPH_HITMISS, kernelHit);
	imshow("hitmiss_key", hitmiss_key);

	waitKey(0);
}
//图像细化
void test34()
{
	Mat word = imread("../word.png", IMREAD_GRAYSCALE);
	if (word.empty())
		exit(-1);
	//绘制英文和实心圆和圆环的图片
	Mat words = Mat::zeros(100, 200, CV_8UC1);
	putText(words, "Learn", Point(30, 30), 2, 1, Scalar(255), 2);
	putText(words, "OpenCV", Point(30, 60), 2, 1, Scalar(255), 2);
	circle(words, Point(80, 75), 10, Scalar(255), -1);
	circle(words, Point(130, 75), 10, Scalar(255),3);
	//进行细化
	Mat thin1, thin2;
	ximgproc::thinning(word, thin1, 0);
	ximgproc::thinning(words, thin2, 0);
	//显示结果
	namedWindow("words", WINDOW_NORMAL);
	namedWindow("thin2", WINDOW_NORMAL);
	imshow("word", word);
	imshow("thin1", thin1);
	imshow("words", words);
	imshow("thin2", thin2);
	waitKey(0);
}
//轮廓检测
void test35()
{
	Mat image = imread("../key.jpg");
	if (image.empty())
		exit(-1);
	imshow("原图", image);
	Mat gray, bin;
	cvtColor(image, gray, COLOR_BGR2GRAY);	//转换为灰度图像
	GaussianBlur(gray, gray, Size(5, 5), 4, 4);	//平滑滤波
	threshold(gray, bin, 170, 255, THRESH_BINARY | THRESH_OTSU);	//自适应二值化

	//轮廓检测与绘制
	vector<vector<Point>> contours;	//轮廓
	vector<Vec4i> hierarchy;	//存放轮廓结构变量

	findContours(bin, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	//绘制轮廓
	for (int i = 0; i < hierarchy.size(); i++)
	{
		cout << hierarchy[i] << endl;
 	}
	for (int t = 0; t < contours.size(); t++)
	{
		drawContours(image, contours, t, Scalar(0, 0, 255), 2, 8);
		imshow("绘制结果", image);
		waitKey(0);
	}
}
//轮廓信息统计
void test36()
{
	vector<Point> contour;
	contour.push_back(Point(0, 0));
	contour.push_back(Point(10, 0));
	contour.push_back(Point(10, 10));
	contour.push_back(Point(5, 5));
	//三角形面积
	double area = contourArea(contour);
	cout << "三角形面积 = " << area << endl;
	//三角形周长
	double length1 = arcLength(contour, false);
	double length2 = arcLength(contour, true);
	cout << "三角形周长length1 = " << length1 << endl;
	cout << "三角形周长length2 = " << length2 << endl;
	//图像轮廓面积
	Mat key = imread("../key.jpg", IMREAD_GRAYSCALE);
	GaussianBlur(key, key, Size(9, 9), 5, 5);
	threshold(key, key, 170, 255, THRESH_BINARY | THRESH_OTSU);
	//获得轮廓
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(key, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	//计算并输出各个轮廓的面积和周长
	for (int i = 0; i < hierarchy.size(); i++)
	{
		cout << hierarchy[i] << endl;
	}
	for (int i = 0; i < contours.size(); i++)
	{
		cout << "第" << i << "个轮廓的面积 = " << contourArea(contours[i]) << endl;
		cout << "第" << i << "个轮廓的周长 = " << arcLength(contours[i], true) << endl;
	}
}
//轮廓外接多边形
void drawapp(Mat& input, Mat& image);

void test37()
{
	Mat image = imread("../con.jpg");
	if (image.empty())
		exit(-1);
	imshow("image", image);
	//深拷贝图像用于绘制结果
	Mat image1, image2;
	image.copyTo(image1);	//绘制最大外接矩阵
	image.copyTo(image2);	//绘制最小外接矩阵
	//去噪声和二值化
	Mat canny1;
	Canny(image, canny1, 200, 240);
	//膨胀运算，将细小缝隙填补
	Mat kernel = getStructuringElement(0, Size(9, 9));
	dilate(canny1, canny1, kernel);
	//轮廓检测与绘制
	vector<vector<Point>> contours1;
	vector<Vec4i> hierarchy1;
	findContours(canny1, contours1, hierarchy1, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	//寻找轮廓外接矩形
	for (int i = 0; i < contours1.size(); i++)
	{
		//绘制最大外接矩形
		Rect rect = boundingRect(contours1[i]);
		rectangle(image1, rect, Scalar(0, 255, 0), 2, 8, 0);
		//绘制最小外接矩形
		RotatedRect rrect = minAreaRect(contours1[i]);
		Point2f points[4];
		rrect.points(points);	//获得最小矩形的四个点
		Point2f center = rrect.center;	//获得最小外接矩形的中心
		//绘制矩形
		for (int p = 0; p < 4; p++)
		{
			line(image2, points[p], points[(p + 1) % 4], Scalar(0, 255, 0), 2, 8, 0);	//连接四个点
		}
		////绘制矩形中心
		circle(image2, center, 4, Scalar(0, 255, 0), -1, 8, 0);
	}
	//输出结果
	namedWindow("max", WINDOW_GUI_NORMAL);
	namedWindow("min", WINDOW_GUI_NORMAL);
	imshow("max", image1);
	imshow("min", image2);
	waitKey(0);
	//多边形拟合
	Mat approx = imread("../angle.png");
	imshow("原图", approx);
	Mat canny2;
	//去噪声和二值化
	Canny(approx, canny2, 100, 160, 3, false);
	//膨胀运算
	Mat kernel_ = getStructuringElement(0, Size(3, 3));
	dilate(canny2, canny2, kernel_);
	//轮廓检测
	vector<vector<Point>> contours2;
	vector<Vec4i> hierarchy2;
	findContours(canny2, contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	//绘制多边形
	for (int i = 0; i < contours2.size(); i++)
	{
		//利用最小矩形求得几何中心
		RotatedRect rrect = minAreaRect(contours2[i]);
		Point2f center = rrect.center;
		circle(approx, center, 2, Scalar(0, 0, 255), 2, 8, 0);
		//绘制多边形
		Mat result;
		approxPolyDP(contours2[i], result, 4, true);
		drawapp(result, approx);
		//判断形状
		cout << "corners: " << result.rows << endl;
		if (result.rows == 3)
		{
			putText(approx, "triangle", center, 0, 1, Scalar(0, 0, 255), 2, 8, false);
		}
		if (result.rows == 4)
		{
			putText(approx, "rectangle", center, 0, 1, Scalar(0, 0, 255), 2, 8, false);
		}
		if (result.rows == 6)
		{
			putText(approx, "ploy-6", center, 0, 1, Scalar(0, 0, 255), 2, 8, false);
		}
		if (result.rows == 8)
		{
			putText(approx, "ploy-8", center, 0, 1, Scalar(0, 0, 255), 2, 8, false);
		}
		if (result.rows > 12)
		{
			putText(approx, "circle", center, 0, 1, Scalar(0, 0, 255), 2, 8, false);
		}
	}
	//显示结果
	imshow("approx", approx);
	waitKey(0);
}
//凸包检测
void test38()
{
	Mat image = imread("../hand.jpg", IMREAD_ANYCOLOR);
	if (image.empty())
		exit(-1);
	//图像二值化
	Mat gray, bin;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	threshold(gray, bin, 150, 255, THRESH_BINARY_INV);
	//图像开运算去除噪声
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	morphologyEx(bin, bin, MORPH_OPEN, kernel);
	namedWindow("bin", WINDOW_NORMAL);
	imshow("bin", bin);
	//获取轮廓
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(bin, contours, hierarchy, 0, 2, Point());
	for (int i = 0; i < contours.size(); i++)
	{
		//计算凸包顶点
		vector<Point> hull;
		convexHull(contours[i], hull);
		for (int j = 0; j < hull.size(); j++)
		{
			line(image, hull[j], hull[(j + 1) % hull.size()], Scalar(0, 0, 255), 4, 8);
		}
	}
	namedWindow("hull", WINDOW_NORMAL);
	imshow("hull", image);
	waitKey(0);
}
//直线检测
void drawLines(Mat& image, vector<Vec2f> lines, Scalar scalar, int width);

void test39()
{
	Mat gray = imread("../houghline.jpg", IMREAD_GRAYSCALE);
	if (gray.empty())
		exit(-1);
	//检测图像边缘以及二值化
	Mat edge1;
	Canny(gray, edge1, 100, 200, 3, false);
	imshow("edge1", edge1);
	//用不同累加器进行直线检测
	vector<Vec2f> lines1, lines2;
	HoughLines(edge1, lines1, 1, CV_PI / 180.0, 50, 0);
	HoughLines(edge1, lines2, 1, CV_PI / 180.0, 100, 0);
	//在原图像中绘制直线
	Mat gray1, gray2;
	gray.copyTo(gray1);
	gray.copyTo(gray2);
	drawLines(gray1, lines1, Scalar(255), 2);
	drawLines(gray2, lines2, Scalar(255), 2);
	//显示图像
	imshow("gray", gray);
	imshow("gray1", gray1);
	imshow("gray2", gray2);
	waitKey(0);
	//利用渐进概率式霍夫变换绘制直线
	Mat edge2;
	Canny(gray, edge2, 80, 200, 3, false);
	imshow("edge2", edge2);
	//渐进概率式霍夫变换提取线段的点
	vector<Vec4i> linesP1, linesP2;
	HoughLinesP(edge2, linesP1, 1, CV_PI / 180.0, 150, 30, 10);	//两个点最大连接距离为10
	HoughLinesP(edge2, linesP2, 1, CV_PI / 180.0, 150, 30, 30);	//两个点最大连接距离为30
	//绘制最大距离为10的直线
	Mat gray3;
	gray.copyTo(gray3);
	for (int i = 0; i < linesP1.size(); i++)
	{
		line(gray3, Point(linesP1[i][0], linesP1[i][1]), Point(linesP1[i][2], linesP1[i][3]), Scalar(255), 4);
	}
	//绘制最大距离为30的直线
	Mat gray4;
	gray.copyTo(gray4);
	for (int i = 0; i < linesP2.size(); i++)
	{
		line(gray4, Point(linesP2[i][0], linesP2[i][1]), Point(linesP2[i][2], linesP2[i][3]), Scalar(255), 4);
	}
	//显示结果
	imshow("gray3", gray3);
	imshow("gray4", gray4);
	waitKey(0);
}
//直线拟合
void test40()
{
	system("color F0");	//改变输出界面颜色
	Vec4i lines;	//存放拟合的直线
	vector<Point2f> points;	//存放待拟合直线的所有点
	const static float Points[21][2] = { { 0.0f, 0.0f }, { 10.0f, 11.0f }, { 21.0f, 20.0f },
		{ 40.0f, 42.0f }, { 50.0f, 50.0f }, { 60.0f, 60.0f }, { 70.0f, 70.0f}, { 80.0f, 80.0f },
		{ 90.0f, 92.0f }, { 100.0f, 100.0f }, { 110.0f, 110.0f }, { 120.0f, 120.0f }, { 136.0f, 130.0f },
		{ 175.0f, 170.0f }, { 181.0f, 180.0f }, {200.0f, 190.0f }, { 23.5f , 23.5f } };
	for (int i = 0; i < 21; i++)	//将所有点放进vector中
	{
		points.push_back(Point2f(Points[i][0], Points[i][1]));
	}
	//参数设置
	double param = 0;	//距离模型中的数值参数C
	double reps = 0.01;	//坐标原点与直线之间的距离精度
	double aeps = 0.01;	//角度精度
	fitLine(points, lines, DIST_L1, param, reps, aeps);	//拟合直线
	double k = lines[1] / lines[0];	//直线斜率
	cout << "直线斜率 k = " << k << endl;
	cout << "直线上一点坐标 x = " << lines[2] << " y = " << lines[3] << endl;
	cout << "直线解析式：y = " << k << " * (x - " << lines[2] << ") - " << lines[3] << endl;
	waitKey(0);
	//三角形和圆形拟合
	RNG rng = RNG((unsigned)time(NULL));	//随机数种子
	while (true)
	{
		Mat img1 = Mat(500, 500, CV_8UC3, Scalar::all(0));	//创建图像进行三角形拟合
		Mat img2;
		img1.copyTo(img2);	//进行圆形拟合的图像
		int countB = rng.uniform(10, 101);	//蓝色随机点个数
		int countG = rng.uniform(10, 101);	//绿色随机点个数
		int countR = rng.uniform(10, 101);	//红色随机点个数
		vector<Point> ptsB, ptsG, ptsR;	//存放不同颜色随机点的容器
		Point areaB;	//标记蓝色点生成的区域
		areaB.x = rng.uniform(0, 4);
		areaB.y = rng.uniform(0, 4);
		for (int i = 0; i < countB; i++)	//生成蓝色的点
		{
			int x = rng.uniform(img1.cols * areaB.x / 4, img1.cols * (areaB.x + 1) / 4);	//随机点x坐标
			int y = rng.uniform(img1.cols * areaB.y / 4, img1.cols * (areaB.y + 1) / 4);	//随机点y坐标
			ptsB.push_back(Point(x, y));
		}
		Point areaG;	//标记绿色点生成的区域
		areaG.x = rng.uniform(0, 4);
		areaG.y = rng.uniform(0, 4);
		for (int i = 0; i < countG; i++)	//生成绿色的点
		{
			int x = rng.uniform(img1.cols * areaG.x / 4, img1.cols * (areaG.x + 1) / 4);	//随机点x坐标
			int y = rng.uniform(img1.cols * areaG.y / 4, img1.cols * (areaG.y + 1) / 4);	//随机点y坐标
			ptsG.push_back(Point(x, y));
		}
		Point areaR;	//标记红色点生成的区域
		areaR.x = rng.uniform(0, 4);
		areaR.y = rng.uniform(0, 4);
		for (int i = 0; i < countR; i++)	//生成红色的点
		{
			int x = rng.uniform(img1.cols * areaR.x / 4, img1.cols * (areaR.x + 1) / 4);	//随机点x坐标
			int y = rng.uniform(img1.cols * areaR.y / 4, img1.cols * (areaR.y + 1) / 4);	//随机点y坐标
			ptsR.push_back(Point(x, y));
		}
		//在图片中绘制随机点
		for (int i = 0; i < countB; i++)	//蓝色
		{
			circle(img1, ptsB[i], 3, Scalar(255, 0, 0), FILLED, 8, 0);
			circle(img2, ptsB[i], 3, Scalar(255, 0, 0), FILLED, 8, 0);
		}
		for (int i = 0; i < countG; i++)	//绿色
		{
			circle(img1, ptsG[i], 3, Scalar(0, 255, 0), FILLED, 8, 0);
			circle(img2, ptsG[i], 3, Scalar(0, 255, 0), FILLED, 8, 0);
		}
		for (int i = 0; i < countR; i++)	//红色
		{
			circle(img1, ptsR[i], 3, Scalar(0, 0, 255), FILLED, 8, 0);
			circle(img2, ptsR[i], 3, Scalar(0, 0, 255), FILLED, 8, 0);
		}
		//寻找包围点集的三角形
		vector<Point2f> triangleB, triangleG, triangleR;
		minEnclosingTriangle(ptsB, triangleB);
		minEnclosingTriangle(ptsG, triangleG);
		minEnclosingTriangle(ptsR, triangleR);
		//绘制拟合三角形
		for (int i = 0; i < 3; i++)
		{
			line(img1, triangleB[i], triangleB[(size_t)(i + 1) % 3], Scalar(255, 0, 0), 2, LINE_AA, 0);
			line(img1, triangleG[i], triangleG[(size_t)(i + 1) % 3], Scalar(0, 255, 0), 2, LINE_AA, 0);
			line(img1, triangleR[i], triangleR[(size_t)(i + 1) % 3], Scalar(0, 0, 255), 2, LINE_AA, 0);
		}
		//寻找包围点集的圆形
		Point2f centerB, centerG, centerR;
		float radiusB = 0, radiusG = 0, radiusR = 0;
		minEnclosingCircle(ptsB, centerB, radiusB);
		minEnclosingCircle(ptsG, centerG, radiusG);
		minEnclosingCircle(ptsR, centerR, radiusR);
		//绘制拟合圆形
		circle(img2, centerB, radiusB, Scalar(255, 0, 0), 2, LINE_AA, 0);
		circle(img2, centerG, radiusG, Scalar(0, 255, 0), 2, LINE_AA, 0);
		circle(img2, centerR, radiusR, Scalar(0, 0, 255), 2, LINE_AA, 0);
		//显示绘制结果
		imshow("TRIANGLE", img1);
		imshow("CIRCLE", img2);
		char input = waitKey();
		if (input == 27 || input == 'q' || input == 'Q')
		{
			break;
		}
	}
}
//二维码识别
void test41()
{
	Mat QRCode = imread("../QRCode.png", IMREAD_ANYCOLOR);
	if (QRCode.empty())
		exit(-1);

	Mat QRCode_gray, QRCode_bin;
	cvtColor(QRCode, QRCode_gray, COLOR_BGR2GRAY);
	QRCodeDetector QRCodedetector;
	vector<Point> points1;
	string information1;
	bool isQRCode = QRCodedetector.detect(QRCode_gray, points1);	//识别二维码
	if (isQRCode)
	{
		//解码二维码
		information1 = QRCodedetector.decode(QRCode_gray, points1, QRCode_bin);
		cout << points1 << endl;	//输出四个顶点的坐标
	}
	else
	{
		cout << "无法识别二维码，请确认图像是否含有二维码！" << endl;
		exit(-1);
	}
	//绘制二维码边框
	for (int i = 0; i < 4; i++)
	{
		line(QRCode, points1[i], points1[(i + 1) % 4], Scalar(0, 255, 255), 3, 8);
	}
	//将解码出的信息绘制到二维码上
	putText(QRCode, information1, Point(20, 20), 0, 1, Scalar(0, 0, 255), 2);
	//直接定位二维码并解码
	string information2;
	vector<Point> points2;
	information2 = QRCodedetector.detectAndDecode(QRCode_gray, points2);
	putText(QRCode, information2, Point(20, 40), 0, 1, Scalar::all(0), 2);
	cout << points2 << endl;
	//输出结果
	namedWindow("QRCode", WINDOW_GUI_NORMAL);
	namedWindow("QRCode_bin", WINDOW_GUI_NORMAL);
	imshow("QRCode", QRCode);
	imshow("QRCode_bin", QRCode_bin);
	waitKey(0);
}
//图像的积分求和
void test42()
{
	//创建一个16*16全为1的矩阵
	Mat matrix = Mat::ones(Size(16, 16), CV_32FC1);
	//在图像中随机加入噪声
	RNG rng = RNG((unsigned)time(NULL));
	for (int i = 0; i < matrix.rows; i++)
	{
		for (int j = 0; j < matrix.cols; j++)
		{
			float noise = rng.uniform(-0.5, 0.5);
			matrix.at<float>(i, j) = matrix.at<float>(i, j) + noise;
		}
	}
	//计算标准求和积分
	Mat sum;
	integral(matrix, sum);
	//便于展示转换为CV_8U形式
	Mat sum8U = Mat_<uchar>(sum);
	namedWindow("sum8U", WINDOW_GUI_NORMAL);
	imshow("sum8U", sum8U);
	//计算平方求和积分
	Mat sqsum;
	integral(matrix, sum, sqsum);
	//便于展示转换为CV_8U形式
	Mat sqsum8U = Mat_<uchar>(sqsum);
	namedWindow("sqsum8U", WINDOW_GUI_NORMAL);
	imshow("sqsum8U", sqsum8U);
	//计算倾斜求和积分
	Mat tilted;
	integral(matrix, sum, sqsum, tilted);
	//便于展示转换为CV_8U形式
	Mat tilted8U = Mat_<uchar>(tilted);
	namedWindow("tilted8U", WINDOW_GUI_NORMAL);
	imshow("tilted8U", tilted8U);
	waitKey(0);
}
//漫水填充法
void test43()
{
	Mat image = imread("../lena.jpg", IMREAD_ANYCOLOR);
	if (image.empty())
		exit(-1);
	
	RNG rng = RNG((unsigned)time(NULL));	//生成随机像素
	//设置操作标志flag
	int connectivity = 4;	//连通邻域方式
	int maskVal = 255;	//掩码图像数值
	int flags = connectivity | (maskVal << 8) | FLOODFILL_FIXED_RANGE;	//漫水填充方式标志
	//设置选中像素的差值
	Scalar loDiff = Scalar(20, 20, 20);
	Scalar upDiff = Scalar(20, 20, 20);
	//设置掩码矩阵变量(比原图像大2)
	Mat mask = Mat::zeros(image.rows + 2, image.cols + 2, CV_8UC1);
	while (true)
	{
		//随机产生图像中的一点
		int x = rng.uniform(0, image.cols);
		int y = rng.uniform(0, image.rows);
		Point point = Point(x, y);
		//随机产生填充的像素值
		Scalar newVal = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		//漫水填充函数
		int area = floodFill(image, mask, point, newVal, nullptr, loDiff, upDiff, flags);
		//输出像素点和像素点数目
		cout << "种子点：" << point << "\t填充像素数目：" << area << endl;
		//输出结果
		imshow("填充的彩色图像", image);
		imshow("掩码图像", mask);
		//判断程序是否结束
		char input = waitKey();
		if (input == 27)
		{
			break;
		}
	}
}
//分水岭法
void test44()
{
	Mat image, image_, gray, mask;
	Mat maskWaterShed;	//watershed()函数的参数
	image = imread("../lena.jpg");	//读取原图像
	image_ = imread("../lena_.png");	//读取含有标记的图像
	cvtColor(image_, gray, COLOR_BGR2GRAY);
	//二值化并进行开运算
	threshold(gray, mask, 254, 255, THRESH_BINARY);
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(mask, mask, MORPH_OPEN, kernel);
	imshow("原图像", image);
	imshow("含有标记的图像", image_);
	//在maskWaterShed上绘制轮廓，用于输入分水岭算法
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(mask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	maskWaterShed = Mat::zeros(mask.size(), CV_32S);
	for (int idx = 0; idx < contours.size(); idx++)
	{
		drawContours(maskWaterShed, contours, idx, Scalar::all(idx + 1), -1, 8, hierarchy, INT_MAX);
	}
	watershed(image, maskWaterShed);	//分水岭算法对原图像处理
	//随机生成几种颜色
	vector<Vec3b> colors;
	for (int i = 0; i < contours.size(); i++)
	{
		int b = theRNG().uniform(0, 256);
		int g = theRNG().uniform(0, 256);
		int r = theRNG().uniform(0, 256);
		colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
	}
	Mat result = Mat(image.size(), CV_8UC3);	//显示图像
	for (int i = 0; i < mask.rows; i++)
	{
		for (int j = 0; j < mask.cols; j++)
		{
			//绘制每个区域的颜色
			int idx = maskWaterShed.at<int>(i, j);
			if (idx == -1)	//区域的值为-1(即边界)
			{
				result.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
			}
			else if (idx <= 0 || idx > contours.size())	//没有标记清楚的区域为0
			{
				result.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
			else
			{
				result.at<Vec3b>(i, j) = colors[(size_t)idx - 1];	//将这些区域绘制成不同颜色
			}
		}
	}
	//显示结果
	imshow("result", result);
	result = result * 0.8 + image * 0.2;
	//addWeighted(result, 0.8, image, 0.2, 0, result, -1);
	imshow("分水岭结果", result);
	//绘制每个分区图像
	for (int n = 0; n < contours.size(); n++)
	{
		Mat result_ = Mat(image.size(), CV_8UC3);
		for (int i = 0; i < image.rows; i++)
		{
			for (int j = 0; j < image.cols; j++)
			{
				int idx = maskWaterShed.at<int>(i, j);
				if (idx == n + 1)
				{
					result_.at<Vec3b>(i, j) = image.at<Vec3b>(i, j);
				}
				else
				{
					result_.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
				}
			}
		}
		//显示图像
		imshow(to_string(n), result_);
	}
	waitKey(0);
}
//Harris角点
void test45()
{
	Mat lena = imread("../lena.jpg", IMREAD_COLOR);
	Mat gray;
	cvtColor(lena, gray, COLOR_BGR2GRAY);
	//计算Harris系数
	Mat harris;
	int blockSize = 2;	//邻域半径
	int aperturSize = 3;	//Soble算子大小
	cornerHarris(gray, harris, blockSize, aperturSize, 0.04);
	//归一化便于数值比较和结果显示
	Mat harrisn;
	normalize(harris, harrisn, 0, 255, NORM_MINMAX);
	//将图像数据类型变为CV_8U
	convertScaleAbs(harrisn, harrisn);
	//寻找Harris角点
	vector<KeyPoint> keyPoints;
	for (int i = 0; i < harrisn.rows; i++)
	{
		for (int j = 0; j < harrisn.cols; j++)
		{
			int R = harrisn.at<uchar>(i, j);
			if (R > 125)
			{
				//将角点存入KeyPoint中
				KeyPoint keyPoint;
				keyPoint.pt.x = j;
				keyPoint.pt.y = i;
				keyPoints.push_back(keyPoint);
			}
		}
	}
	//绘制角点
	drawKeypoints(lena, keyPoints, lena);
	imshow("系数矩阵", harrisn);
	imshow("Harris角点", lena);
	waitKey(0);
}
//Shi-Tomas角点检测
void test46()
{
	Mat image = imread("../lena.jpg");
	if (image.empty())
		exit(-1);
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	//提取角点
	int maxCorners = 100;	//检测角点数目
	double qualityLevel = 0.01;	//质量等级，即阈值与最佳角点之间的关系
	double minDistance = 0.04;	//两个角点之间的最小欧式距离
	vector<Point2f> corners;	//存放角点的坐标
	goodFeaturesToTrack(gray, corners, maxCorners, qualityLevel, minDistance, Mat(), 3, false);
	//绘制角点
	vector<KeyPoint> keyPoints;	//将角点存放到KeyPoints中，方便后期绘制
	for (int i = 0; i < corners.size(); i++)
	{
		KeyPoint keyPoint;
		keyPoint.pt = corners[i];
		keyPoints.push_back(keyPoint);
	}
	drawKeypoints(image, keyPoints, image);
	imshow("角点结果", image);
	waitKey(0);
}
//亚像素级别角点优化
void test47()
{
	system("color F0");
	Mat image = imread("../lena.jpg");
	if (image.empty())
		exit(-1);
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	//提取角点
	int maxCorner = 100;	//检测角点数目
	double qualityLevel = 0.01;	//质量等级
	double mindistance = 0.04;	//最小欧式距离
	vector<Point2f> corners;	//存放角点坐标
	goodFeaturesToTrack(gray, corners, maxCorner, qualityLevel, mindistance, Mat(), 3, false);
	//计算亚像素级角点坐标
	vector<Point2f> cornersSub = corners;	//将角点备份防止函数修改
	Size winSize = Size(5, 5);	//窗口大小，实际为改大小的2倍+1
	Size zeroZone = Size(-1, -1);	//死区大小
	TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
	cornerSubPix(gray, cornersSub, winSize, zeroZone, criteria);
	//输出初始化坐标和精细化坐标
	for (int i = 0; i < corners.size(); i++)
	{
		string str = to_string(i);
		str = "第" + str + "个角点初始坐标：";
		cout << str << corners[i] << "精细化后的坐标：" << cornersSub[i] << endl;
	}
}
//ORB特征点
void orb_features(Mat& image, vector<KeyPoint>& keyPoints, Mat& descriptions);	//计算ORB特征点

void test48()
{
	Mat image = imread("../lena.jpg");
	if (image.empty())
		exit(-1);
	vector<KeyPoint> keyPoints;
	Mat descriptions;
	//计算 ORB 特征点
	orb_features(image, keyPoints, descriptions);
	 //绘制特征点
	Mat imageAngle;
	image.copyTo(imageAngle);
	//绘制不含角度大小的结果
	drawKeypoints(image, keyPoints, image, Scalar(255, 255, 255));
	//绘制含有角度和大小的结果
	drawKeypoints(imageAngle, keyPoints, imageAngle, Scalar(255, 255, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//绘制结果
	imshow("image", image);
	imshow("imageAngle", imageAngle);
	waitKey(0);
}
//特征点匹配
void match_min(vector<DMatch>& matches, vector<DMatch>& good_matches);	//筛选汉明距离

void test49()
{
	Mat gray = imread("../book.jpg");
	Mat gray_ = imread("../book_.jpg");
	if (gray.empty() || gray_.empty())
		exit(-1);
	//计算ORB特征点
	vector<KeyPoint> keyPoints1, keyPoints2;
	Mat description1, description2;
	orb_features(gray, keyPoints1, description1);
	orb_features(gray_, keyPoints2, description2);
	//特征点匹配
	vector<DMatch> matches;	//定义匹配存放结果的变量
	BFMatcher matcher(NORM_HAMMING);	//定义特征点匹配的类，使用汉明距离
	matcher.match(description1, description2, matches);	//特征点匹配
	cout << "matches = " << matches.size() << endl;	//匹配成功的特征点数目
	//通过汉明距离筛选匹配结果
	vector<DMatch> good_matches;
	match_min(matches, good_matches);
	//绘制结果
	Mat out1, out2;
	drawMatches(gray, keyPoints1, gray_, keyPoints2, matches, out1);
	drawMatches(gray, keyPoints1, gray_, keyPoints2, good_matches, out2);
	//输出结果
	imshow("未筛选结果", out1);
	waitKey(0);
	imshow("筛选结果", out2);
	waitKey(0);
}
//RANSCA 优化特征点匹配
void ransac(vector<DMatch>& matches, vector<KeyPoint>& queryKeyPoint, vector<KeyPoint>& trainKeyPoint, vector<DMatch>& matches_ransac);	//ransac算法实现

void test50()
{
	Mat image1 = imread("../book.jpg");
	Mat image2 = imread("../book_.jpg");
	if (image1.empty() || image2.empty())
		exit(-1);
	vector<KeyPoint> keyPoints1, keyPoints2;
	Mat description1, description2;
	orb_features(image1, keyPoints1, description1);
	orb_features(image2, keyPoints2, description2);
	//特征点匹配
	vector<DMatch> matches;	//定义匹配存放结果的变量
	BFMatcher matcher(NORM_HAMMING);	//定义特征点匹配的类，使用汉明距离
	matcher.match(description1, description2, matches);	//特征点匹配
	cout << "matches = " << matches.size() << endl;	//匹配成功的特征点数目
	//通过汉明距离筛选匹配结果
	vector<DMatch> good_matches;
	match_min(matches, good_matches);
	//ransac算法筛选配结果
	vector<DMatch> good_ransac;
	ransac(good_matches, keyPoints1, keyPoints2, good_ransac);
	//绘制结果
	Mat out1, out2, out3;
	drawMatches(image1, keyPoints1, image2, keyPoints2, matches, out1);
	drawMatches(image1, keyPoints1, image2, keyPoints2, good_matches, out2);
	drawMatches(image1, keyPoints1, image2, keyPoints2, good_ransac, out3);
	imshow("未筛选结果", out1);
	waitKey(0);
	imshow("最小汉明距离筛选", out2);
	waitKey(0);
	imshow("ransac筛选", out3);
	waitKey(0);
}
//相机模型与投影
void test51()
{
	//输入计算的得到的内参矩阵和畸变矩阵
	Mat cameraMatrix = (Mat_<float>(3, 3) << 532.016297, 0, 332.172519,
		0, 531.565159, 233.388075,
		0, 0, 1);
	Mat distCoeffs = (Mat_<float>(1, 5) << -0.285188, 0.080097, 0.001274,
		-0.002415, 0.106579);
	//代码清单10-10中计算的第一张图片相机坐标系与世界坐标系之间的关系
	Mat rvec = (Mat_<float>(1, 3) << -1.977853, -2.002220, 0.130029);
	Mat tvec = (Mat_<float>(1, 3) << -26.8815, -42.79936, 159.19703);
	//生成第一张图片中内角点的三维世界坐标
	Size boardSize = Size(9, 6);
	Size squareSize = Size(10, 10);	//棋盘格每个方格的真实尺寸
	vector<Point3f> PointSets;
	for (int j = 0; j < boardSize.height; j++)
	{
		for (int k = 0; k < boardSize.width; k++)
		{
			Point3f realPoint;
			//假设标定板为世界坐标系的z平面，即z=0
			realPoint.x = j * squareSize.width;
			realPoint.y = k * squareSize.height;
			realPoint.z = 0;
			PointSets.push_back(realPoint);
		}
	}
	//根据三维坐标和相机与世界坐标系时间的关系估计内角点像素坐标
	vector<Point2f> imagePoints;
	projectPoints(PointSets, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
	for (int i = 0; i < imagePoints.size(); i++)
	{
		cout << "第" << to_string(i) << "个点的坐标" << imagePoints[i] << endl;
	}
	waitKey(0);
}
//单目相机的标定
void test52()
{
	//标定板需要有白边，需要清晰
	vector<Mat> images;	//读取所有图片
	string imagename;
	ifstream ifs("../test.txt");
	while (getline(ifs, imagename))
	{
		Mat image = imread(imagename);
		images.push_back(image);
	}
	Size board_size = Size(8, 5);	//方格标定板角点数目(行，列)
	vector<vector<Point2f>> imagesPoints;
	for (int i = 0; i < images.size(); i++)
	{
		Mat image_ = images[i];
		Mat gray;
		cvtColor(image_, gray, COLOR_BGR2GRAY);
		vector<Point2f> image_Points;
		findChessboardCorners(gray, board_size, image_Points);	//计算方格标定板角点
		find4QuadCornerSubpix(gray, image_Points, Size(5, 5));	//细化方格标定板坐标
		bool pattern = true;
		drawChessboardCorners(image_, board_size, image_Points, pattern);
		imshow("image_", image_);
		waitKey(0);
		imagesPoints.push_back(image_Points);
	}
	//生成棋盘格内每个内角点的空间三维坐标
	Size squareSize = Size(1, 1);	//棋盘格的真实尺寸
	vector<vector<Point3f>> objectPoints;
	for (int i = 0; i < imagesPoints.size(); i++)
	{
		vector<Point3f> tempPoints;
		for (int j = 0; j < board_size.height; j++)
		{
			for (int k = 0; k < board_size.width; k++)
			{
				Point3f realPoint;
				realPoint.x = j * squareSize.width;
				realPoint.y = k * squareSize.height;
				//假设标定板为世界坐标系的oxy面，即z=0
				realPoint.z = 0;
				tempPoints.push_back(realPoint);
			}
		}
		objectPoints.push_back(tempPoints);
	}
	//图像尺寸
	Size imageSize = images[0].size();
	
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));	//相机的内参数矩阵
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));	//摄像机的5个畸变系数：k1,k2,p1,p2,k3
	vector<Mat> rvecs;	//每张图像的旋转向量
	vector<Mat> tvecs;	//每张图像的平移量
	calibrateCamera(objectPoints, imagesPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);
	cout << "内参数矩阵：" << endl << cameraMatrix << endl;
	cout << "畸变系数：" << distCoeffs << endl;
	waitKey(0);
}
//图像的校正
void undist(vector<Mat> images, Mat cameraMatrix, Mat distCoeffs, vector<Mat>& undistImages);	//图像校正函数

void test53()
{
	vector<Mat> images;	//读取所有图片
	string imagename;
	ifstream ifs("../test.txt");
	while (getline(ifs, imagename))
	{
		Mat image = imread(imagename);
		images.push_back(image);
	}
	
	Mat cameraMatrix = (Mat_<float>(3, 3) << 315.0799467892673, 0, 362.4333928879762,
	0, 314.3207165955638, 235.8589898288922,
	0, 0, 1);	//内参数矩阵	
	Mat distCoeffs = (Mat_<float>(1, 5) << -0.05984332512739104, 0.5171941952024617, -0.008125548898121952, 0.005511163544186489, -0.9147395306811681);	//畸变系数

	vector<Mat> undistImages;	//存放校正后的图片

	undist(images, cameraMatrix, distCoeffs, undistImages);	//利用undist函数计算校正函数
	//显示校正后的图像
	for (int i = 0; i < images.size(); i++)
	{

		string windowNumber = to_string(i);
		imshow("未校正的图像" + windowNumber, images[i]);
		imshow("校正后的图像" + windowNumber, undistImages[i]);
		waitKey(0);
		destroyWindow("未校正的图像" + windowNumber);
		destroyWindow("校正后的图像" + windowNumber);
	}
}
//单目相机位姿估计
void test54()
{
	Mat image = imread("../test2.jpg");
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	vector<Point2f> imgPoints;
	Size boardSize = Size(8, 5);
	findChessboardCorners(gray, boardSize, imgPoints);	//计算方格标定板角点
	find4QuadCornerSubpix(gray, imgPoints, boardSize);	//细化方格标定板角点坐标
	//生成棋盘格每个内角点的空间三维空间坐标
	Size squareSize = Size(1, 1);	//棋盘格每个方格真实尺寸
	vector<Point3f> PointSets;
	for (int j = 0; j < boardSize.height; j++)
	{
		for (int k = 0; k < boardSize.width; k++)
		{
			Point3f realPoint;
			realPoint.x = j * squareSize.width;
			realPoint.y = k * squareSize.height;
			realPoint.z = 0;
			PointSets.push_back(realPoint);
		}
	}
	Mat cameraMatrix = (Mat_<float>(3, 3) << 315.0799467892673, 0, 362.4333928879762,
		0, 314.3207165955638, 235.8589898288922,
		0, 0, 1);	//内参数矩阵	
	Mat distCoeffs = (Mat_<float>(1, 5) << -0.05984332512739104, 0.5171941952024617, -0.008125548898121952, 0.005511163544186489, -0.9147395306811681);	//畸变系数
	//利用PnP算法计算旋转和平移
	Mat rvec, tvec;
	solvePnP(PointSets, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);
	cout << "世界坐标系变换到相机坐标系的旋转向量：" << rvec << endl;
	//旋转向量转换旋转矩阵
	Mat R;
	Rodrigues(rvec, R);
	cout << "旋转向量转换旋转矩阵：" << endl << R << endl;
	//用PnP+RANSAC算法计算旋转向量和平移向量
	Mat rvecRansac, tvecRansac;
	solvePnPRansac(PointSets, imgPoints, cameraMatrix, distCoeffs, rvecRansac, tvecRansac);
	Mat RRansac;
	Rodrigues(rvecRansac, RRansac);
	cout << "旋转向量转换为旋转矩阵：" << endl << RRansac << endl;
	waitKey(0);
}
//差值检测移动物体
void test55()
{
	VideoCapture capture("../car.avi");
	if (!capture.isOpened())
		exit(-1);
	//输出视频相关信息
	int fps = capture.get(CAP_PROP_FPS);
	int width = capture.get(CAP_PROP_FRAME_WIDTH);
	int height = capture.get(CAP_PROP_FRAME_HEIGHT);
	int num_of_frames = capture.get(CAP_PROP_FRAME_COUNT);
	cout << "视频帧率：" << fps << "\n视频宽度：" << width << "\n视频高度：" << height << "\n视频总帧数：" << num_of_frames << endl;
	//读取视频第一帧图像作为前一帧图像
	Mat preFrame, preGray;
	capture.read(preFrame);
	cvtColor(preFrame, preGray, COLOR_BGR2GRAY);
	//进行高斯滤波减少噪声
	GaussianBlur(preGray, preGray, Size(0, 0), 15);
	//提取运动物体
	Mat binary;
	Mat frame, gray;
	while (true)
	{
		if (!capture.read(frame))	//视频读取结束，退出循环
		{
			break;
		}
		//对当前帧进行灰度化和高斯滤波
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		GaussianBlur(gray, gray, Size(0, 0), 15);
		//前后帧图像相减取绝对值
		absdiff(preGray, gray, binary);
		//对相减结果进行二值化并开运算
		threshold(binary, binary, 100, 255, THRESH_BINARY | THRESH_OTSU);
		Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));
		morphologyEx(binary, binary, MORPH_OPEN, kernel);
		//显示结果
		imshow("input", frame);
		imshow("output", binary);
		//将当前帧变为前一帧，准备下一循环，注释掉该语句表示使用固定背景
		frame.copyTo(preFrame);
		//5毫秒延迟判断是否退出程序
		char ch = waitKey(5);
		if (ch == 27)
		{
			break;
		}
	}
}
//稠密光流法跟踪移动物体
void test56()
{
	VideoCapture capture("../car.avi");
	if (!capture.isOpened())
		exit(-1);
	Mat preFrame, preGray;
	capture.read(preFrame);
	cvtColor(preFrame, preGray, COLOR_BGR2GRAY);
	while (true)
	{
		Mat frame, gray;
		if (!capture.read(frame))
		{
			break;
		}
		imshow("原视频", frame);
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		GaussianBlur(gray, gray, Size(3, 3), 10);
		//计算稠密光流
		Mat_<Point2f> flow;	//两个方向的运动速度
		calcOpticalFlowFarneback(preGray, gray, flow, 0.5, 3, 15, 5, 5, 1.2, 0);
		
		Mat xV = Mat::zeros(preFrame.size(), CV_32FC1);	//x方向移动速度
		Mat yV = Mat::zeros(preFrame.size(), CV_32FC1);	//y方向移动速度
		//提取两个方向的移动速度
		for (int i = 0; i < flow.rows; i++)
		{
			for (int j = 0; j < flow.cols; j++)
			{
				const Point2f& flow_xy = flow.at<Point2f>(i, j);
				xV.at<float>(i, j) = flow_xy.x;
				yV.at<float>(i, j) = flow_xy.y;
			}
		}
		//计算向量弧度和幅值
		Mat magnitude, angle;
		cartToPolar(xV, yV, magnitude, angle);
		//将弧度转化为角度
		angle = angle * 180 / CV_PI;
		//将幅值归一化到0~255区间，以便显示
		normalize(magnitude, magnitude, 0, 255, NORM_MINMAX);
		//计算角度和幅值的绝对值
		convertScaleAbs(magnitude, magnitude);
		convertScaleAbs(angle, angle);
		//用运动的幅值和角度生成HSV颜色空间
		Mat HSV = Mat::zeros(preFrame.size(), preFrame.type());
		vector<Mat> result;
		split(HSV, result);
		result[0] = angle;	//决定颜色
		result[1] = Scalar(255);	
		result[2] = magnitude;	//决定形态
		//将三个单通道图像合并成多通道图像
		merge(result, HSV);
		//将HSV颜色空间转换到RGB颜色空间
		Mat RGBImg;
		cvtColor(HSV, RGBImg, COLOR_HSV2BGR);
		//显示结果
		imshow("运动检测结果", RGBImg);
		char ch = waitKey(5);
		if (ch == 27)
		{
			break;
		}
	}
}
//稀疏光流法跟踪移动物体
vector<Scalar> color_lut;	//颜色查找表
void draw_lines(Mat& image, vector<Point2f> pt1, vector<Point2f> pt2);

void test57()
{
	VideoCapture capture("../car.avi");
	Mat preFrame, preGray;
	if (!capture.isOpened())
		exit(-1);
	capture.read(preFrame);
	cvtColor(preFrame, preGray, COLOR_BGR2GRAY);
	//角点检测相关参数设置
	vector<Point2f> points;
	double qualityLevel = 0.01;
	int minDistance = 10;
	int blockSize = 3;
	bool usrHarrisDetector = false;
	double k = 0.04;
	int Corners = 5000;
	//角点检测
	goodFeaturesToTrack(preGray, points, Corners, qualityLevel, minDistance, Mat(), blockSize, usrHarrisDetector, k);
	//稀疏光流检测相关参数设置
	vector<Point2f> prePoints;	//前一帧图像角点坐标
	vector<Point2f> nextPoints;	//当前帧图像角点坐标
	vector<uchar> status;	//角点检测到的状态
	vector<float> err;	//误差
	TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
	double derivlamda = 0.5;
	int flags = 0;
	//初始状态的角点
	vector<Point2f> initPoints;
	initPoints.insert(initPoints.end(), points.begin(), points.end()); 
	//前一帧图像中角点坐标
	prePoints.insert(prePoints.end(), points.begin(), points.end());
	
	while (true)
	{
		Mat nextFrame, nextGray;
		if (!capture.read(nextFrame))
		{
			break;
		}
		imshow("nextFrame", nextFrame);
		cvtColor(nextFrame, nextGray, COLOR_BGR2GRAY);
		//光流跟踪
		calcOpticalFlowPyrLK(preGray, nextGray, prePoints, nextPoints, status, err, Size(21, 21), 3, criteria, flags);
		//判断角点是否移动，不移动就删除
		size_t i, k;
		for (i = k = 0; i < prePoints.size(); i++)
		{
			//距离与状态测量
			double dist = abs(prePoints[i].x - nextPoints[i].x) + abs(prePoints[i].y - nextPoints[i].y);
			if (status[i] && dist > 2)
			{
				prePoints[k] = prePoints[i];
				initPoints[k] = initPoints[i];
				nextPoints[k++] = nextPoints[i];
				circle(nextFrame, nextPoints[i], 3, Scalar(0, 255, 0), -1);
			}
		}
		//更新角点数目
		prePoints.resize(k);
		nextPoints.resize(k);
		initPoints.resize(k);
		//绘制跟踪轨迹
		draw_lines(nextFrame, initPoints, nextPoints);
		imshow("result", nextFrame);

		char ch = waitKey(50);
		if (ch == 27)
		{
			break;
		}
		//更新角点坐标和前一帧图像
		swap(prePoints, nextPoints);
		nextGray.copyTo(preGray);
		//如果角点数目小于30个，则重新检测角点
		if (initPoints.size() < 30)
		{
			goodFeaturesToTrack(preGray, points, Corners, qualityLevel, minDistance, Mat(), blockSize, usrHarrisDetector, k);
			initPoints.insert(initPoints.end(), points.begin(), points.end());
			prePoints.insert(prePoints.end(), points.begin(), points.end());
			cout << "total feature points: " << prePoints.size() << endl;
		}
	}
}
//k均值聚类
void test58()
{
	//生成一个500*500的图像用于显示特征点和分类结果
	Mat image(500, 500, CV_8UC3, Scalar::all(255));
	RNG rng(10000);
	//设置三种颜色
	Scalar colors[3] = { Scalar(0, 0, 255), Scalar(0, 255, 0), Scalar(255, 0, 0) };
	//设置三个点集，并且每个点集中点的数目随机
	int number = 3;
	int Points1 = rng.uniform(20, 200);
	int Points2 = rng.uniform(20, 200);
	int Points3 = rng.uniform(20, 200);
	int Points_num = Points1 + Points2 + Points3;
	Mat Points(Points_num, 1, CV_32FC2);

	int i = 0;
	for (; i < Points1; i++)
	{
		Point2f pts;
		pts.x = rng.uniform(100, 200);
		pts.y = rng.uniform(390, 490);
		Points.at<Point2f>(i, 0) = pts;
	}
	for (; i < Points1 + Points2; i++)
	{
		Point2f pts;
		pts.x = rng.uniform(300, 400);
		pts.y = rng.uniform(100, 300);
		Points.at<Point2f>(i, 0) = pts;
	}
	for (; i < Points1 + Points2 + Points3; i++)
	{
		Point2f pts;
		pts.x = rng.uniform(100, 200);
		pts.y = rng.uniform(100, 200);
		Points.at<Point2f>(i, 0) = pts;
	}
	//使用KMeans
	Mat labels;	//每个点所属的种类
	Mat centers;	//每个类的中心位置坐标
	kmeans(Points, number, labels, TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01), 3, KMEANS_PP_CENTERS, centers);
	//根据分类为每个点设置不同的颜色
	for (i = 0; i < Points_num; i++)
	{
		int index = labels.at<int>(i);
		Point point = Points.at<Point2f>(i);
		circle(image, point, 2, colors[index], -1, 4);
	}
	//绘制每个聚类的中心来绘制圆
	for (i = 0; i < centers.rows; i++)
	{
		int x = centers.at<float>(i, 0);
		int y = centers.at<float>(i, 1);
		cout << "第" << i + 1 << "类的中心坐标：x = " << x << " y = " << y << endl;
		circle(image, Point(x, y), 50, colors[i], 1, LINE_AA);
	}
	imshow("K均值聚类分类结果", image);
	waitKey(0);
	
	Mat image_ = imread("../lena.jpg");
	if (image_.empty())
		exit(-1);
	Vec3b colorLut[3] = { Vec3b(0, 0, 255), Vec3b(0, 255, 0), Vec3b(255, 0, 0) };
	//图像尺寸，用于计算图像中像素点的数目
	int width = image_.cols;
	int height = image_.rows;
	//初始化定义
	int sampleCount = width * height;
	//将图像矩阵数据转化成每一行一个数据的形式
	Mat sample_data = image_.reshape(3, sampleCount);
	Mat data;
	sample_data.convertTo(data, CV_32F);
	//KMeans函数将像素值进行分类
	int number_ = 3;	//分割后的颜色种类
	Mat labels_;
	TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 0.1);
	kmeans(data, number_, labels_, criteria, number_, KMEANS_PP_CENTERS);
	//显示图像分割结果
	Mat result = Mat::zeros(image_.size(), image_.type());
	for (int row = 0; row < height; row++)
	{
		for (int col = 0; col < width; col++)
		{
			int index = row * width + col;
			int label = labels_.at<int>(index, 0);
			result.at<Vec3b>(row, col) = colorLut[label];
		}
	}
	namedWindow("原图", WINDOW_NORMAL);
	imshow("原图", image_);
	namedWindow("分割后图像", WINDOW_NORMAL);
	imshow("分割后图像", result);
	waitKey(0);
}
//加载深度神经网络模型
void test59()
{
	using namespace cv::dnn;
	system("color F0");
	string model = "../googlenet/bvlc_googlenet.caffemodel";
	string config = "../googlenet/bvlc_googlenet.prototxt";
	//加载模型
	Net net = dnn::readNet(model, config);
	if (net.empty())
		exit(-1);
	//获取每一层信息
	vector<string> layerNames = net.getLayerNames();
	for (int i = 0; i < layerNames.size(); i++)
	{
		//读取每一层网络的ID
		int ID = net.getLayerId(layerNames[i]);
		//读取每一层网络的信息
		Ptr<Layer> layer = net.getLayer(ID);
		cout << "网络层数：" << ID << "  网络层名称：" << layerNames[i] << endl
			<< "网络层类型：" << layer->type.c_str() << endl;

	}
}
//深度神经网络模型的使用
void test60()	//物品时辨别
{
	using namespace cv::dnn;
	Mat image = imread("../airplane.jpg");
	if (image.empty())
		exit(-1);

	//读取分类种类名称
	String typeListFile = "../image_recognition/imagenet_comp_graph_label_strings.txt";
	vector<String> typeList;
	ifstream file(typeListFile);
	if (!file.is_open())
		exit(-1);
	string type;
	while (!file.eof())
	{
		getline(file, type);	//读取名称
		if (type.length())
			typeList.push_back(type);
	}
	file.close();
	//加载网络
	String tf_pb_file = "../image_recognition/tensorflow_inception_graph.pb";
	Net net = readNet(tf_pb_file);
	if (net.empty())
		exit(-1);
	//对输入图像进行数据处理
	Mat blob = blobFromImage(image, 1.0f, Size(224, 224), Scalar(), true, false);
	//进行图像种类预测
	Mat prob;
	net.setInput(blob, "input");
	prob = net.forward("softmax2");
	//得到最可能分类输出
	Mat probMat = prob.reshape(1, 1);
	Point classNumber;
	double classProb;	//最大可能性
	minMaxLoc(probMat, NULL, &classProb, NULL, &classNumber);

	string typeName = typeList.at(classNumber.x).c_str();
	cout << "物体可能为：" << typeName << "  可能性为：" << classProb;
	//检测内容
	string str = typeName + " Possibility: " + to_string(classProb);
	putText(image, str, Point(25, 50), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2, 8);
	imshow("图像判断结果", image);
	waitKey(0);
}

void test61()	//图像风格化
{
	using namespace cv::dnn;
	Mat image = imread("../lena.jpg");
	String models[5] = { "candy.t7", "feathers.t7", "la_muse.t7", "starry_night.t7", "the_scream.t7" };
	for (int i = 0; i < size(models); i++)
	{
		Net net = readNet("../fast_style/" + models[i]);
		imshow("原始图像", image);
		//计算图像每个通道的均值
		Scalar imageMean = mean(image);
		//调整图像尺寸格式
		Mat blobImage = blobFromImage(image, 1.0, Size(256, 256), imageMean, false, false);
		//计算网络对原图像处理结果
		net.setInput(blobImage);
		Mat output = net.forward();
		//输出结果的尺寸和通道数
		int outputChannels = output.size[1];
		int outputRows = output.size[2];
		int outputCols = output.size[3];
		//将输出结果存放到图像中
		Mat result = Mat::zeros(Size(outputCols, outputRows), CV_32FC3);
		float* data = output.ptr<float>();
		for (int channel = 0; channel < outputChannels; channel++)
		{
			for (int row = 0; row < outputRows; row++)
			{
				for (int col = 0; col < outputCols; col++)
				{
					result.at<Vec3f>(row, col)[channel] = *data++;
				}
			}
		}
		//对迁移结果进行进一步操作处理
		//回复图像减掉的均值
		result = result + imageMean;
		//对图像进行归一化，便于图像显示
		result = result / 255.0;
		//调整图像尺寸，使得与原图像尺寸相同
		resize(result, result, image.size());
		//显示结果
		imshow("第" + to_string(i + 1) + "种风格迁移结果", result);
		waitKey(0);
	}
}

void test62()	//人脸检测和性别判别
{
	using namespace cv::dnn;
	Mat image = imread("../faces.jpg");
	if (image.empty())
		exit(-1);
	//读取人脸识别模型
	String model_bin = "../face_gender/opencv_face_detector_uint8.pb";
	String config_text = "../face_gender/opencv_face_detector.pbtxt";
	Net faceNet = readNet(model_bin, config_text);
	//读取性别检测模型
	String genderModel = "../face_gender/gender_net.caffemodel";
	String genderProto = "../face_gender/gender_deploy.prototxt";
	String genderList[] = { "Male", "Female" };
	Net genderNet = readNet(genderModel, genderProto);
	if (faceNet.empty() || genderNet.empty())
		exit(-1);
	//对整幅图像进行人脸检测
	Mat blobImage = blobFromImage(image, 1.0, Size(300, 300), Scalar(), false, false);
	faceNet.setInput(blobImage, "data");
	Mat detect = faceNet.forward("detection_out");
	//人脸概率、人脸矩形区域的位置
	Mat detectionMat(detect.size[2], detect.size[3], CV_32F, detect.ptr<float>());
	//对每个人脸区域进行性别检测
	int exBoundray = 20;	//每个人脸区域四个方向扩充尺寸
	float confidenceThreshold = 0.5;	//判定为人脸的概率阈值，阈值越大准确性越高
	for (int i = 0; i < detectionMat.rows; i++)
	{
		float confidence = detectionMat.at<float>(i, 2);	//检测为人脸的概率
		if (confidence > confidenceThreshold)
		{
			//网络检测人脸区域大小
			int topLx = detectionMat.at<float>(i, 3) * image.cols;
			int topLy = detectionMat.at <float>(i, 4) * image.rows;
			int bottomRx = detectionMat.at<float>(i, 5) * image.cols;
			int bottomRy = detectionMat.at<float>(i, 6) * image.rows;
			Rect faceRect(topLx, topLy, bottomRx - topLx, bottomRy - topLy);
			//将网络检测出的区域尺寸进行扩充，要注意防止尺寸在图像真实尺寸之外
			Rect faceTextRect;
			faceTextRect.x = max(0, faceRect.x - exBoundray);
			faceTextRect.y = max(0, faceRect.y - exBoundray);
			faceTextRect.width = min(faceRect.width + exBoundray, image.cols - 1);
			faceTextRect.height = min(faceRect.height + exBoundray, image.rows - 1);
			Mat face = image(faceTextRect);	//扩充后的人脸图像
			//调整面部图像尺寸
			Mat faceblob = blobFromImage(face, 1.0, Size(227, 227), Scalar(), false, false);
			//将调整后的面部图像输入到性别检测网络
 			genderNet.setInput(faceblob);
			//计算检测结果
			Mat genderPreds = genderNet.forward();	//两个性别的可能性
			//性别检测结果
			float male, female;
			male = genderPreds.at<float>(0, 0);
			female = genderPreds.at<float>(0, 1);
			int classID = male > female ? 0 : 1;
			String gender = genderList[classID];
			//在原图像中绘制面部轮廓和性别
			rectangle(image, faceRect, Scalar(0, 0, 255), 2, 8, 0);
			putText(image, gender.c_str(), faceRect.tl(), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, 8);
		}	
	}
	imshow("检测结果", image);
	waitKey(0);
}
//监督学习聚类	K近邻分类方法
void test63()
{
	using namespace cv::ml;
	Mat image = imread("../handwriting.png");
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	//分割为5000个cells
	Mat images = Mat::zeros(5000, 400, CV_8UC1);
	Mat labels = Mat::zeros(5000, 1, CV_8UC1);

	int index = 0;
	Rect numberImage;
	numberImage.x = 0;
	numberImage.height = 1;
	numberImage.width = 400;
	for (int row = 0; row < 50; row++)
	{
		//从图像中分割出20*20的图像作为独立数字图像
		int label = row / 5;
		int datay = row * 20;
		for (int col = 0; col < 100; col++)
		{
			int datax = col * 20;
			Mat number = Mat::zeros(Size(20, 20), CV_8UC1);
			for (int x = 0; x < 20; x++)
			{
				for (int y = 0; y < 20; y++)
				{
					number.at<uchar>(x, y) = gray.at<uchar>(x + datay, y + datax);
				}
			}
			//将二维图片数据转换成行数据
			Mat row = number.reshape(1, 1);
			cout << "提取第" << index + 1 << "个数据" << endl;
			numberImage.y = index;
			//添加到总数据中
			row.copyTo(images(numberImage));
			//记录每个图像对应的数字标签
			labels.at<uchar>(index, 0) = label;
			index++;
 		}
	}
	imwrite("../所有数据按行排列结果.png", images);
	imwrite("../标签.png", labels);
	//加载训练数据集
	images.convertTo(images, CV_32FC1);
	labels.convertTo(labels, CV_32SC1);
	Ptr<ml::TrainData> tdata = ml::TrainData::create(images, ml::ROW_SAMPLE, labels);
	//创建K近邻类
	Ptr<KNearest> knn = KNearest::create();
	knn->setDefaultK(5);	//每个类别拿出5个数据
	knn->setIsClassifier(true);	//进行分类
	//训练数据
	knn->train(tdata);
	//保存训练结束
	knn->save("../knn_model.ymal");
	//输出出运行结果提示
	cout << "已使用K近邻完成数据训练和保存" << endl;
	waitKey(0);
}
//只能识别黑底白字数字0~9
void test64()	//测试训练的模型
{
	using namespace cv::ml;
	//加载KNN分类器
	Mat data = imread("../所有数据按行排列结果.png", IMREAD_ANYDEPTH);
	Mat labels = imread("../标签.png", IMREAD_ANYDEPTH);
	data.convertTo(data, CV_32FC1);
	labels.convertTo(labels, CV_32SC1);
	Ptr<KNearest> knn = Algorithm::load<KNearest>("../knn_model.ymal");
	//查看分类结果
	Mat result;
	knn->findNearest(data, 5, result);
	//统计分类结果与真实结果相同的数目
	int count = 0;
	for (int row = 0; row < result.rows; row++)
	{
		int predict = result.at<float>(row, 0);
		if (labels.at<int>(row, 0) == predict)
		{
			count++;
		}
	}
	float rate = 1.0 * count / result.rows;
	cout << "分类的正确性：" << rate << endl;
	//测试新图像是否能识别数字
	Mat testImage1 = imread("../handwritingtest1.png", IMREAD_GRAYSCALE);
	Mat testImage2 = imread("../handwritingtest2.png", IMREAD_GRAYSCALE);
	imshow("testImage1", testImage1);
	imshow("testImage2", testImage2);
	//缩放到20*20的尺寸
	resize(testImage1, testImage1, Size(20, 20));
	resize(testImage2, testImage2, Size(20, 20));
	Mat testdata = Mat::zeros(2, 400, CV_8UC1);
	Rect rect;
	rect.x = 0;
	rect.y = 0;
	rect.height = 1;
	rect.width = 400;
	Mat oneData = testImage1.reshape(1, 1);
	Mat twoData = testImage2.reshape(1, 1);
	oneData.copyTo(testdata(rect));
	rect.y = 1;
	twoData.copyTo(testdata(rect));
	//数据类型转换
	testdata.convertTo(testdata, CV_32F);
	//进行估计识别
	Mat result2;
	knn->findNearest(testdata, 5, result2);
	//查看预测结果
	for (int i = 0; i < result2.rows; i++)
	{
		int predict = result2.at<float>(i, 0);
		cout << "第" << i + 1 << "图像预测结果：" << predict
			<< "  真实结果：" << i + 1 << endl;
	}
	waitKey(0);
}
//支持向量积的分类方法
void test65()
{
	using namespace cv::ml;
	//训练数据
	Mat samples, labels;
	FileStorage fread("../point.yml", FileStorage::READ);
	fread["data"] >> samples;
	fread["labls"] >> labels;
	fread.release();
	//不同种类坐标拥有不同颜色
	vector<Vec3b> colors;
	colors.push_back(Vec3b(0, 255, 0));
	colors.push_back(Vec3b(0, 0, 255));
	//创建空白图像用于显示坐标点
	Mat image(480, 640, CV_8UC3, Scalar(255, 255, 255));
	Mat image_;
	image.copyTo(image_);
	//在空白图像中绘制坐标点
	for (int i = 0; i < samples.rows; i++)
	{
		Point2f point;
		point.x = samples.at<float>(i, 0);
		point.y = samples.at<float>(i, 1);
		Scalar color = colors[labels.at<int>(i, 0)];
		circle(image, point, 3, color, -1);
		circle(image_, point, 3, color, -1);
	}
	imshow("两类像素点图像", image);
	//建立模型
	Ptr<SVM> model = SVM::create();
	//参数设置
	model->setKernel(SVM::INTER);	//设置内核
	model->setType(SVM::C_SVC);	//设置SVM类型
	model->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 100, 0.01));
	model->setGamma(5.383);
	model->setC(0.01);
	model->setDegree(3);
	//训练模型
	model->train(TrainData::create(samples, ROW_SAMPLE, labels));
	//用模型对图像中的全部像素点进行分类
	Mat imagePoint(1, 2, CV_32FC1);
	int y = 0; int x = 0;
	for (; y < image_.rows; y += 2)
	{
		for (x = 0; x < image_.cols; x += 2)
		{
			imagePoint.at<float>(0) = (float)x;
			imagePoint.at<float>(1) = (float)y;
			int color = (int)model->predict(imagePoint);
			image_.at<Vec3b>(y, x) = colors[color];
		}
	}
	imshow("图像所有像素点分类结果", image_);
	waitKey();
}

int main()
{
	//test01();	//Mat类
	//test02();	//图像读取显示与保存
	//test03();	//视频加载与摄像头使用
	//test04();	//颜色空间变换
	//test05();	//多通道的分离与合并
	//test06();	//像素比较
	//test07();	//逻辑操作
	//test08();	//图像二值化
	//test09();	//LUT查找表
	//test10();	//尺寸变换
	//test11();	//仿射变换
	//test12();	//透视变换
	//test13();	//绘制基础图形
	//test14();	//ROI区域截取
	//test15();	//高斯&拉普拉斯图像金字塔
	//test16();	//滑动条
	//test17();	//鼠标响应
	//test18();	//直方图绘制
	//test19();	//直方图均衡化
	//test20();	//直方图匹配
	//test21();	//模板匹配
	//test22();	//图像的卷积
	//test23();	//椒盐噪声和高斯噪声
	//test24();	//椒盐噪声和高斯噪声
	//test25();	//中值滤波
	//test26();	//滤波的叠加性
	//test27();	//Soble和Scharr边缘检测算子
	//test28();	//拉普拉斯和Canny边缘检测
	//test29();	//连通域分析
	//test30();	//图像距离变化
	//test31();	//图像腐蚀操作
	//test32();	//图像膨胀操作
	//test33();	//对图像进行形态学操作
	//test34();	//图像细化
	//test35();	//轮廓检测
	//test36();	//轮廓信息统计
	//test37();	//轮廓外接多边形
	//test38();	//凸包检测
	//test39();	//直线检测
	//test40();	//直线拟合
	//test41();	//二维码识别
	//test42();	//图像的积分求和
	//test43();	//漫水填充法
	//test44();	//分水岭法
	//test45();	//Harris角点
	//test46();	//Shi-Tomas角点检测
	//test47();	//亚像素级别角点优化
	//test48(); //ORB 特征点
	//test49();	//特征点匹配
	//test50();	//RANSCA 优化特征点匹配
	//test51();	//相机模型与投影
	//test52();	//单目相机的标定
	//test53();	//图像的校正
	//test54();	//单目相机位姿估计
	//test55();	//差值检测移动物体
	//test56();	//稠密光流法跟踪移动物体
	//test57();	//稀疏光流法跟踪移动物体
	//test58();	//K均值聚类
	//test59();	//加载深度神经网络模型
	//深度神经网络模型的使用
	//test60();	//物体检测
	//test61();	//图像风格化
	//test62();	//人脸识别和性别判别
	//test63();	//监督学习聚类
	//test64();	//测试训练模型
	test65();	//向量积

	return 0;
}

void callBack(int value, void*)
{
	float a = value / 100.0;
	Mat img_ = img * a;
	imshow("img", img_);
}

void mouse(int event, int x, int y, int flags, void*)
{
	if (event == EVENT_RBUTTONDOWN)	//单击鼠标右键
	{
		cout << "点击鼠标左键才可绘制图像！" << endl;
	}
	if (event == EVENT_LBUTTONDOWN)	//单击鼠标左键，输出坐标
	{
		prePoint = Point(x, y);
		cout << "轨迹起始坐标：" << prePoint << endl;
	}
	
	if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON))	//按住鼠标左键移动
	{
		//通过绘制直线显示鼠标移动轨迹
		Point pt(x, y);
		line(img, prePoint, pt, Scalar(255, 255, 255), 4, 5, 0);
		prePoint = pt;
		imshow("窗口1", img);
		//通过改变图像像素绘制显示鼠标移动轨迹
		imgPoint.at<Vec3b>(y, x) = Vec3b(255, 255, 255);
		imgPoint.at<Vec3b>(y, x - 1) = Vec3b(255, 255, 255);
		imgPoint.at<Vec3b>(y, x + 1) = Vec3b(255, 255, 255);
		imgPoint.at<Vec3b>(y - 1, x) = Vec3b(255, 255, 255);
		imgPoint.at<Vec3b>(y + 1, x) = Vec3b(255, 255, 255);
		imshow("窗口2", imgPoint);
	}
}

void drawHist(Mat& gray, Mat& hist, string name)
{
	//计算绘制直方图
	int hist_w = 512;
	int hist_h = 400;
	int width = 2;
	Mat histImg = Mat::zeros(hist_h, hist_w, CV_8UC3);
	//for (int i = 1; i <= hist.rows; i++)
	//{
	//	rectangle(histImg, Point(width * (i - 1), hist_h - 1),
	//		Point(width * i - 1, hist_h - cvRound(hist.at<float>(i - 1) / 60.0)),
	//		Scalar(255, 255, 255), -1);
	//}
	//归一化处理数据
	Mat hist_INF;
	normalize(hist, hist_INF, 1, 0, NORM_L1, -1, Mat());
	for (int i = 1; i <= hist_INF.rows; i++)
	{
		rectangle(histImg, Point(width * (i - 1), hist_h - 1),
			Point(width * i - 1, hist_h - cvRound(40 * hist_h * hist_INF.at<float>(i - 1))),
			Scalar(255, 255, 255), -1);
	}
	namedWindow(name, WINDOW_NORMAL);
	imshow(name, gray);
	imshow(name + string("hist"), histImg);
}

void saltAndPepper(Mat& image, int n)
{
	cvflann::seed_random((unsigned)time(NULL));
	for (int k = 0; k < n; k++)
	{
		int i = cvflann::rand_int() % image.rows;	//获得随机噪声的行数
		int j = cvflann::rand_int() % image.cols;	//获得随机噪声的列数
		if (image.type() == CV_8UC1)	//灰色图像添加噪声
		{
			int flag = cvflann::rand_int() % 2;	//噪声的颜色
			if (flag == 0)	//添加黑色噪声
				image.at<uchar>(i, j) = 0;
			else	//灰色图像添加白色噪声
				image.at<uchar>(i, j) = 255;
		}
		else
		{
			int channels = cvflann::rand_int() % 8;	//7(111)代表三个通道均为白色
			int B = (channels & 4) >> 2;	//判断蓝色通道为0还是255
			int G = (channels & 2) >> 1;	//判断绿色通道为0还是255
			int R = channels & 1;			//判断红色通道为0还是255
			image.at<Vec3b>(i, j)[0] = B * 255;
			image.at<Vec3b>(i, j)[1] = G * 255;
			image.at<Vec3b>(i, j)[2] = R * 255;
		}
	}
}

void drawStatus(Mat& image, Mat& stats, Mat& centroids, int number)
{
	RNG rng((unsigned)time(NULL));
	vector<Vec3b> colors_new;
	for (int i = 0; i < number; i++)
	{
		//用随机数确定颜色
		Vec3b v3 = Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		colors_new.push_back(v3);
	}
	//用不同颜色标记连通域
	for (int i = 0; i < number; i++)
	{
		//每块连通域中心坐标
		int center_x = centroids.at<double>(i, 0);
		int center_y = centroids.at<double>(i, 1);
		//每块连通域的其他信息
		int x = stats.at<int>(i, CC_STAT_LEFT);
		int y = stats.at<int>(i, CC_STAT_TOP);
		int width = stats.at<int>(i, CC_STAT_WIDTH);
		int height = stats.at<int>(i, CC_STAT_HEIGHT);
		int area = stats.at<int>(i, CC_STAT_AREA);
		//圈出中心位置
		circle(image, Point(center_x, center_y), 2, Scalar(0, 255, 0), 2, 8, 0);
		//外接矩形
		Rect rect(x, y, width, height);
		rectangle(image, rect, colors_new[i], 1, 8, 0);
		putText(image, format("%d", i), Point(center_x, center_y), FONT_HERSHEY_COMPLEX,
			0.5, Scalar(0, 0, 255));
		cout << "number: " << i << "area: " << area << endl;
	}
}

void drawapp(Mat& input, Mat& image)
{
	for (int p = 0; p < input.rows; p++)
	{
		//绘制多边形
		Vec2i pre = input.at<Vec2i>(p);
		Vec2i post = input.at<Vec2i>((p + 1) % input.rows);	//pre为最后一个点时，post变为第零个点
		line(image, pre, post, Scalar(0, 0, 255), 2, 8, 0);
	}

}

void drawLines(Mat& image, vector<Vec2f> lines, Scalar scalar, int thickness)
{
	//image绘制直线的图形，lines绘制直线的数据， 图像行，列， 绘制直线的颜色， 线宽
	Point p1, p2;
	for (int i = 0; i < lines.size(); i++)
	{
		double rho = lines[i][0];	//直线距离左边原点的距离
		double theta = lines[i][1];	//直线过坐标原点垂线与x轴的夹角
		double a = cos(theta);	//夹角的余弦值
		double b = sin(theta);	//夹角的正弦值
		double x0 = rho * a, y0 = rho * b;	//过原点垂线与直线的交点
		double length = max(image.rows, image.cols);	//图像高度的最大值
		//直线上的一点(尽可能里离焦点远)
		p1.x = cvRound(x0 + length * (-b));
		p1.y = cvRound(y0 + length * a);
		//直线上的另一点(尽可能里离焦点远)
		p2.x = cvRound(x0 - length * (-b));
		p2.y = cvRound(y0 - length * a);
		//两点一线
		line(image, p1, p2, scalar, thickness);
	}
}

void orb_features(Mat& image, vector<KeyPoint>& keyPoints, Mat& descriptions)
{
	//创建 ORB 特征点类变量
	Ptr<ORB> orb = ORB::create(1000,	//特征点数目
		1.2f,	//金字塔层级之间的缩放比例
		8,	//金字塔层数系数
		31,	//边缘阈值
		0,	//原图在金字塔中的层数
		2,	//生成描述子时需要用的像素点数目
		ORB::HARRIS_SCORE,	//使用 HARRIS 方法评价特征点
		31,	//生成描述点时关键点周围邻域尺寸
		20	//计算 FAST 角点时像素值差值的阈值
	);
	//计算关键点
	orb->detect(image, keyPoints);
	//计算 ORB 描述子
	orb->compute(image, keyPoints, descriptions);
}

void match_min(vector<DMatch>& matches, vector<DMatch>& good_matches)
{
	//通过汉明距离筛选匹配结果
	double min_dist = 10000, max_dist = 0;
	for (int i = 0; i < matches.size(); i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist)	min_dist = dist;
		if (dist > max_dist)	max_dist = dist;
	}
	//输出最大和最小的汉明距离
	cout << "min_dist = " << min_dist << endl;
	cout << "max_dist = " << max_dist << endl;
	//将汉明距离较远的删除
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 20.0))
			good_matches.push_back(matches[i]);
	}
	cout << "good_matches = " << good_matches.size() << endl;	//剩余特征点数目
}

void ransac(vector<DMatch>& matches, vector<KeyPoint>& queryKeyPoint, vector<KeyPoint>& trainKeyPoint, vector<DMatch>& matches_ransac)
{
	//定义保存匹配点对坐标
	vector<Point2f> srcPoints(matches.size()), dstPoints(matches.size());
	//保存从关键点中提取的匹配点对坐标
	for (int i = 0; i < matches.size(); i++)
	{
		srcPoints[i] = queryKeyPoint[matches[i].queryIdx].pt;
		dstPoints[i] = trainKeyPoint[matches[i].trainIdx].pt;
	}
	//匹配点对进行ransac过滤
	vector<int> inliersMask(srcPoints.size());
	//Mat homography;
	//homography = findHomography(srcPoints, dstPoints, RANSAC, 5, inliersMask);
	findHomography(srcPoints, dstPoints, RANSAC, 5, inliersMask);
	//手动保留RANSAC过滤后的匹配点对
	for (int i = 0; i < inliersMask.size(); i++)
	{
		if (inliersMask[i])
			matches_ransac.push_back(matches[i]);
	}
	cout << "good_ransac = " << matches_ransac.size() << endl;
}

void undist(vector<Mat> images, Mat cameraMatrix, Mat distCoeffs, vector<Mat>& undistImages)
{
	for (int i = 0; i < images.size(); i++)
	{
		Mat undistImage;
		undistort(images[i], undistImage, cameraMatrix, distCoeffs);
		undistImages.push_back(undistImage);
	}
}

void draw_lines(Mat& image, vector<Point2f> pt1, vector<Point2f> pt2)
{
	RNG rng((unsigned)time(NULL));
	if (color_lut.size() < pt1.size())
	{
		for (int i = 0; i < pt1.size(); i++)
		{
			color_lut.push_back(Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256)));
		}
	}
	for (int i = 0; i < pt1.size(); i++)
	{
		line(image, pt1[i], pt2[i], color_lut[i], 2, 8);
	}
}