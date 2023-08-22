#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

using namespace cv;
using namespace std;

//Mat��
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
//ͼ���ȡ��ʾ�뱣��
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
//��Ƶ����������ͷʹ��
void test03()
{
	/*VideoCapture video;
	video.open(0);
	if (!video.isOpened())
	{
		cout << "��Ƶ��ʧ��!" << endl;
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
		cout << "��Ƶ֡��=" << camera_fps << endl;
		cout << "��Ƶ���=" << video.get(CAP_PROP_FRAME_WIDTH) << endl;

		uchar input = waitKey(1000 / camera_fps);
		if (input == 27)
			break;
	}*/

	Mat img;
	VideoCapture video(0);	//��������ͷ
							//��ȡ��Ƶ video.open(filename)
	if (!video.isOpened())	//�ж�����ͷ��ͼ���Ƿ��
	{
		cout << "����ͷδ�򿪣�" << endl;
		return;
	}

	video >> img;	//�ж��Ƿ��ȡͼ��
	if (img.empty())
	{
		cout << "û�л�ȡͼ��" << endl;
		return;
	}

	bool isColor = (img.type() == CV_8UC3);	//�ж�����Ƿ�Ϊ��ɫ
	VideoWriter writer;
	int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');	//ѡ����Ƶ�����ʽ

	double fps = 10.0;	//������Ƶ���֡��
	string filename = "../live.avi";	//������Ƶ�ļ���

	writer.open(filename, codec, fps, img.size(), isColor);	//����������Ƶ�ļ�����Ƶ��

	if (!writer.isOpened()) //�ж���Ƶ���Ƿ񴴽��ɹ�
	{
		cout << "��Ƶ������ʧ�ܣ�" << endl;
		return;
	}

	while (true)
	{
		if (!video.read(img))	//�ж��Ƿ��ܶ�����һ֡ͼ��
		{
			cout << "����ͷ�Ͽ����ӻ���Ƶ��ȡ��ϣ�" << endl;
			break;
		}
		writer.write(img); //��ͼ��д����Ƶ��//writer << img

		imshow("live", img); //��ʾͼ��
		char input = waitKey(50);
		if (input == 27)	//��Esc���沢�˳�
			break;
	}

	video.release();		//�ر���Ƶ��
	writer.release();

}
//��ɫ�ռ�任
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
//��ͨ���ķ�����ϲ�
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
//���رȽ�
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
//�߼�����
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
//ͼ���ֵ��
void test08()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	if (img.empty())
		exit(-1);

	imshow("atri", img);
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	Mat img_B, img_B_I, gray_B, gray_B_I, gray_T, gray_T_I, gray_TRUNC;
	//��ɫͼ���ֵ��
	threshold(img, img_B, 125, 255, THRESH_BINARY);
	threshold(img, img_B_I, 125, 255, THRESH_BINARY_INV);
	imshow("atriB", img_B);
	imshow("atriBI", img_B_I);
	//�Ҷ�ͼ���ֵ��
	threshold(gray, gray_B, 125, 255, THRESH_BINARY);
	threshold(gray, gray_B_I, 125, 255, THRESH_BINARY_INV);
	imshow("atriGB", gray_B);
	imshow("atriGBI", gray_B_I);
	//�Ҷ�ͼ��TOZERO�任
	threshold(gray, gray_T, 125, 255, THRESH_TOZERO);
	threshold(gray, gray_T_I, 125, 255, THRESH_TOZERO_INV);
	imshow("atriT", gray_T);
	imshow("atriTI", gray_T_I);
	//�Ҷ�ͼ��TRUNC�任
	threshold(gray, gray_TRUNC, 125, 255, THRESH_TRUNC);
	imshow("atriTR", gray_TRUNC);
	//�Ҷ�ͼ���򷨺������η���ֵ��
	Mat img_Thr = imread("F:/study/code/Opencv_C++_Learn/sonnet.png", IMREAD_GRAYSCALE);
	Mat img_Thr_O, img_Thr_T;
	threshold(img_Thr, img_Thr_O, 100, 255, THRESH_BINARY | THRESH_OTSU);
	threshold(img_Thr, img_Thr_T, 100, 255, THRESH_BINARY | THRESH_TRIANGLE);
	imshow("img_Thr", img_Thr);
	imshow("img_Thr_O", img_Thr_O);
	imshow("img_Thr_T", img_Thr_T);
	//��ֵ���͸�˹��
	Mat adaptive_mean, adaptive_gauss;
	adaptiveThreshold(img_Thr, adaptive_mean, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 55, 0);
	adaptiveThreshold(img_Thr, adaptive_gauss, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 55, 0);
	imshow("adaptive_mean", adaptive_mean);
	imshow("adaptive_gauss", adaptive_gauss);

	waitKey(0);
}
//LUT���ұ�
void test09()
{
	//LUT���ұ��һ��
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
	//LUT���ұ�ڶ���
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
	//LUT���ұ������
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
	//��ͨ����LUT���ұ�
	vector<Mat> Mergelut;
	Mergelut.push_back(lutOne);
	Mergelut.push_back(lutTwo);
	Mergelut.push_back(lutThree);
	Mat lutTree;
	merge(Mergelut, lutTree);
	//����ͼ��Ĳ��ұ�
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
//�ߴ�任
void test10()
{
	Mat gray = imread("F:/study/code/Opencv_C++_Learn/atri.jpg", IMREAD_GRAYSCALE);
	Mat small, big0, big1, big2;
	resize(gray, small, Size(30, 30), 0, 0, INTER_AREA);	//��Сͼ��
	resize(small, big0, Size(60, 60), 0, 0, INTER_NEAREST);	//����ڲ�ֵ
	resize(small, big1, Size(60, 60), 0, 0, INTER_LINEAR);	//˫���Բ�ֵ
	resize(small, big2, Size(60, 60), 0, 0, INTER_CUBIC);	//˫���β�ֵ
	//ͼ��ת
	Mat img_x, img_y, img_xy;
	flip(gray, img_x, 0);	//x��Գ�
	flip(gray, img_y, 1);	//y��Գ�
	flip(gray, img_xy, -1);	//x��Գƣ�y��Գ�
	//��ȡ�ĸ���ͼ��
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	Mat img00 = img(Range(0, img.rows / 2), Range(0, img.cols / 2));
	Mat img01 = img(Range(0, img.rows / 2), Range(img.cols / 2, img.cols));
	Mat img10 = img(Range(img.rows / 2, img.rows), Range(0, img.cols / 2));
	Mat img11 = img(Range(img.rows / 2, img.rows), Range(img.cols / 2, img.cols));
	//����ͼ��ƴ��
	Mat img_, img0, img1;
	hconcat(img00, img01, img0);	//��������
	hconcat(img10, img11, img1);	//��������
	vconcat(img0, img1, img_);		//��������

	waitKey(0);
}
//����任
void test11()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	if (img.empty())
		exit(-1);

	Mat rotation0, img_warp0;
	double angle = 30.0;	//������ת�Ƕ�
	Size dst_size(img.cols, img.rows);	//�����������ͼ���С
	Point2f center(img.cols / 2.0, img.rows / 2.0);	//������ת����
	rotation0 = getRotationMatrix2D(center, angle, 0.5);	//��÷���任����
	warpAffine(img, img_warp0, rotation0, dst_size);	//��ԭͼ����з���任
	imshow("img_warp0", img_warp0);

	Point2f src_points[3], dst_points[3];
	//ԭͼ��������
	src_points[0] = Point2f(0, 0);
	src_points[1] = Point2f(0, img.rows - 1);
	src_points[2] = Point2f(img.cols - 1, img.rows - 1);
	//�任��ͼ���Ӧ��������
	dst_points[0] = Point2f(img.cols * 0.71, img.rows * 0.21);
	dst_points[1] = Point2f(img.cols * 0.12, img.rows * 0.01);
	dst_points[2] = Point2f(img.cols * 0.19, img.rows * 0.85);

	Mat rotation1, img_warp1;
	rotation1 = getAffineTransform(src_points, dst_points);	//��÷���任����
	warpAffine(img, img_warp1, rotation1, dst_size);	//��ͼ����з���任

	imshow("img_warp1", img_warp1);
	waitKey(0);
}
//͸�ӱ任
void test12()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/IMG_20230810_105155.jpg");
	if (img.empty())
		exit(-1);
	
	Point2f src_points[4], dst_points[4];
	//��ȡԭͼ���������ĵ�����
	src_points[0] = Point2f(374.0, 555.0);
	src_points[1] = Point2f(2038.0, 587.0);
	src_points[2] = Point2f(56.0, 1361.0);
	src_points[3] = Point2f(2277.0, 1437.0);
	//���ñ任���Ӧ�ĵ������
	dst_points[0] = Point2f(0, 0);
	dst_points[1] = Point2f(1000.0, 0);
	dst_points[2] = Point2f(0, 600.0);
	dst_points[3] = Point2f(1000.0, 600.0);
	
	Mat rotation, img_warp;
	//��ȡ͸�ӱ任�����Լ���ͼ�����͸�ӱ任
	rotation = getPerspectiveTransform(src_points, dst_points);
	warpPerspective(img, img_warp, rotation, Size(1000, 600));

	namedWindow("img", WINDOW_NORMAL);
	imshow("img", img);
	imshow("img_warp", img_warp);

	waitKey(0);
}
//���ƻ���ͼ��
void test13()
{
	Mat img = Mat::zeros(Size(512, 512), CV_8UC3);	//���ɺ�ɫ�������ڻ���ͼ��
	//����Բ��
	circle(img, Point(21, 31), 15, Scalar(255, 0, 0), -1);	//ʵ��Բ
	circle(img, Point(62, 31), 20, Scalar(255, 255, 0), 3); //����Բ
	//����ֱ��
	line(img, Point(23, 500), Point(450, 90), Scalar(0, 0, 255), 2, LINE_4, 0);
	//������Բ
	ellipse(img, Point(200, 300), Size(70, 100), 0, 0, 360, Scalar(0, 255, 0)); //ʵ����Բ
	//���ƾ���
	rectangle(img, Point(100, 150), Point(200, 300), Scalar(0, 255, 255));	//����
	//���ƶ����
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

	const Point* pts[3] = { pp[0], pp[1], pp2 };	//pts��������
	int npts[] = { 6, 6, 5 };	//�����������
	fillPoly(img, pts, npts, 3, Scalar(125, 125, 125), 8);	//�������������
	
	//����ı�
	putText(img, "LearnOpenCV", Point(100, 400), 2, 1, Scalar(255, 200, 124));

	//���ͼ��
	imshow("", img);
	waitKey(0);
}
//ROI�����ȡ
void test14()
{
	Mat img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	Mat noobcv = imread("F:/study/code/Opencv_C++_Learn/black.png");

	Mat ROI1, ROI2, ROI2_copy, mask, img2, img_copy;
	resize(noobcv, mask, Size(200, 200));
	img2 = img;	//ǳ����
	//���
	img.copyTo(img_copy);
	//����ͼ�н�ȡROI����ķ�ʽ
	Rect rect(206, 206, 200, 200);	//����ROI����
	ROI1 = img(rect);	//��ͼ

	ROI2 = img(Range(300, 500), Range(300, 500));	//�ڶ��ֽ�ͼ��ʽ
	img(Range(300, 500), Range(300, 500)).copyTo(ROI2_copy);	//���

	mask.copyTo(ROI1);	//��ͼ���м��벿��ͼ��

	imshow("����noobcv��ͼ��", img);
	imshow("�����img_copy", img_copy);
	imshow("ROI1��ROI2��Ӱ��", ROI2);
	imshow("�����ROI2", ROI2_copy);

	circle(img, Point(300, 200), 20, Scalar(0, 0, 255), -1);	//����һ��Բ��

	imshow("ǳ������img2", img2);
	imshow("��Բ��ROI1��Ӱ��", ROI1);
	waitKey(0);
}
//��˹&������˹ͼ�������
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
//������
Mat img, imgPoint;	//ȫ��ͼ��
void callBack(int value, void*);

void test16()
{
	img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	namedWindow("img", WINDOW_NORMAL);
	imshow("img", img);

	int value = 100;
	createTrackbar("�ٷֱ�", "img", &value, 600, callBack);
	waitKey(0);
}
//�����Ӧ
Point prePoint;	//ǰһ��������꣬���ڻ���ͼ��
void mouse(int event, int x, int y, int flags, void*);

void test17()
{
	img = imread("F:/study/code/Opencv_C++_Learn/atri.jpg");
	img.copyTo(imgPoint);
	namedWindow("����1", WINDOW_NORMAL);
	namedWindow("����2", WINDOW_NORMAL);
	imshow("����1", img);
	imshow("����2", imgPoint);

	setMouseCallback("����1", mouse, 0);	//���Ӱ��
	waitKey(0);
}
//ֱ��ͼ����
void drawHist(Mat& gray, Mat& hist, string name);

void test18()
{
	Mat gray = imread("F:/study/code/Opencv_C++_Learn/atri.jpg", IMREAD_GRAYSCALE);
	Mat hist;	//���ڴ��ֱ��ͼͳ�ƽ��
	const int channels[1] = { 0 };	//ͼ���ͨ����
	const int bins[256] = { 256 };	//ֱ��ͼ��ά�ȼ�ͼ��ĻҶ����ֵ
	float inRanges[2] = { 0, 256 };	//���ػҶȷ�Χ
	const float* ranges[1] = { inRanges };	//ÿһ��ͼƬ�ĻҶȷ�Χ
	calcHist(&gray, 1, channels, Mat(), hist, 1, bins, ranges);	//����ͼ��Ҷ�ֱ��ͼ
	drawHist(gray, hist, "gray");
	waitKey(0);
}
//ֱ��ͼ���⻯
void test19()
{
	Mat gray = imread("F:/study/code/Opencv_C++_Learn/lena.jpg", IMREAD_GRAYSCALE);
	Mat hist, eqhist;	//���ڴ��ֱ��ͼͳ�ƽ��
	Mat equalgray;	//��ž��⻯��ĻҶ�ͼ��
	equalizeHist(gray, equalgray);
	const int channels[1] = { 0 };	//ͼ���ͨ����
	const int bins[256] = { 256 };	//ֱ��ͼ��ά�ȼ�ͼ��ĻҶ����ֵ
	float inRanges[2] = { 0, 256 };	//���ػҶȷ�Χ
	const float* ranges[1] = { inRanges };	//ÿһ��ͼƬ�ĻҶȷ�Χ
	calcHist(&gray, 1, channels, Mat(), hist, 1, bins, ranges);	//����ͼ��Ҷ�ֱ��ͼ
	calcHist(&equalgray, 1, channels, Mat(), eqhist, 1, bins, ranges);	//������⻯��ͼ��Ҷ�ֱ��ͼ
	drawHist(gray, hist, "gray");
	drawHist(equalgray, eqhist, "equalgray");
	imwrite("F:/study/code/Opencv_C++_Learn/eqlena.jpg", equalgray);
	waitKey(0);
}
//ֱ��ͼƥ��
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
	//����ÿ��ͼ����ۼƸ��ʾ���
	float* hist1_cdf = new float[256] { hist1.at<float>(0) };
	float* hist2_cdf = new float[256] { hist2.at<float>(0) };
	for (int i = 1; i < 256; i++)
	{
		hist1_cdf[i] = hist1_cdf[i - 1] + hist1.at<float>(i);
		hist2_cdf[i] = hist2_cdf[i - 1] + hist2.at<float>(i);
	}
	//�����ۻ����ʲ�ֵ����
	float(*diff_cdf)[256] = new float[256][256]{ 0 };
	for (int i = 0; i < 256; i++)
	{
		for (int j = 0; j < 256; j++)
			diff_cdf[i][j] = fabs(hist1_cdf[i] - hist2_cdf[j]);
	}
	//����LUTӳ���
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
//ģ��ƥ��
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
//ͼ��ľ��
void test22()
{
	uchar points[25] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
						16, 17, 18, 19, 20, 21, 22, 23, 24, 25 };
	//������ľ���
	Mat matrix = Mat(5, 5, CV_8UC1, points);
	//���ģ��
	Mat kernal = (Mat_<float>(3, 3) << 1, 2, 1,
		2, 0, 2,
		1, 2, 1);
	//��һ��
	Mat kernal_norm = kernal / 12.0;
	Mat result, result_norm;
	//�Ծ�����о��
	filter2D(matrix, result, CV_32F, kernal, Point(-1, -1), 2, BORDER_CONSTANT);
	filter2D(matrix, result_norm, CV_32F, kernal_norm, Point(-1, -1), 2, BORDER_CONSTANT);
	//�������
	cout << "result:>\n" << result << endl;
	cout << "result_norm:>\n" << result_norm << endl;
	//��ͼ����о��
	Mat lena = imread("../lena.jpg");
	if (lena.empty())
		exit(-1);
	Mat lena_filter;
	filter2D(lena, lena_filter, -1, kernal_norm, Point(-1, -1), 2, BORDER_CONSTANT);
	imshow("lena", lena);
	imshow("lena_filter", lena_filter);
	waitKey(0);
}
//���������͸�˹����
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
	
	imshow("ԭͼ1", lena);
	imshow("ԭͼ2", eqlena);
	//��ӽ�������
	saltAndPepper(lena, 10000);
	saltAndPepper(eqlena, 10000);
	//��ʾ������ͼ��
	imshow("��������1", lena);
	imshow("��������2", eqlena);
	waitKey(0);
	//��˹����
	Mat lena_noise = Mat::zeros(lena_G.rows, lena_G.cols, lena_G.type());
	Mat eqlena_noise = Mat::zeros(eqlena_G.rows, eqlena_G.cols, eqlena_G.type());
	
	imshow("ԭͼ1", lena_G);
	imshow("ԭͼ2", eqlena_G);
	RNG rng;	//����rng��
	rng.fill(lena_noise, RNG::NORMAL, 10, 20);	//������ͨ����˹�ֲ������
	rng.fill(eqlena_noise, RNG::NORMAL, 10, 20);	//���ɵ�ͨ����˹�ֲ������
	
	imshow("��ͨ����˹����", lena_noise);
	imshow("��ͨ����˹����", eqlena_noise);

	lena_G = lena_G + lena_noise; //�ڲ�ɫͼ������Ӹ�˹����
	eqlena_G = eqlena_G + eqlena_noise;	//�ڻ�ɫͼ������Ӹ�˹����

	imshow("��˹����1", lena_G);
	imshow("��˹����2", eqlena_G);

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

	Mat result_3, result_5;	//��Ų����������˲�������������ֱ�ʾ�˲����ߴ�
	Mat result_3salt, result_5salt;	//��ź��н��������˲�������������ֱ�ʾ�˲����ߴ�
	Mat result_3gauss, result_5gauss;	//��ź��и�˹�����˲�������������ֱ�ʾ�˲����ߴ�
	//���þ�ֵ�˲�����blur()�����˲�
	blur(eqlena, result_3, Size(3, 3));
	blur(eqlena, result_5, Size(5, 5));
	blur(eqlena_salt, result_3salt, Size(3, 3));
	blur(eqlena_salt, result_5salt, Size(5, 5));
	blur(eqlena_gauss, result_3gauss, Size(3, 3));
	blur(eqlena_gauss, result_5gauss, Size(5, 5));
	//��ʾ����������ͼ��
	imshow("eqlena", eqlena);
	imshow("result_3", result_3);
	imshow("result_5", result_5);
	//��ʾ������������ͼ��
	imshow("eqlena_salt", eqlena_salt);
	imshow("result_3salt", result_3salt);
	imshow("result_5salt", result_5salt);
	//��ʾ����˹������ͼ��
	imshow("eqlena_gauss", eqlena_gauss);
	imshow("result_3gauss", result_3gauss);
	imshow("result_5gauss", result_5gauss);
	waitKey(0);
	//���÷����˲���������ͼ��
	Mat result, result_Norm;
	boxFilter(eqlena, result, -1, Size(3, 3), Point(-1, -1), false);	//�����й�һ��
	boxFilter(eqlena, result_Norm, -1, Size(3, 3), Point(-1, -1), true);	//���й�һ��
	//��ʾ������
	imshow("result", result);
	imshow("result_Norm", result_Norm);
	waitKey(0);
	//��˹�˲�
	Mat result_3_G, result_5_G;	//��Ų����������˲�������������ֱ�ʾ�˲����ߴ�
	Mat result_3_G_salt, result_5_G_salt;	//��ź��н��������˲�������������ֱ�ʾ�˲����ߴ�
	Mat result_3_G_gauss, result_5_G_gauss;	//��ź��и�˹�����˲�������������ֱ�ʾ�˲����ߴ�
	//��˹�˲�����
	GaussianBlur(eqlena, result_3_G, Size(3, 3), 10, 50);
	GaussianBlur(eqlena, result_5_G, Size(5, 5), 10, 20);
	GaussianBlur(eqlena_salt, result_3_G_salt, Size(3, 3), 10, 20);
	GaussianBlur(eqlena_salt, result_5_G_salt, Size(5, 5), 10, 20);
	GaussianBlur(eqlena_gauss, result_3_G_gauss, Size(3, 3), 10, 20);
	GaussianBlur(eqlena_gauss, result_5_G_gauss, Size(5, 5), 10, 20);
	//��ʾ����������ͼ��
	imshow("eqlena", eqlena);
	imshow("result_3_G", result_3_G);
	imshow("result_5_G", result_5_G);
	//��ʾ������������ͼ��
	imshow("eqlena_salt", eqlena_salt);
	imshow("result_3_G_salt", result_3_G_salt);
	imshow("result_5_G_salt", result_5_G_salt);
	//��ʾ����˹������ͼ��
	imshow("eqlena_gauss", eqlena_gauss);
	imshow("result_3_G_gauss", result_3_G_gauss);
	imshow("result_5_G_gauss", result_5_G_gauss);
	waitKey(0);
}
//��ֵ�˲�
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
	//��ֵ�˲�����
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
	//��ɫͼƬ
	imshow("lena", lena);
	imshow("result_3", result_3);
	imshow("result_5", result_5);
	//��ɫͼƬ
	imshow("eqlena", eqlena);
	imshow("eqresult_3", eqresult_3);
	imshow("eqresult_5", eqresult_5);
	//�����������Ĳ�ɫͼƬ
	imshow("lena_salt", lena_salt);
	imshow("result_3_salt", result_3_salt);
	imshow("result_5_salt", result_5_salt);
	//�����������Ļ�ɫͼƬ
	imshow("eqlena_salt", eqlena_salt);
	imshow("eqresult_3_salt", eqresult_3_salt);
	imshow("eqresult_5_salt", eqresult_5_salt);
	//����˹�����Ĳ�ɫͼƬ
	imshow("lena_gauss", lena_gauss);
	imshow("result_3_gauss", result_3_gauss);
	imshow("result_5_gauss", result_5_gauss);
	//����˹�����Ļ�ɫͼƬ
	imshow("eqlena_gauss", eqlena_gauss);
	imshow("eqresult_3_gauss", eqresult_3_gauss);
	imshow("eqresult_5_gauss", eqresult_5_gauss);

	waitKey(0);
}
//�˲��ĵ�����
void test26()
{
	float points[25] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
						16, 17, 18, 19, 20, 21, 22, 23, 24, 25 };
	Mat matrix = Mat(5, 5, CV_32FC1, points);
	//X����Y����������˲�
	Mat Y = (Mat_<float>(3, 1) << -1, 3, -1);
	Mat X = Y.reshape(1, 1);
	Mat XY = Y * X;
	//��֤�����˲��Ŀɷ�����
	Mat dataY, dataYX, dataXY, dataXY_sep;
	filter2D(matrix, dataY, -1, Y, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(dataY, dataYX, -1, X, Point(-1, -1), 0, BORDER_CONSTANT);
	filter2D(matrix, dataXY, -1, XY, Point(-1, -1), 0, BORDER_CONSTANT);

	sepFilter2D(matrix, dataXY_sep, -1, Y, Y, Point(-1, -1), 0, BORDER_CONSTANT);
	//����˲����
	cout << "matrix = \n" << matrix << endl;
	cout << "dataY = \n" << dataY << endl;
	cout << "dataYX = \n" << dataYX << endl;
	cout << "dataXY = \n" << dataXY << endl;
	cout << "dataXY_sep = \n" << dataXY_sep << endl;

	//��֤��˹�˲��Ŀɷ�����
	Mat gaussX = getGaussianKernel(3, 1);
	Mat gaussData, gaussDataXY;
	GaussianBlur(matrix, gaussData, Size(3, 3), 1, 1, BORDER_CONSTANT);
	sepFilter2D(matrix, gaussDataXY, -1, gaussX, gaussX, Point(-1, -1), 0, BORDER_CONSTANT);
	//��������˲����
	cout << "gaussData = \n" << gaussData << endl;
	cout << "gaussDataXY = \n" << gaussDataXY << endl;

	//ͼ������˲��Ŀɷ�����
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
//Soble��Scharr��Ե�������
void test27()
{
	Mat eqlena = imread("../lena.jpg", IMREAD_ANYDEPTH);
	Mat resultX, resultY, resultXY;
	//Soble��Ե���
	//X��Ե���
	Sobel(eqlena, resultX, CV_16S, 1, 0, 1);
	convertScaleAbs(resultX, resultX);
	//Y��Ե���
	Sobel(eqlena, resultY, CV_16S, 0, 1, 3);
	convertScaleAbs(resultY, resultY);
	//����ͼ��ı�Ե
	resultXY = resultX + resultY;
	//��ʾͼ��
	imshow("eqlena", eqlena);
	imshow("resultX", resultX);
	imshow("resultY", resultY);
	imshow("resultXY", resultXY);
	waitKey(0);
	//Scharr��Ե���
	//X��Ե���
	Scharr(eqlena, resultX, CV_16S, 1, 0);
	convertScaleAbs(resultX, resultX);
	//Y��Ե���
	Scharr(eqlena, resultY, CV_16S, 0, 1);
	convertScaleAbs(resultY, resultY);
	//����ͼ��ı�Ե
	resultXY = resultX + resultY;
	//��ʾͼ��
	imshow("eqlena", eqlena);
	imshow("resultX", resultX);
	imshow("resultY", resultY);
	imshow("resultXY", resultXY);
	waitKey(0);
	//���ɱ�Ե�����
	Mat soble_x, soble_y, soble_X1;
	Mat scharr_x, scharr_y, scharr_X1;
	//����һ��x����soble����
	getDerivKernels(soble_x, soble_y, 1, 0, 3);
	soble_x = soble_x.reshape(CV_8U, 1);
	//�����˲���
	soble_X1 = soble_y * soble_x;
	//����һ��x����scharr����
	getDerivKernels(scharr_x, scharr_y, 1, 0, -1);	//-1��������scharr����
	scharr_x = scharr_x.reshape(CV_8U, 1);
	//�����˲���
	scharr_X1 = scharr_y * scharr_x;
	//������
	cout << "һ��x����soble����:\n" << soble_X1 << endl;
	cout << "һ��x����scharr����:\n" << scharr_X1 << endl;
	waitKey(0);
}
//������˹��Canny��Ե���
void test28()
{
	Mat gray = imread("../GRAYGaussNoise.jpg", IMREAD_ANYDEPTH);
	Mat result, result_g, result_G;
	if (gray.empty())
		exit(-1);
	//������˹��Ե���
	//δ�����˲�ֱ����ȡ��Ե
	Laplacian(gray, result, CV_16S, 3, 1, 0);
	convertScaleAbs(result, result);
	//�˲�����ȡ��Ե
	GaussianBlur(gray, result_g, Size(3, 3), 5, 0);
	Laplacian(result_g, result_G, CV_16S, 3, 1, 0);
	convertScaleAbs(result_G, result_G);
	//��ʾ���
	imshow("gray", gray);
	imshow("result", result);
	imshow("result_G", result_G);
	waitKey(0);
	//Canny��Ե���
	Mat resultHigh, resultLow, resultG;
	//����ֵ���ͼ���Ե
	Canny(gray, resultHigh, 100, 200);
	//С��ֵ���ͼ���Ե
	Canny(gray, resultLow, 20, 40);
	//��˹ģ������ͼ���Ե
	GaussianBlur(gray, resultG, Size(3, 3), 5, 0);
	Canny(resultG, resultG, 100, 200);
	//��ʾ���
	imshow("resulHigh", resultHigh);
	imshow("resultLow", resultLow);
	imshow("resultG", resultG);
	waitKey(0);
}
//��ͨ�����
void drawStatus(Mat& image, Mat& stats, Mat& centroids, int number);

void test29()
{
	Mat image = imread("../rice.jpg");
	if (image.empty())
		exit(-1);
	//��ͼ��ת��Ϊ��ֵͼ������ͳ����ͨ��
	Mat rice, riceB;
	cvtColor(image, rice, COLOR_BGR2GRAY);
	threshold(rice, riceB, 100, 255, THRESH_BINARY);
	//���������ɫ�������ֲ�ͬ��ͨ��
	RNG rng((unsigned)time(NULL));
	Mat out;
	vector<Vec3b> colors;
	int number = connectedComponents(riceB, out, 8, CV_16U);	//ͳ��ͼ������ͨ��ĸ���
	for (int i = 0; i < number; i++)
	{
		//�������ȷ����ɫ
		Vec3b v3 = Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		colors.push_back(v3);
	}
	//�Բ�ͬ��ɫ�����ͨ��
	Mat result = Mat::zeros(rice.size(), image.type());
	int w = result.cols, h = result.rows;
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			int label = out.at<uint16_t>(i, j);
			if (label == 0)	//����Ϊ��ɫ����
				continue;
			result.at<Vec3b>(i, j) = colors[label];
		}
	}
	//��ʾ���
	imshow("ԭͼ", image);
	imshow("��Ǻ��ͼ��", result);
	waitKey(0);
	//ͳ����ͨ�����Ϣ
	Mat stats, centroids;
	number = connectedComponentsWithStats(riceB, out, stats, centroids, 8, CV_16U);
	drawStatus(image, stats, centroids, number);
	imshow("��Ǻ��ͼ��", image);
	waitKey(0);
}
//ͼ�����仯
void test30()
{
	Mat matrix = (Mat_<uchar>(5, 5) << 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 0, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1);
	Mat dist_L1, dist_L2, dist_C;
	//�����������
	distanceTransform(matrix, dist_L1, DIST_L1, 3, CV_8U);
	cout << "�������룺\n" << dist_L1 << endl;
	//����ŷ�Ͼ���
	distanceTransform(matrix, dist_L2, DIST_L2, 5, CV_32F);
	cout << "ŷ�Ͼ��룺\n" << dist_L2 << endl;
	//�������̾���
	distanceTransform(matrix, dist_C, DIST_C, 3, CV_8U);
	cout << "���̾��룺\n" << dist_C << endl;
	//��ͼ����о���任
	Mat rice = imread("../rice.jpg", IMREAD_GRAYSCALE);
	Mat riceB, riceBI;
	threshold(rice, riceB, 50, 255, THRESH_BINARY);
	threshold(rice, riceBI, 50, 255, THRESH_BINARY_INV);
	//����ͼ��Ľ�������
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
//ͼ��ʴ����
void test31()
{
	Mat matrix = (Mat_<uchar>(6, 6) << 0, 0, 0, 0, 255, 0,
		0, 255, 255, 255, 255, 255,
		0, 255, 255, 255, 255, 0,
		0, 255, 255, 255, 255, 0,
		0, 255, 255, 255, 255, 0,
		0, 0, 0, 0, 0, 0);
	Mat struct1, struct2;
	struct1 = getStructuringElement(0, Size(3, 3));	//���νṹԪ��
	struct2 = getStructuringElement(1, Size(3, 3));	//ʮ�ֽṹԪ��
	//��Ÿ�ʴ��Ľ��
	Mat eroMatrix;
	erode(matrix, eroMatrix, struct2);
	namedWindow("matrix", WINDOW_GUI_NORMAL);
	namedWindow("eroMatrix", WINDOW_GUI_NORMAL);
	imshow("matrix", matrix);
	imshow("eroMatrix", eroMatrix);
	waitKey(0);
	//���ָ�ʴ
	Mat learn = imread("../word.png", IMREAD_ANYCOLOR);
	Mat ero1, ero2;
	erode(learn, ero1, struct1);
	erode(learn, ero2, struct2);
	imshow("learn", learn);
	imshow("ero1", ero1);
	imshow("ero2", ero2);
	waitKey(0);
	//ͼ��ʴ
	Mat rice = imread("../rice.jpg");
	Mat gray;
	//��ԭͼƬת��Ϊ��ɫͼƬ
	cvtColor(rice, gray, COLOR_BGR2GRAY);
	//���ݴ�Ŷ�ֵ��ͼƬ������һ�ݺ��ڻ�ͼ��ͼƬ�Լ�����һ�ݴ�Ÿ�ʴͼ��
	Mat riceB, riceBW, rice_copy, eroRice;
	rice.copyTo(rice_copy);
	//��ʴԭͼ��
	erode(gray, eroRice, struct2);
	//��ԭͼ���ֵ��
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
//ͼ�����Ͳ���
void test32()
{
	Mat matrix = (Mat_<uchar>(6, 6) << 0, 0, 0, 0, 255, 0,
		0, 255, 255, 255, 255, 255,
		0, 255, 255, 255, 255, 0,
		0, 255, 255, 255, 255, 0,
		0, 255, 255, 255, 255, 0,
		0, 0, 0, 0, 0, 0);
	Mat struct1, struct2;
	struct1 = getStructuringElement(0, Size(3, 3));	//���νṹԪ��
	struct2 = getStructuringElement(1, Size(3, 3));	//ʮ�ֽṹԪ��
	//������ͺ�Ľ��
	Mat eroMatrix;
	dilate(matrix, eroMatrix, struct2);
	namedWindow("matrix", WINDOW_GUI_NORMAL);
	namedWindow("eroMatrix", WINDOW_GUI_NORMAL);
	imshow("matrix", matrix);
	imshow("eroMatrix", eroMatrix);
	waitKey(0);
	//��������
	Mat learn = imread("../word.png", IMREAD_ANYCOLOR);
	Mat ero1, ero2;
	dilate(learn, ero1, struct1);
	dilate(learn, ero2, struct2);
	imshow("learn", learn);
	imshow("ero1", ero1);
	imshow("ero2", ero2);
	waitKey(0);
}
//��ͼ�������̬ѧ����
void test33()
{
	//������֤��̬ѧӦ�õĶ�ֵ������
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
	//3 * 3�ľ��νṹԪ��
	Mat kernal = getStructuringElement(0, Size(3, 3));
	Mat kernelHit = (Mat_<uchar>(3, 3) << 0, 1, 0, 1, -1, 1, 0, 1, 0);
	//�Ծ��������̬ѧ����
	Mat open, close, gradient, tophat, blackhat, hitmiss;
	//������
	morphologyEx(matrix, open, MORPH_OPEN, kernal);
	namedWindow("open", WINDOW_GUI_NORMAL);
	imshow("open", open);
	//������
	morphologyEx(matrix, close, MORPH_CLOSE, kernal);
	namedWindow("close", WINDOW_GUI_NORMAL);
	imshow("close", close);
	//�ݶ�����
	morphologyEx(matrix, gradient, MORPH_GRADIENT, kernal);
	namedWindow("gradient", WINDOW_GUI_NORMAL);
	imshow("gradient", gradient);
	//��ñ����
	morphologyEx(matrix, tophat, MORPH_TOPHAT, kernal);
	namedWindow("tophat", WINDOW_GUI_NORMAL);
	imshow("tophat", tophat);
	//��ñ����
	morphologyEx(matrix, blackhat, MORPH_BLACKHAT, kernal);
	namedWindow("blackhat", WINDOW_GUI_NORMAL);
	imshow("blackhat", blackhat);
	//���л���������
	morphologyEx(matrix, hitmiss, MORPH_HITMISS, kernelHit);
	namedWindow("hitmiss", WINDOW_GUI_NORMAL);
	imshow("hitmiss", hitmiss);
	waitKey(0);
	//��ͼƬ������̬ѧ����
	Mat key = imread("../key.jpg", IMREAD_GRAYSCALE);
	//���ԭͼ��
	imshow("key", key);
	Mat keyB;
	threshold(key, keyB, 100, 255, THRESH_BINARY);
	//�����ֵ�����ͼ��
	imshow("keyB", keyB);
	//5 * 5����ṹԪ��
	Mat kernal_ = getStructuringElement(0, Size(5, 5));
	Mat open_key, close_key, gradient_key, tophat_key, blackhat_key, hitmiss_key;
	//��ͼ���������
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
//ͼ��ϸ��
void test34()
{
	Mat word = imread("../word.png", IMREAD_GRAYSCALE);
	if (word.empty())
		exit(-1);
	//����Ӣ�ĺ�ʵ��Բ��Բ����ͼƬ
	Mat words = Mat::zeros(100, 200, CV_8UC1);
	putText(words, "Learn", Point(30, 30), 2, 1, Scalar(255), 2);
	putText(words, "OpenCV", Point(30, 60), 2, 1, Scalar(255), 2);
	circle(words, Point(80, 75), 10, Scalar(255), -1);
	circle(words, Point(130, 75), 10, Scalar(255),3);
	//����ϸ��
	Mat thin1, thin2;
	ximgproc::thinning(word, thin1, 0);
	ximgproc::thinning(words, thin2, 0);
	//��ʾ���
	namedWindow("words", WINDOW_NORMAL);
	namedWindow("thin2", WINDOW_NORMAL);
	imshow("word", word);
	imshow("thin1", thin1);
	imshow("words", words);
	imshow("thin2", thin2);
	waitKey(0);
}
//�������
void test35()
{
	Mat image = imread("../key.jpg");
	if (image.empty())
		exit(-1);
	imshow("ԭͼ", image);
	Mat gray, bin;
	cvtColor(image, gray, COLOR_BGR2GRAY);	//ת��Ϊ�Ҷ�ͼ��
	GaussianBlur(gray, gray, Size(5, 5), 4, 4);	//ƽ���˲�
	threshold(gray, bin, 170, 255, THRESH_BINARY | THRESH_OTSU);	//����Ӧ��ֵ��

	//������������
	vector<vector<Point>> contours;	//����
	vector<Vec4i> hierarchy;	//��������ṹ����

	findContours(bin, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	//��������
	for (int i = 0; i < hierarchy.size(); i++)
	{
		cout << hierarchy[i] << endl;
 	}
	for (int t = 0; t < contours.size(); t++)
	{
		drawContours(image, contours, t, Scalar(0, 0, 255), 2, 8);
		imshow("���ƽ��", image);
		waitKey(0);
	}
}
//������Ϣͳ��
void test36()
{
	vector<Point> contour;
	contour.push_back(Point(0, 0));
	contour.push_back(Point(10, 0));
	contour.push_back(Point(10, 10));
	contour.push_back(Point(5, 5));
	//���������
	double area = contourArea(contour);
	cout << "��������� = " << area << endl;
	//�������ܳ�
	double length1 = arcLength(contour, false);
	double length2 = arcLength(contour, true);
	cout << "�������ܳ�length1 = " << length1 << endl;
	cout << "�������ܳ�length2 = " << length2 << endl;
	//ͼ���������
	Mat key = imread("../key.jpg", IMREAD_GRAYSCALE);
	GaussianBlur(key, key, Size(9, 9), 5, 5);
	threshold(key, key, 170, 255, THRESH_BINARY | THRESH_OTSU);
	//�������
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(key, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	//���㲢�������������������ܳ�
	for (int i = 0; i < hierarchy.size(); i++)
	{
		cout << hierarchy[i] << endl;
	}
	for (int i = 0; i < contours.size(); i++)
	{
		cout << "��" << i << "����������� = " << contourArea(contours[i]) << endl;
		cout << "��" << i << "���������ܳ� = " << arcLength(contours[i], true) << endl;
	}
}
//������Ӷ����
void drawapp(Mat& input, Mat& image);

void test37()
{
	Mat image = imread("../con.jpg");
	if (image.empty())
		exit(-1);
	imshow("image", image);
	//���ͼ�����ڻ��ƽ��
	Mat image1, image2;
	image.copyTo(image1);	//���������Ӿ���
	image.copyTo(image2);	//������С��Ӿ���
	//ȥ�����Ͷ�ֵ��
	Mat canny1;
	Canny(image, canny1, 200, 240);
	//�������㣬��ϸС��϶�
	Mat kernel = getStructuringElement(0, Size(9, 9));
	dilate(canny1, canny1, kernel);
	//������������
	vector<vector<Point>> contours1;
	vector<Vec4i> hierarchy1;
	findContours(canny1, contours1, hierarchy1, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	//Ѱ��������Ӿ���
	for (int i = 0; i < contours1.size(); i++)
	{
		//���������Ӿ���
		Rect rect = boundingRect(contours1[i]);
		rectangle(image1, rect, Scalar(0, 255, 0), 2, 8, 0);
		//������С��Ӿ���
		RotatedRect rrect = minAreaRect(contours1[i]);
		Point2f points[4];
		rrect.points(points);	//�����С���ε��ĸ���
		Point2f center = rrect.center;	//�����С��Ӿ��ε�����
		//���ƾ���
		for (int p = 0; p < 4; p++)
		{
			line(image2, points[p], points[(p + 1) % 4], Scalar(0, 255, 0), 2, 8, 0);	//�����ĸ���
		}
		////���ƾ�������
		circle(image2, center, 4, Scalar(0, 255, 0), -1, 8, 0);
	}
	//������
	namedWindow("max", WINDOW_GUI_NORMAL);
	namedWindow("min", WINDOW_GUI_NORMAL);
	imshow("max", image1);
	imshow("min", image2);
	waitKey(0);
	//��������
	Mat approx = imread("../angle.png");
	imshow("ԭͼ", approx);
	Mat canny2;
	//ȥ�����Ͷ�ֵ��
	Canny(approx, canny2, 100, 160, 3, false);
	//��������
	Mat kernel_ = getStructuringElement(0, Size(3, 3));
	dilate(canny2, canny2, kernel_);
	//�������
	vector<vector<Point>> contours2;
	vector<Vec4i> hierarchy2;
	findContours(canny2, contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	//���ƶ����
	for (int i = 0; i < contours2.size(); i++)
	{
		//������С������ü�������
		RotatedRect rrect = minAreaRect(contours2[i]);
		Point2f center = rrect.center;
		circle(approx, center, 2, Scalar(0, 0, 255), 2, 8, 0);
		//���ƶ����
		Mat result;
		approxPolyDP(contours2[i], result, 4, true);
		drawapp(result, approx);
		//�ж���״
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
	//��ʾ���
	imshow("approx", approx);
	waitKey(0);
}
//͹�����
void test38()
{
	Mat image = imread("../hand.jpg", IMREAD_ANYCOLOR);
	if (image.empty())
		exit(-1);
	//ͼ���ֵ��
	Mat gray, bin;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	threshold(gray, bin, 150, 255, THRESH_BINARY_INV);
	//ͼ������ȥ������
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	morphologyEx(bin, bin, MORPH_OPEN, kernel);
	namedWindow("bin", WINDOW_NORMAL);
	imshow("bin", bin);
	//��ȡ����
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(bin, contours, hierarchy, 0, 2, Point());
	for (int i = 0; i < contours.size(); i++)
	{
		//����͹������
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
//ֱ�߼��
void drawLines(Mat& image, vector<Vec2f> lines, Scalar scalar, int width);

void test39()
{
	Mat gray = imread("../houghline.jpg", IMREAD_GRAYSCALE);
	if (gray.empty())
		exit(-1);
	//���ͼ���Ե�Լ���ֵ��
	Mat edge1;
	Canny(gray, edge1, 100, 200, 3, false);
	imshow("edge1", edge1);
	//�ò�ͬ�ۼ�������ֱ�߼��
	vector<Vec2f> lines1, lines2;
	HoughLines(edge1, lines1, 1, CV_PI / 180.0, 50, 0);
	HoughLines(edge1, lines2, 1, CV_PI / 180.0, 100, 0);
	//��ԭͼ���л���ֱ��
	Mat gray1, gray2;
	gray.copyTo(gray1);
	gray.copyTo(gray2);
	drawLines(gray1, lines1, Scalar(255), 2);
	drawLines(gray2, lines2, Scalar(255), 2);
	//��ʾͼ��
	imshow("gray", gray);
	imshow("gray1", gray1);
	imshow("gray2", gray2);
	waitKey(0);
	//���ý�������ʽ����任����ֱ��
	Mat edge2;
	Canny(gray, edge2, 80, 200, 3, false);
	imshow("edge2", edge2);
	//��������ʽ����任��ȡ�߶εĵ�
	vector<Vec4i> linesP1, linesP2;
	HoughLinesP(edge2, linesP1, 1, CV_PI / 180.0, 150, 30, 10);	//������������Ӿ���Ϊ10
	HoughLinesP(edge2, linesP2, 1, CV_PI / 180.0, 150, 30, 30);	//������������Ӿ���Ϊ30
	//����������Ϊ10��ֱ��
	Mat gray3;
	gray.copyTo(gray3);
	for (int i = 0; i < linesP1.size(); i++)
	{
		line(gray3, Point(linesP1[i][0], linesP1[i][1]), Point(linesP1[i][2], linesP1[i][3]), Scalar(255), 4);
	}
	//����������Ϊ30��ֱ��
	Mat gray4;
	gray.copyTo(gray4);
	for (int i = 0; i < linesP2.size(); i++)
	{
		line(gray4, Point(linesP2[i][0], linesP2[i][1]), Point(linesP2[i][2], linesP2[i][3]), Scalar(255), 4);
	}
	//��ʾ���
	imshow("gray3", gray3);
	imshow("gray4", gray4);
	waitKey(0);
}
//ֱ�����
void test40()
{
	system("color F0");	//�ı����������ɫ
	Vec4i lines;	//�����ϵ�ֱ��
	vector<Point2f> points;	//��Ŵ����ֱ�ߵ����е�
	const static float Points[21][2] = { { 0.0f, 0.0f }, { 10.0f, 11.0f }, { 21.0f, 20.0f },
		{ 40.0f, 42.0f }, { 50.0f, 50.0f }, { 60.0f, 60.0f }, { 70.0f, 70.0f}, { 80.0f, 80.0f },
		{ 90.0f, 92.0f }, { 100.0f, 100.0f }, { 110.0f, 110.0f }, { 120.0f, 120.0f }, { 136.0f, 130.0f },
		{ 175.0f, 170.0f }, { 181.0f, 180.0f }, {200.0f, 190.0f }, { 23.5f , 23.5f } };
	for (int i = 0; i < 21; i++)	//�����е�Ž�vector��
	{
		points.push_back(Point2f(Points[i][0], Points[i][1]));
	}
	//��������
	double param = 0;	//����ģ���е���ֵ����C
	double reps = 0.01;	//����ԭ����ֱ��֮��ľ��뾫��
	double aeps = 0.01;	//�ǶȾ���
	fitLine(points, lines, DIST_L1, param, reps, aeps);	//���ֱ��
	double k = lines[1] / lines[0];	//ֱ��б��
	cout << "ֱ��б�� k = " << k << endl;
	cout << "ֱ����һ������ x = " << lines[2] << " y = " << lines[3] << endl;
	cout << "ֱ�߽���ʽ��y = " << k << " * (x - " << lines[2] << ") - " << lines[3] << endl;
	waitKey(0);
	//�����κ�Բ�����
	RNG rng = RNG((unsigned)time(NULL));	//���������
	while (true)
	{
		Mat img1 = Mat(500, 500, CV_8UC3, Scalar::all(0));	//����ͼ��������������
		Mat img2;
		img1.copyTo(img2);	//����Բ����ϵ�ͼ��
		int countB = rng.uniform(10, 101);	//��ɫ��������
		int countG = rng.uniform(10, 101);	//��ɫ��������
		int countR = rng.uniform(10, 101);	//��ɫ��������
		vector<Point> ptsB, ptsG, ptsR;	//��Ų�ͬ��ɫ����������
		Point areaB;	//�����ɫ�����ɵ�����
		areaB.x = rng.uniform(0, 4);
		areaB.y = rng.uniform(0, 4);
		for (int i = 0; i < countB; i++)	//������ɫ�ĵ�
		{
			int x = rng.uniform(img1.cols * areaB.x / 4, img1.cols * (areaB.x + 1) / 4);	//�����x����
			int y = rng.uniform(img1.cols * areaB.y / 4, img1.cols * (areaB.y + 1) / 4);	//�����y����
			ptsB.push_back(Point(x, y));
		}
		Point areaG;	//�����ɫ�����ɵ�����
		areaG.x = rng.uniform(0, 4);
		areaG.y = rng.uniform(0, 4);
		for (int i = 0; i < countG; i++)	//������ɫ�ĵ�
		{
			int x = rng.uniform(img1.cols * areaG.x / 4, img1.cols * (areaG.x + 1) / 4);	//�����x����
			int y = rng.uniform(img1.cols * areaG.y / 4, img1.cols * (areaG.y + 1) / 4);	//�����y����
			ptsG.push_back(Point(x, y));
		}
		Point areaR;	//��Ǻ�ɫ�����ɵ�����
		areaR.x = rng.uniform(0, 4);
		areaR.y = rng.uniform(0, 4);
		for (int i = 0; i < countR; i++)	//���ɺ�ɫ�ĵ�
		{
			int x = rng.uniform(img1.cols * areaR.x / 4, img1.cols * (areaR.x + 1) / 4);	//�����x����
			int y = rng.uniform(img1.cols * areaR.y / 4, img1.cols * (areaR.y + 1) / 4);	//�����y����
			ptsR.push_back(Point(x, y));
		}
		//��ͼƬ�л��������
		for (int i = 0; i < countB; i++)	//��ɫ
		{
			circle(img1, ptsB[i], 3, Scalar(255, 0, 0), FILLED, 8, 0);
			circle(img2, ptsB[i], 3, Scalar(255, 0, 0), FILLED, 8, 0);
		}
		for (int i = 0; i < countG; i++)	//��ɫ
		{
			circle(img1, ptsG[i], 3, Scalar(0, 255, 0), FILLED, 8, 0);
			circle(img2, ptsG[i], 3, Scalar(0, 255, 0), FILLED, 8, 0);
		}
		for (int i = 0; i < countR; i++)	//��ɫ
		{
			circle(img1, ptsR[i], 3, Scalar(0, 0, 255), FILLED, 8, 0);
			circle(img2, ptsR[i], 3, Scalar(0, 0, 255), FILLED, 8, 0);
		}
		//Ѱ�Ұ�Χ�㼯��������
		vector<Point2f> triangleB, triangleG, triangleR;
		minEnclosingTriangle(ptsB, triangleB);
		minEnclosingTriangle(ptsG, triangleG);
		minEnclosingTriangle(ptsR, triangleR);
		//�������������
		for (int i = 0; i < 3; i++)
		{
			line(img1, triangleB[i], triangleB[(size_t)(i + 1) % 3], Scalar(255, 0, 0), 2, LINE_AA, 0);
			line(img1, triangleG[i], triangleG[(size_t)(i + 1) % 3], Scalar(0, 255, 0), 2, LINE_AA, 0);
			line(img1, triangleR[i], triangleR[(size_t)(i + 1) % 3], Scalar(0, 0, 255), 2, LINE_AA, 0);
		}
		//Ѱ�Ұ�Χ�㼯��Բ��
		Point2f centerB, centerG, centerR;
		float radiusB = 0, radiusG = 0, radiusR = 0;
		minEnclosingCircle(ptsB, centerB, radiusB);
		minEnclosingCircle(ptsG, centerG, radiusG);
		minEnclosingCircle(ptsR, centerR, radiusR);
		//�������Բ��
		circle(img2, centerB, radiusB, Scalar(255, 0, 0), 2, LINE_AA, 0);
		circle(img2, centerG, radiusG, Scalar(0, 255, 0), 2, LINE_AA, 0);
		circle(img2, centerR, radiusR, Scalar(0, 0, 255), 2, LINE_AA, 0);
		//��ʾ���ƽ��
		imshow("TRIANGLE", img1);
		imshow("CIRCLE", img2);
		char input = waitKey();
		if (input == 27 || input == 'q' || input == 'Q')
		{
			break;
		}
	}
}
//��ά��ʶ��
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
	bool isQRCode = QRCodedetector.detect(QRCode_gray, points1);	//ʶ���ά��
	if (isQRCode)
	{
		//�����ά��
		information1 = QRCodedetector.decode(QRCode_gray, points1, QRCode_bin);
		cout << points1 << endl;	//����ĸ����������
	}
	else
	{
		cout << "�޷�ʶ���ά�룬��ȷ��ͼ���Ƿ��ж�ά�룡" << endl;
		exit(-1);
	}
	//���ƶ�ά��߿�
	for (int i = 0; i < 4; i++)
	{
		line(QRCode, points1[i], points1[(i + 1) % 4], Scalar(0, 255, 255), 3, 8);
	}
	//�����������Ϣ���Ƶ���ά����
	putText(QRCode, information1, Point(20, 20), 0, 1, Scalar(0, 0, 255), 2);
	//ֱ�Ӷ�λ��ά�벢����
	string information2;
	vector<Point> points2;
	information2 = QRCodedetector.detectAndDecode(QRCode_gray, points2);
	putText(QRCode, information2, Point(20, 40), 0, 1, Scalar::all(0), 2);
	cout << points2 << endl;
	//������
	namedWindow("QRCode", WINDOW_GUI_NORMAL);
	namedWindow("QRCode_bin", WINDOW_GUI_NORMAL);
	imshow("QRCode", QRCode);
	imshow("QRCode_bin", QRCode_bin);
	waitKey(0);
}
//ͼ��Ļ������
void test42()
{
	//����һ��16*16ȫΪ1�ľ���
	Mat matrix = Mat::ones(Size(16, 16), CV_32FC1);
	//��ͼ���������������
	RNG rng = RNG((unsigned)time(NULL));
	for (int i = 0; i < matrix.rows; i++)
	{
		for (int j = 0; j < matrix.cols; j++)
		{
			float noise = rng.uniform(-0.5, 0.5);
			matrix.at<float>(i, j) = matrix.at<float>(i, j) + noise;
		}
	}
	//�����׼��ͻ���
	Mat sum;
	integral(matrix, sum);
	//����չʾת��ΪCV_8U��ʽ
	Mat sum8U = Mat_<uchar>(sum);
	namedWindow("sum8U", WINDOW_GUI_NORMAL);
	imshow("sum8U", sum8U);
	//����ƽ����ͻ���
	Mat sqsum;
	integral(matrix, sum, sqsum);
	//����չʾת��ΪCV_8U��ʽ
	Mat sqsum8U = Mat_<uchar>(sqsum);
	namedWindow("sqsum8U", WINDOW_GUI_NORMAL);
	imshow("sqsum8U", sqsum8U);
	//������б��ͻ���
	Mat tilted;
	integral(matrix, sum, sqsum, tilted);
	//����չʾת��ΪCV_8U��ʽ
	Mat tilted8U = Mat_<uchar>(tilted);
	namedWindow("tilted8U", WINDOW_GUI_NORMAL);
	imshow("tilted8U", tilted8U);
	waitKey(0);
}
//��ˮ��䷨
void test43()
{
	Mat image = imread("../lena.jpg", IMREAD_ANYCOLOR);
	if (image.empty())
		exit(-1);
	
	RNG rng = RNG((unsigned)time(NULL));	//�����������
	//���ò�����־flag
	int connectivity = 4;	//��ͨ����ʽ
	int maskVal = 255;	//����ͼ����ֵ
	int flags = connectivity | (maskVal << 8) | FLOODFILL_FIXED_RANGE;	//��ˮ��䷽ʽ��־
	//����ѡ�����صĲ�ֵ
	Scalar loDiff = Scalar(20, 20, 20);
	Scalar upDiff = Scalar(20, 20, 20);
	//��������������(��ԭͼ���2)
	Mat mask = Mat::zeros(image.rows + 2, image.cols + 2, CV_8UC1);
	while (true)
	{
		//�������ͼ���е�һ��
		int x = rng.uniform(0, image.cols);
		int y = rng.uniform(0, image.rows);
		Point point = Point(x, y);
		//���������������ֵ
		Scalar newVal = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		//��ˮ��亯��
		int area = floodFill(image, mask, point, newVal, nullptr, loDiff, upDiff, flags);
		//������ص�����ص���Ŀ
		cout << "���ӵ㣺" << point << "\t���������Ŀ��" << area << endl;
		//������
		imshow("���Ĳ�ɫͼ��", image);
		imshow("����ͼ��", mask);
		//�жϳ����Ƿ����
		char input = waitKey();
		if (input == 27)
		{
			break;
		}
	}
}
//��ˮ�뷨
void test44()
{
	Mat image, image_, gray, mask;
	Mat maskWaterShed;	//watershed()�����Ĳ���
	image = imread("../lena.jpg");	//��ȡԭͼ��
	image_ = imread("../lena_.png");	//��ȡ���б�ǵ�ͼ��
	cvtColor(image_, gray, COLOR_BGR2GRAY);
	//��ֵ�������п�����
	threshold(gray, mask, 254, 255, THRESH_BINARY);
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(mask, mask, MORPH_OPEN, kernel);
	imshow("ԭͼ��", image);
	imshow("���б�ǵ�ͼ��", image_);
	//��maskWaterShed�ϻ������������������ˮ���㷨
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(mask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	maskWaterShed = Mat::zeros(mask.size(), CV_32S);
	for (int idx = 0; idx < contours.size(); idx++)
	{
		drawContours(maskWaterShed, contours, idx, Scalar::all(idx + 1), -1, 8, hierarchy, INT_MAX);
	}
	watershed(image, maskWaterShed);	//��ˮ���㷨��ԭͼ����
	//������ɼ�����ɫ
	vector<Vec3b> colors;
	for (int i = 0; i < contours.size(); i++)
	{
		int b = theRNG().uniform(0, 256);
		int g = theRNG().uniform(0, 256);
		int r = theRNG().uniform(0, 256);
		colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
	}
	Mat result = Mat(image.size(), CV_8UC3);	//��ʾͼ��
	for (int i = 0; i < mask.rows; i++)
	{
		for (int j = 0; j < mask.cols; j++)
		{
			//����ÿ���������ɫ
			int idx = maskWaterShed.at<int>(i, j);
			if (idx == -1)	//�����ֵΪ-1(���߽�)
			{
				result.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
			}
			else if (idx <= 0 || idx > contours.size())	//û�б�����������Ϊ0
			{
				result.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
			else
			{
				result.at<Vec3b>(i, j) = colors[(size_t)idx - 1];	//����Щ������Ƴɲ�ͬ��ɫ
			}
		}
	}
	//��ʾ���
	imshow("result", result);
	result = result * 0.8 + image * 0.2;
	//addWeighted(result, 0.8, image, 0.2, 0, result, -1);
	imshow("��ˮ����", result);
	//����ÿ������ͼ��
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
		//��ʾͼ��
		imshow(to_string(n), result_);
	}
	waitKey(0);
}
//Harris�ǵ�
void test45()
{
	Mat lena = imread("../lena.jpg", IMREAD_COLOR);
	Mat gray;
	cvtColor(lena, gray, COLOR_BGR2GRAY);
	//����Harrisϵ��
	Mat harris;
	int blockSize = 2;	//����뾶
	int aperturSize = 3;	//Soble���Ӵ�С
	cornerHarris(gray, harris, blockSize, aperturSize, 0.04);
	//��һ��������ֵ�ȽϺͽ����ʾ
	Mat harrisn;
	normalize(harris, harrisn, 0, 255, NORM_MINMAX);
	//��ͼ���������ͱ�ΪCV_8U
	convertScaleAbs(harrisn, harrisn);
	//Ѱ��Harris�ǵ�
	vector<KeyPoint> keyPoints;
	for (int i = 0; i < harrisn.rows; i++)
	{
		for (int j = 0; j < harrisn.cols; j++)
		{
			int R = harrisn.at<uchar>(i, j);
			if (R > 125)
			{
				//���ǵ����KeyPoint��
				KeyPoint keyPoint;
				keyPoint.pt.x = j;
				keyPoint.pt.y = i;
				keyPoints.push_back(keyPoint);
			}
		}
	}
	//���ƽǵ�
	drawKeypoints(lena, keyPoints, lena);
	imshow("ϵ������", harrisn);
	imshow("Harris�ǵ�", lena);
	waitKey(0);
}
//Shi-Tomas�ǵ���
void test46()
{
	Mat image = imread("../lena.jpg");
	if (image.empty())
		exit(-1);
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	//��ȡ�ǵ�
	int maxCorners = 100;	//���ǵ���Ŀ
	double qualityLevel = 0.01;	//�����ȼ�������ֵ����ѽǵ�֮��Ĺ�ϵ
	double minDistance = 0.04;	//�����ǵ�֮�����Сŷʽ����
	vector<Point2f> corners;	//��Žǵ������
	goodFeaturesToTrack(gray, corners, maxCorners, qualityLevel, minDistance, Mat(), 3, false);
	//���ƽǵ�
	vector<KeyPoint> keyPoints;	//���ǵ��ŵ�KeyPoints�У�������ڻ���
	for (int i = 0; i < corners.size(); i++)
	{
		KeyPoint keyPoint;
		keyPoint.pt = corners[i];
		keyPoints.push_back(keyPoint);
	}
	drawKeypoints(image, keyPoints, image);
	imshow("�ǵ���", image);
	waitKey(0);
}
//�����ؼ���ǵ��Ż�
void test47()
{
	system("color F0");
	Mat image = imread("../lena.jpg");
	if (image.empty())
		exit(-1);
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	//��ȡ�ǵ�
	int maxCorner = 100;	//���ǵ���Ŀ
	double qualityLevel = 0.01;	//�����ȼ�
	double mindistance = 0.04;	//��Сŷʽ����
	vector<Point2f> corners;	//��Žǵ�����
	goodFeaturesToTrack(gray, corners, maxCorner, qualityLevel, mindistance, Mat(), 3, false);
	//���������ؼ��ǵ�����
	vector<Point2f> cornersSub = corners;	//���ǵ㱸�ݷ�ֹ�����޸�
	Size winSize = Size(5, 5);	//���ڴ�С��ʵ��Ϊ�Ĵ�С��2��+1
	Size zeroZone = Size(-1, -1);	//������С
	TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
	cornerSubPix(gray, cornersSub, winSize, zeroZone, criteria);
	//�����ʼ������;�ϸ������
	for (int i = 0; i < corners.size(); i++)
	{
		string str = to_string(i);
		str = "��" + str + "���ǵ��ʼ���꣺";
		cout << str << corners[i] << "��ϸ��������꣺" << cornersSub[i] << endl;
	}
}
//ORB������
void orb_features(Mat& image, vector<KeyPoint>& keyPoints, Mat& descriptions);	//����ORB������

void test48()
{
	Mat image = imread("../lena.jpg");
	if (image.empty())
		exit(-1);
	vector<KeyPoint> keyPoints;
	Mat descriptions;
	//���� ORB ������
	orb_features(image, keyPoints, descriptions);
	 //����������
	Mat imageAngle;
	image.copyTo(imageAngle);
	//���Ʋ����Ƕȴ�С�Ľ��
	drawKeypoints(image, keyPoints, image, Scalar(255, 255, 255));
	//���ƺ��нǶȺʹ�С�Ľ��
	drawKeypoints(imageAngle, keyPoints, imageAngle, Scalar(255, 255, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//���ƽ��
	imshow("image", image);
	imshow("imageAngle", imageAngle);
	waitKey(0);
}
//������ƥ��
void match_min(vector<DMatch>& matches, vector<DMatch>& good_matches);	//ɸѡ��������

void test49()
{
	Mat gray = imread("../book.jpg");
	Mat gray_ = imread("../book_.jpg");
	if (gray.empty() || gray_.empty())
		exit(-1);
	//����ORB������
	vector<KeyPoint> keyPoints1, keyPoints2;
	Mat description1, description2;
	orb_features(gray, keyPoints1, description1);
	orb_features(gray_, keyPoints2, description2);
	//������ƥ��
	vector<DMatch> matches;	//����ƥ���Ž���ı���
	BFMatcher matcher(NORM_HAMMING);	//����������ƥ����࣬ʹ�ú�������
	matcher.match(description1, description2, matches);	//������ƥ��
	cout << "matches = " << matches.size() << endl;	//ƥ��ɹ�����������Ŀ
	//ͨ����������ɸѡƥ����
	vector<DMatch> good_matches;
	match_min(matches, good_matches);
	//���ƽ��
	Mat out1, out2;
	drawMatches(gray, keyPoints1, gray_, keyPoints2, matches, out1);
	drawMatches(gray, keyPoints1, gray_, keyPoints2, good_matches, out2);
	//������
	imshow("δɸѡ���", out1);
	waitKey(0);
	imshow("ɸѡ���", out2);
	waitKey(0);
}
//RANSCA �Ż�������ƥ��
void ransac(vector<DMatch>& matches, vector<KeyPoint>& queryKeyPoint, vector<KeyPoint>& trainKeyPoint, vector<DMatch>& matches_ransac);	//ransac�㷨ʵ��

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
	//������ƥ��
	vector<DMatch> matches;	//����ƥ���Ž���ı���
	BFMatcher matcher(NORM_HAMMING);	//����������ƥ����࣬ʹ�ú�������
	matcher.match(description1, description2, matches);	//������ƥ��
	cout << "matches = " << matches.size() << endl;	//ƥ��ɹ�����������Ŀ
	//ͨ����������ɸѡƥ����
	vector<DMatch> good_matches;
	match_min(matches, good_matches);
	//ransac�㷨ɸѡ����
	vector<DMatch> good_ransac;
	ransac(good_matches, keyPoints1, keyPoints2, good_ransac);
	//���ƽ��
	Mat out1, out2, out3;
	drawMatches(image1, keyPoints1, image2, keyPoints2, matches, out1);
	drawMatches(image1, keyPoints1, image2, keyPoints2, good_matches, out2);
	drawMatches(image1, keyPoints1, image2, keyPoints2, good_ransac, out3);
	imshow("δɸѡ���", out1);
	waitKey(0);
	imshow("��С��������ɸѡ", out2);
	waitKey(0);
	imshow("ransacɸѡ", out3);
	waitKey(0);
}
//���ģ����ͶӰ
void test51()
{
	//�������ĵõ����ڲξ���ͻ������
	Mat cameraMatrix = (Mat_<float>(3, 3) << 532.016297, 0, 332.172519,
		0, 531.565159, 233.388075,
		0, 0, 1);
	Mat distCoeffs = (Mat_<float>(1, 5) << -0.285188, 0.080097, 0.001274,
		-0.002415, 0.106579);
	//�����嵥10-10�м���ĵ�һ��ͼƬ�������ϵ����������ϵ֮��Ĺ�ϵ
	Mat rvec = (Mat_<float>(1, 3) << -1.977853, -2.002220, 0.130029);
	Mat tvec = (Mat_<float>(1, 3) << -26.8815, -42.79936, 159.19703);
	//���ɵ�һ��ͼƬ���ڽǵ����ά��������
	Size boardSize = Size(9, 6);
	Size squareSize = Size(10, 10);	//���̸�ÿ���������ʵ�ߴ�
	vector<Point3f> PointSets;
	for (int j = 0; j < boardSize.height; j++)
	{
		for (int k = 0; k < boardSize.width; k++)
		{
			Point3f realPoint;
			//����궨��Ϊ��������ϵ��zƽ�棬��z=0
			realPoint.x = j * squareSize.width;
			realPoint.y = k * squareSize.height;
			realPoint.z = 0;
			PointSets.push_back(realPoint);
		}
	}
	//������ά������������������ϵʱ��Ĺ�ϵ�����ڽǵ���������
	vector<Point2f> imagePoints;
	projectPoints(PointSets, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
	for (int i = 0; i < imagePoints.size(); i++)
	{
		cout << "��" << to_string(i) << "���������" << imagePoints[i] << endl;
	}
	waitKey(0);
}
//��Ŀ����ı궨
void test52()
{
	//�궨����Ҫ�аױߣ���Ҫ����
	vector<Mat> images;	//��ȡ����ͼƬ
	string imagename;
	ifstream ifs("../test.txt");
	while (getline(ifs, imagename))
	{
		Mat image = imread(imagename);
		images.push_back(image);
	}
	Size board_size = Size(8, 5);	//����궨��ǵ���Ŀ(�У���)
	vector<vector<Point2f>> imagesPoints;
	for (int i = 0; i < images.size(); i++)
	{
		Mat image_ = images[i];
		Mat gray;
		cvtColor(image_, gray, COLOR_BGR2GRAY);
		vector<Point2f> image_Points;
		findChessboardCorners(gray, board_size, image_Points);	//���㷽��궨��ǵ�
		find4QuadCornerSubpix(gray, image_Points, Size(5, 5));	//ϸ������궨������
		bool pattern = true;
		drawChessboardCorners(image_, board_size, image_Points, pattern);
		imshow("image_", image_);
		waitKey(0);
		imagesPoints.push_back(image_Points);
	}
	//�������̸���ÿ���ڽǵ�Ŀռ���ά����
	Size squareSize = Size(1, 1);	//���̸����ʵ�ߴ�
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
				//����궨��Ϊ��������ϵ��oxy�棬��z=0
				realPoint.z = 0;
				tempPoints.push_back(realPoint);
			}
		}
		objectPoints.push_back(tempPoints);
	}
	//ͼ��ߴ�
	Size imageSize = images[0].size();
	
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));	//������ڲ�������
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));	//�������5������ϵ����k1,k2,p1,p2,k3
	vector<Mat> rvecs;	//ÿ��ͼ�����ת����
	vector<Mat> tvecs;	//ÿ��ͼ���ƽ����
	calibrateCamera(objectPoints, imagesPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);
	cout << "�ڲ�������" << endl << cameraMatrix << endl;
	cout << "����ϵ����" << distCoeffs << endl;
	waitKey(0);
}
//ͼ���У��
void undist(vector<Mat> images, Mat cameraMatrix, Mat distCoeffs, vector<Mat>& undistImages);	//ͼ��У������

void test53()
{
	vector<Mat> images;	//��ȡ����ͼƬ
	string imagename;
	ifstream ifs("../test.txt");
	while (getline(ifs, imagename))
	{
		Mat image = imread(imagename);
		images.push_back(image);
	}
	
	Mat cameraMatrix = (Mat_<float>(3, 3) << 315.0799467892673, 0, 362.4333928879762,
	0, 314.3207165955638, 235.8589898288922,
	0, 0, 1);	//�ڲ�������	
	Mat distCoeffs = (Mat_<float>(1, 5) << -0.05984332512739104, 0.5171941952024617, -0.008125548898121952, 0.005511163544186489, -0.9147395306811681);	//����ϵ��

	vector<Mat> undistImages;	//���У�����ͼƬ

	undist(images, cameraMatrix, distCoeffs, undistImages);	//����undist��������У������
	//��ʾУ�����ͼ��
	for (int i = 0; i < images.size(); i++)
	{

		string windowNumber = to_string(i);
		imshow("δУ����ͼ��" + windowNumber, images[i]);
		imshow("У�����ͼ��" + windowNumber, undistImages[i]);
		waitKey(0);
		destroyWindow("δУ����ͼ��" + windowNumber);
		destroyWindow("У�����ͼ��" + windowNumber);
	}
}
//��Ŀ���λ�˹���
void test54()
{
	Mat image = imread("../test2.jpg");
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	vector<Point2f> imgPoints;
	Size boardSize = Size(8, 5);
	findChessboardCorners(gray, boardSize, imgPoints);	//���㷽��궨��ǵ�
	find4QuadCornerSubpix(gray, imgPoints, boardSize);	//ϸ������궨��ǵ�����
	//�������̸�ÿ���ڽǵ�Ŀռ���ά�ռ�����
	Size squareSize = Size(1, 1);	//���̸�ÿ��������ʵ�ߴ�
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
		0, 0, 1);	//�ڲ�������	
	Mat distCoeffs = (Mat_<float>(1, 5) << -0.05984332512739104, 0.5171941952024617, -0.008125548898121952, 0.005511163544186489, -0.9147395306811681);	//����ϵ��
	//����PnP�㷨������ת��ƽ��
	Mat rvec, tvec;
	solvePnP(PointSets, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);
	cout << "��������ϵ�任���������ϵ����ת������" << rvec << endl;
	//��ת����ת����ת����
	Mat R;
	Rodrigues(rvec, R);
	cout << "��ת����ת����ת����" << endl << R << endl;
	//��PnP+RANSAC�㷨������ת������ƽ������
	Mat rvecRansac, tvecRansac;
	solvePnPRansac(PointSets, imgPoints, cameraMatrix, distCoeffs, rvecRansac, tvecRansac);
	Mat RRansac;
	Rodrigues(rvecRansac, RRansac);
	cout << "��ת����ת��Ϊ��ת����" << endl << RRansac << endl;
	waitKey(0);
}
//��ֵ����ƶ�����
void test55()
{
	VideoCapture capture("../car.avi");
	if (!capture.isOpened())
		exit(-1);
	//�����Ƶ�����Ϣ
	int fps = capture.get(CAP_PROP_FPS);
	int width = capture.get(CAP_PROP_FRAME_WIDTH);
	int height = capture.get(CAP_PROP_FRAME_HEIGHT);
	int num_of_frames = capture.get(CAP_PROP_FRAME_COUNT);
	cout << "��Ƶ֡�ʣ�" << fps << "\n��Ƶ��ȣ�" << width << "\n��Ƶ�߶ȣ�" << height << "\n��Ƶ��֡����" << num_of_frames << endl;
	//��ȡ��Ƶ��һ֡ͼ����Ϊǰһ֡ͼ��
	Mat preFrame, preGray;
	capture.read(preFrame);
	cvtColor(preFrame, preGray, COLOR_BGR2GRAY);
	//���и�˹�˲���������
	GaussianBlur(preGray, preGray, Size(0, 0), 15);
	//��ȡ�˶�����
	Mat binary;
	Mat frame, gray;
	while (true)
	{
		if (!capture.read(frame))	//��Ƶ��ȡ�������˳�ѭ��
		{
			break;
		}
		//�Ե�ǰ֡���лҶȻ��͸�˹�˲�
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		GaussianBlur(gray, gray, Size(0, 0), 15);
		//ǰ��֡ͼ�����ȡ����ֵ
		absdiff(preGray, gray, binary);
		//�����������ж�ֵ����������
		threshold(binary, binary, 100, 255, THRESH_BINARY | THRESH_OTSU);
		Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));
		morphologyEx(binary, binary, MORPH_OPEN, kernel);
		//��ʾ���
		imshow("input", frame);
		imshow("output", binary);
		//����ǰ֡��Ϊǰһ֡��׼����һѭ����ע�͵�������ʾʹ�ù̶�����
		frame.copyTo(preFrame);
		//5�����ӳ��ж��Ƿ��˳�����
		char ch = waitKey(5);
		if (ch == 27)
		{
			break;
		}
	}
}
//���ܹ����������ƶ�����
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
		imshow("ԭ��Ƶ", frame);
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		GaussianBlur(gray, gray, Size(3, 3), 10);
		//������ܹ���
		Mat_<Point2f> flow;	//����������˶��ٶ�
		calcOpticalFlowFarneback(preGray, gray, flow, 0.5, 3, 15, 5, 5, 1.2, 0);
		
		Mat xV = Mat::zeros(preFrame.size(), CV_32FC1);	//x�����ƶ��ٶ�
		Mat yV = Mat::zeros(preFrame.size(), CV_32FC1);	//y�����ƶ��ٶ�
		//��ȡ����������ƶ��ٶ�
		for (int i = 0; i < flow.rows; i++)
		{
			for (int j = 0; j < flow.cols; j++)
			{
				const Point2f& flow_xy = flow.at<Point2f>(i, j);
				xV.at<float>(i, j) = flow_xy.x;
				yV.at<float>(i, j) = flow_xy.y;
			}
		}
		//�����������Ⱥͷ�ֵ
		Mat magnitude, angle;
		cartToPolar(xV, yV, magnitude, angle);
		//������ת��Ϊ�Ƕ�
		angle = angle * 180 / CV_PI;
		//����ֵ��һ����0~255���䣬�Ա���ʾ
		normalize(magnitude, magnitude, 0, 255, NORM_MINMAX);
		//����ǶȺͷ�ֵ�ľ���ֵ
		convertScaleAbs(magnitude, magnitude);
		convertScaleAbs(angle, angle);
		//���˶��ķ�ֵ�ͽǶ�����HSV��ɫ�ռ�
		Mat HSV = Mat::zeros(preFrame.size(), preFrame.type());
		vector<Mat> result;
		split(HSV, result);
		result[0] = angle;	//������ɫ
		result[1] = Scalar(255);	
		result[2] = magnitude;	//������̬
		//��������ͨ��ͼ��ϲ��ɶ�ͨ��ͼ��
		merge(result, HSV);
		//��HSV��ɫ�ռ�ת����RGB��ɫ�ռ�
		Mat RGBImg;
		cvtColor(HSV, RGBImg, COLOR_HSV2BGR);
		//��ʾ���
		imshow("�˶������", RGBImg);
		char ch = waitKey(5);
		if (ch == 27)
		{
			break;
		}
	}
}
//ϡ������������ƶ�����
vector<Scalar> color_lut;	//��ɫ���ұ�
void draw_lines(Mat& image, vector<Point2f> pt1, vector<Point2f> pt2);

void test57()
{
	VideoCapture capture("../car.avi");
	Mat preFrame, preGray;
	if (!capture.isOpened())
		exit(-1);
	capture.read(preFrame);
	cvtColor(preFrame, preGray, COLOR_BGR2GRAY);
	//�ǵ�����ز�������
	vector<Point2f> points;
	double qualityLevel = 0.01;
	int minDistance = 10;
	int blockSize = 3;
	bool usrHarrisDetector = false;
	double k = 0.04;
	int Corners = 5000;
	//�ǵ���
	goodFeaturesToTrack(preGray, points, Corners, qualityLevel, minDistance, Mat(), blockSize, usrHarrisDetector, k);
	//ϡ����������ز�������
	vector<Point2f> prePoints;	//ǰһ֡ͼ��ǵ�����
	vector<Point2f> nextPoints;	//��ǰ֡ͼ��ǵ�����
	vector<uchar> status;	//�ǵ��⵽��״̬
	vector<float> err;	//���
	TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
	double derivlamda = 0.5;
	int flags = 0;
	//��ʼ״̬�Ľǵ�
	vector<Point2f> initPoints;
	initPoints.insert(initPoints.end(), points.begin(), points.end()); 
	//ǰһ֡ͼ���нǵ�����
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
		//��������
		calcOpticalFlowPyrLK(preGray, nextGray, prePoints, nextPoints, status, err, Size(21, 21), 3, criteria, flags);
		//�жϽǵ��Ƿ��ƶ������ƶ���ɾ��
		size_t i, k;
		for (i = k = 0; i < prePoints.size(); i++)
		{
			//������״̬����
			double dist = abs(prePoints[i].x - nextPoints[i].x) + abs(prePoints[i].y - nextPoints[i].y);
			if (status[i] && dist > 2)
			{
				prePoints[k] = prePoints[i];
				initPoints[k] = initPoints[i];
				nextPoints[k++] = nextPoints[i];
				circle(nextFrame, nextPoints[i], 3, Scalar(0, 255, 0), -1);
			}
		}
		//���½ǵ���Ŀ
		prePoints.resize(k);
		nextPoints.resize(k);
		initPoints.resize(k);
		//���Ƹ��ٹ켣
		draw_lines(nextFrame, initPoints, nextPoints);
		imshow("result", nextFrame);

		char ch = waitKey(50);
		if (ch == 27)
		{
			break;
		}
		//���½ǵ������ǰһ֡ͼ��
		swap(prePoints, nextPoints);
		nextGray.copyTo(preGray);
		//����ǵ���ĿС��30���������¼��ǵ�
		if (initPoints.size() < 30)
		{
			goodFeaturesToTrack(preGray, points, Corners, qualityLevel, minDistance, Mat(), blockSize, usrHarrisDetector, k);
			initPoints.insert(initPoints.end(), points.begin(), points.end());
			prePoints.insert(prePoints.end(), points.begin(), points.end());
			cout << "total feature points: " << prePoints.size() << endl;
		}
	}
}
//k��ֵ����
void test58()
{
	//����һ��500*500��ͼ��������ʾ������ͷ�����
	Mat image(500, 500, CV_8UC3, Scalar::all(255));
	RNG rng(10000);
	//����������ɫ
	Scalar colors[3] = { Scalar(0, 0, 255), Scalar(0, 255, 0), Scalar(255, 0, 0) };
	//���������㼯������ÿ���㼯�е����Ŀ���
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
	//ʹ��KMeans
	Mat labels;	//ÿ��������������
	Mat centers;	//ÿ���������λ������
	kmeans(Points, number, labels, TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01), 3, KMEANS_PP_CENTERS, centers);
	//���ݷ���Ϊÿ�������ò�ͬ����ɫ
	for (i = 0; i < Points_num; i++)
	{
		int index = labels.at<int>(i);
		Point point = Points.at<Point2f>(i);
		circle(image, point, 2, colors[index], -1, 4);
	}
	//����ÿ�����������������Բ
	for (i = 0; i < centers.rows; i++)
	{
		int x = centers.at<float>(i, 0);
		int y = centers.at<float>(i, 1);
		cout << "��" << i + 1 << "����������꣺x = " << x << " y = " << y << endl;
		circle(image, Point(x, y), 50, colors[i], 1, LINE_AA);
	}
	imshow("K��ֵ���������", image);
	waitKey(0);
	
	Mat image_ = imread("../lena.jpg");
	if (image_.empty())
		exit(-1);
	Vec3b colorLut[3] = { Vec3b(0, 0, 255), Vec3b(0, 255, 0), Vec3b(255, 0, 0) };
	//ͼ��ߴ磬���ڼ���ͼ�������ص����Ŀ
	int width = image_.cols;
	int height = image_.rows;
	//��ʼ������
	int sampleCount = width * height;
	//��ͼ���������ת����ÿһ��һ�����ݵ���ʽ
	Mat sample_data = image_.reshape(3, sampleCount);
	Mat data;
	sample_data.convertTo(data, CV_32F);
	//KMeans����������ֵ���з���
	int number_ = 3;	//�ָ�����ɫ����
	Mat labels_;
	TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 0.1);
	kmeans(data, number_, labels_, criteria, number_, KMEANS_PP_CENTERS);
	//��ʾͼ��ָ���
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
	namedWindow("ԭͼ", WINDOW_NORMAL);
	imshow("ԭͼ", image_);
	namedWindow("�ָ��ͼ��", WINDOW_NORMAL);
	imshow("�ָ��ͼ��", result);
	waitKey(0);
}
//�������������ģ��
void test59()
{
	using namespace cv::dnn;
	system("color F0");
	string model = "../googlenet/bvlc_googlenet.caffemodel";
	string config = "../googlenet/bvlc_googlenet.prototxt";
	//����ģ��
	Net net = dnn::readNet(model, config);
	if (net.empty())
		exit(-1);
	//��ȡÿһ����Ϣ
	vector<string> layerNames = net.getLayerNames();
	for (int i = 0; i < layerNames.size(); i++)
	{
		//��ȡÿһ�������ID
		int ID = net.getLayerId(layerNames[i]);
		//��ȡÿһ���������Ϣ
		Ptr<Layer> layer = net.getLayer(ID);
		cout << "���������" << ID << "  ��������ƣ�" << layerNames[i] << endl
			<< "��������ͣ�" << layer->type.c_str() << endl;

	}
}
//���������ģ�͵�ʹ��
void test60()	//��Ʒʱ���
{
	using namespace cv::dnn;
	Mat image = imread("../airplane.jpg");
	if (image.empty())
		exit(-1);

	//��ȡ������������
	String typeListFile = "../image_recognition/imagenet_comp_graph_label_strings.txt";
	vector<String> typeList;
	ifstream file(typeListFile);
	if (!file.is_open())
		exit(-1);
	string type;
	while (!file.eof())
	{
		getline(file, type);	//��ȡ����
		if (type.length())
			typeList.push_back(type);
	}
	file.close();
	//��������
	String tf_pb_file = "../image_recognition/tensorflow_inception_graph.pb";
	Net net = readNet(tf_pb_file);
	if (net.empty())
		exit(-1);
	//������ͼ��������ݴ���
	Mat blob = blobFromImage(image, 1.0f, Size(224, 224), Scalar(), true, false);
	//����ͼ������Ԥ��
	Mat prob;
	net.setInput(blob, "input");
	prob = net.forward("softmax2");
	//�õ�����ܷ������
	Mat probMat = prob.reshape(1, 1);
	Point classNumber;
	double classProb;	//��������
	minMaxLoc(probMat, NULL, &classProb, NULL, &classNumber);

	string typeName = typeList.at(classNumber.x).c_str();
	cout << "�������Ϊ��" << typeName << "  ������Ϊ��" << classProb;
	//�������
	string str = typeName + " Possibility: " + to_string(classProb);
	putText(image, str, Point(25, 50), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2, 8);
	imshow("ͼ���жϽ��", image);
	waitKey(0);
}

void test61()	//ͼ����
{
	using namespace cv::dnn;
	Mat image = imread("../lena.jpg");
	String models[5] = { "candy.t7", "feathers.t7", "la_muse.t7", "starry_night.t7", "the_scream.t7" };
	for (int i = 0; i < size(models); i++)
	{
		Net net = readNet("../fast_style/" + models[i]);
		imshow("ԭʼͼ��", image);
		//����ͼ��ÿ��ͨ���ľ�ֵ
		Scalar imageMean = mean(image);
		//����ͼ��ߴ��ʽ
		Mat blobImage = blobFromImage(image, 1.0, Size(256, 256), imageMean, false, false);
		//���������ԭͼ������
		net.setInput(blobImage);
		Mat output = net.forward();
		//�������ĳߴ��ͨ����
		int outputChannels = output.size[1];
		int outputRows = output.size[2];
		int outputCols = output.size[3];
		//����������ŵ�ͼ����
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
		//��Ǩ�ƽ�����н�һ����������
		//�ظ�ͼ������ľ�ֵ
		result = result + imageMean;
		//��ͼ����й�һ��������ͼ����ʾ
		result = result / 255.0;
		//����ͼ��ߴ磬ʹ����ԭͼ��ߴ���ͬ
		resize(result, result, image.size());
		//��ʾ���
		imshow("��" + to_string(i + 1) + "�ַ��Ǩ�ƽ��", result);
		waitKey(0);
	}
}

void test62()	//���������Ա��б�
{
	using namespace cv::dnn;
	Mat image = imread("../faces.jpg");
	if (image.empty())
		exit(-1);
	//��ȡ����ʶ��ģ��
	String model_bin = "../face_gender/opencv_face_detector_uint8.pb";
	String config_text = "../face_gender/opencv_face_detector.pbtxt";
	Net faceNet = readNet(model_bin, config_text);
	//��ȡ�Ա���ģ��
	String genderModel = "../face_gender/gender_net.caffemodel";
	String genderProto = "../face_gender/gender_deploy.prototxt";
	String genderList[] = { "Male", "Female" };
	Net genderNet = readNet(genderModel, genderProto);
	if (faceNet.empty() || genderNet.empty())
		exit(-1);
	//������ͼ������������
	Mat blobImage = blobFromImage(image, 1.0, Size(300, 300), Scalar(), false, false);
	faceNet.setInput(blobImage, "data");
	Mat detect = faceNet.forward("detection_out");
	//�������ʡ��������������λ��
	Mat detectionMat(detect.size[2], detect.size[3], CV_32F, detect.ptr<float>());
	//��ÿ��������������Ա���
	int exBoundray = 20;	//ÿ�����������ĸ���������ߴ�
	float confidenceThreshold = 0.5;	//�ж�Ϊ�����ĸ�����ֵ����ֵԽ��׼ȷ��Խ��
	for (int i = 0; i < detectionMat.rows; i++)
	{
		float confidence = detectionMat.at<float>(i, 2);	//���Ϊ�����ĸ���
		if (confidence > confidenceThreshold)
		{
			//���������������С
			int topLx = detectionMat.at<float>(i, 3) * image.cols;
			int topLy = detectionMat.at <float>(i, 4) * image.rows;
			int bottomRx = detectionMat.at<float>(i, 5) * image.cols;
			int bottomRy = detectionMat.at<float>(i, 6) * image.rows;
			Rect faceRect(topLx, topLy, bottomRx - topLx, bottomRy - topLy);
			//���������������ߴ�������䣬Ҫע���ֹ�ߴ���ͼ����ʵ�ߴ�֮��
			Rect faceTextRect;
			faceTextRect.x = max(0, faceRect.x - exBoundray);
			faceTextRect.y = max(0, faceRect.y - exBoundray);
			faceTextRect.width = min(faceRect.width + exBoundray, image.cols - 1);
			faceTextRect.height = min(faceRect.height + exBoundray, image.rows - 1);
			Mat face = image(faceTextRect);	//����������ͼ��
			//�����沿ͼ��ߴ�
			Mat faceblob = blobFromImage(face, 1.0, Size(227, 227), Scalar(), false, false);
			//����������沿ͼ�����뵽�Ա�������
 			genderNet.setInput(faceblob);
			//��������
			Mat genderPreds = genderNet.forward();	//�����Ա�Ŀ�����
			//�Ա�����
			float male, female;
			male = genderPreds.at<float>(0, 0);
			female = genderPreds.at<float>(0, 1);
			int classID = male > female ? 0 : 1;
			String gender = genderList[classID];
			//��ԭͼ���л����沿�������Ա�
			rectangle(image, faceRect, Scalar(0, 0, 255), 2, 8, 0);
			putText(image, gender.c_str(), faceRect.tl(), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, 8);
		}	
	}
	imshow("�����", image);
	waitKey(0);
}
//�ලѧϰ����	K���ڷ��෽��
void test63()
{
	using namespace cv::ml;
	Mat image = imread("../handwriting.png");
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	//�ָ�Ϊ5000��cells
	Mat images = Mat::zeros(5000, 400, CV_8UC1);
	Mat labels = Mat::zeros(5000, 1, CV_8UC1);

	int index = 0;
	Rect numberImage;
	numberImage.x = 0;
	numberImage.height = 1;
	numberImage.width = 400;
	for (int row = 0; row < 50; row++)
	{
		//��ͼ���зָ��20*20��ͼ����Ϊ��������ͼ��
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
			//����άͼƬ����ת����������
			Mat row = number.reshape(1, 1);
			cout << "��ȡ��" << index + 1 << "������" << endl;
			numberImage.y = index;
			//��ӵ���������
			row.copyTo(images(numberImage));
			//��¼ÿ��ͼ���Ӧ�����ֱ�ǩ
			labels.at<uchar>(index, 0) = label;
			index++;
 		}
	}
	imwrite("../�������ݰ������н��.png", images);
	imwrite("../��ǩ.png", labels);
	//����ѵ�����ݼ�
	images.convertTo(images, CV_32FC1);
	labels.convertTo(labels, CV_32SC1);
	Ptr<ml::TrainData> tdata = ml::TrainData::create(images, ml::ROW_SAMPLE, labels);
	//����K������
	Ptr<KNearest> knn = KNearest::create();
	knn->setDefaultK(5);	//ÿ������ó�5������
	knn->setIsClassifier(true);	//���з���
	//ѵ������
	knn->train(tdata);
	//����ѵ������
	knn->save("../knn_model.ymal");
	//��������н����ʾ
	cout << "��ʹ��K�����������ѵ���ͱ���" << endl;
	waitKey(0);
}
//ֻ��ʶ��ڵװ�������0~9
void test64()	//����ѵ����ģ��
{
	using namespace cv::ml;
	//����KNN������
	Mat data = imread("../�������ݰ������н��.png", IMREAD_ANYDEPTH);
	Mat labels = imread("../��ǩ.png", IMREAD_ANYDEPTH);
	data.convertTo(data, CV_32FC1);
	labels.convertTo(labels, CV_32SC1);
	Ptr<KNearest> knn = Algorithm::load<KNearest>("../knn_model.ymal");
	//�鿴������
	Mat result;
	knn->findNearest(data, 5, result);
	//ͳ�Ʒ���������ʵ�����ͬ����Ŀ
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
	cout << "�������ȷ�ԣ�" << rate << endl;
	//������ͼ���Ƿ���ʶ������
	Mat testImage1 = imread("../handwritingtest1.png", IMREAD_GRAYSCALE);
	Mat testImage2 = imread("../handwritingtest2.png", IMREAD_GRAYSCALE);
	imshow("testImage1", testImage1);
	imshow("testImage2", testImage2);
	//���ŵ�20*20�ĳߴ�
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
	//��������ת��
	testdata.convertTo(testdata, CV_32F);
	//���й���ʶ��
	Mat result2;
	knn->findNearest(testdata, 5, result2);
	//�鿴Ԥ����
	for (int i = 0; i < result2.rows; i++)
	{
		int predict = result2.at<float>(i, 0);
		cout << "��" << i + 1 << "ͼ��Ԥ������" << predict
			<< "  ��ʵ�����" << i + 1 << endl;
	}
	waitKey(0);
}
//֧���������ķ��෽��
void test65()
{
	using namespace cv::ml;
	//ѵ������
	Mat samples, labels;
	FileStorage fread("../point.yml", FileStorage::READ);
	fread["data"] >> samples;
	fread["labls"] >> labels;
	fread.release();
	//��ͬ��������ӵ�в�ͬ��ɫ
	vector<Vec3b> colors;
	colors.push_back(Vec3b(0, 255, 0));
	colors.push_back(Vec3b(0, 0, 255));
	//�����հ�ͼ��������ʾ�����
	Mat image(480, 640, CV_8UC3, Scalar(255, 255, 255));
	Mat image_;
	image.copyTo(image_);
	//�ڿհ�ͼ���л��������
	for (int i = 0; i < samples.rows; i++)
	{
		Point2f point;
		point.x = samples.at<float>(i, 0);
		point.y = samples.at<float>(i, 1);
		Scalar color = colors[labels.at<int>(i, 0)];
		circle(image, point, 3, color, -1);
		circle(image_, point, 3, color, -1);
	}
	imshow("�������ص�ͼ��", image);
	//����ģ��
	Ptr<SVM> model = SVM::create();
	//��������
	model->setKernel(SVM::INTER);	//�����ں�
	model->setType(SVM::C_SVC);	//����SVM����
	model->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 100, 0.01));
	model->setGamma(5.383);
	model->setC(0.01);
	model->setDegree(3);
	//ѵ��ģ��
	model->train(TrainData::create(samples, ROW_SAMPLE, labels));
	//��ģ�Ͷ�ͼ���е�ȫ�����ص���з���
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
	imshow("ͼ���������ص������", image_);
	waitKey();
}

int main()
{
	//test01();	//Mat��
	//test02();	//ͼ���ȡ��ʾ�뱣��
	//test03();	//��Ƶ����������ͷʹ��
	//test04();	//��ɫ�ռ�任
	//test05();	//��ͨ���ķ�����ϲ�
	//test06();	//���رȽ�
	//test07();	//�߼�����
	//test08();	//ͼ���ֵ��
	//test09();	//LUT���ұ�
	//test10();	//�ߴ�任
	//test11();	//����任
	//test12();	//͸�ӱ任
	//test13();	//���ƻ���ͼ��
	//test14();	//ROI�����ȡ
	//test15();	//��˹&������˹ͼ�������
	//test16();	//������
	//test17();	//�����Ӧ
	//test18();	//ֱ��ͼ����
	//test19();	//ֱ��ͼ���⻯
	//test20();	//ֱ��ͼƥ��
	//test21();	//ģ��ƥ��
	//test22();	//ͼ��ľ��
	//test23();	//���������͸�˹����
	//test24();	//���������͸�˹����
	//test25();	//��ֵ�˲�
	//test26();	//�˲��ĵ�����
	//test27();	//Soble��Scharr��Ե�������
	//test28();	//������˹��Canny��Ե���
	//test29();	//��ͨ�����
	//test30();	//ͼ�����仯
	//test31();	//ͼ��ʴ����
	//test32();	//ͼ�����Ͳ���
	//test33();	//��ͼ�������̬ѧ����
	//test34();	//ͼ��ϸ��
	//test35();	//�������
	//test36();	//������Ϣͳ��
	//test37();	//������Ӷ����
	//test38();	//͹�����
	//test39();	//ֱ�߼��
	//test40();	//ֱ�����
	//test41();	//��ά��ʶ��
	//test42();	//ͼ��Ļ������
	//test43();	//��ˮ��䷨
	//test44();	//��ˮ�뷨
	//test45();	//Harris�ǵ�
	//test46();	//Shi-Tomas�ǵ���
	//test47();	//�����ؼ���ǵ��Ż�
	//test48(); //ORB ������
	//test49();	//������ƥ��
	//test50();	//RANSCA �Ż�������ƥ��
	//test51();	//���ģ����ͶӰ
	//test52();	//��Ŀ����ı궨
	//test53();	//ͼ���У��
	//test54();	//��Ŀ���λ�˹���
	//test55();	//��ֵ����ƶ�����
	//test56();	//���ܹ����������ƶ�����
	//test57();	//ϡ������������ƶ�����
	//test58();	//K��ֵ����
	//test59();	//�������������ģ��
	//���������ģ�͵�ʹ��
	//test60();	//������
	//test61();	//ͼ����
	//test62();	//����ʶ����Ա��б�
	//test63();	//�ලѧϰ����
	//test64();	//����ѵ��ģ��
	test65();	//������

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
	if (event == EVENT_RBUTTONDOWN)	//��������Ҽ�
	{
		cout << "����������ſɻ���ͼ��" << endl;
	}
	if (event == EVENT_LBUTTONDOWN)	//�������������������
	{
		prePoint = Point(x, y);
		cout << "�켣��ʼ���꣺" << prePoint << endl;
	}
	
	if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON))	//��ס�������ƶ�
	{
		//ͨ������ֱ����ʾ����ƶ��켣
		Point pt(x, y);
		line(img, prePoint, pt, Scalar(255, 255, 255), 4, 5, 0);
		prePoint = pt;
		imshow("����1", img);
		//ͨ���ı�ͼ�����ػ�����ʾ����ƶ��켣
		imgPoint.at<Vec3b>(y, x) = Vec3b(255, 255, 255);
		imgPoint.at<Vec3b>(y, x - 1) = Vec3b(255, 255, 255);
		imgPoint.at<Vec3b>(y, x + 1) = Vec3b(255, 255, 255);
		imgPoint.at<Vec3b>(y - 1, x) = Vec3b(255, 255, 255);
		imgPoint.at<Vec3b>(y + 1, x) = Vec3b(255, 255, 255);
		imshow("����2", imgPoint);
	}
}

void drawHist(Mat& gray, Mat& hist, string name)
{
	//�������ֱ��ͼ
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
	//��һ����������
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
		int i = cvflann::rand_int() % image.rows;	//����������������
		int j = cvflann::rand_int() % image.cols;	//����������������
		if (image.type() == CV_8UC1)	//��ɫͼ���������
		{
			int flag = cvflann::rand_int() % 2;	//��������ɫ
			if (flag == 0)	//��Ӻ�ɫ����
				image.at<uchar>(i, j) = 0;
			else	//��ɫͼ����Ӱ�ɫ����
				image.at<uchar>(i, j) = 255;
		}
		else
		{
			int channels = cvflann::rand_int() % 8;	//7(111)��������ͨ����Ϊ��ɫ
			int B = (channels & 4) >> 2;	//�ж���ɫͨ��Ϊ0����255
			int G = (channels & 2) >> 1;	//�ж���ɫͨ��Ϊ0����255
			int R = channels & 1;			//�жϺ�ɫͨ��Ϊ0����255
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
		//�������ȷ����ɫ
		Vec3b v3 = Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		colors_new.push_back(v3);
	}
	//�ò�ͬ��ɫ�����ͨ��
	for (int i = 0; i < number; i++)
	{
		//ÿ����ͨ����������
		int center_x = centroids.at<double>(i, 0);
		int center_y = centroids.at<double>(i, 1);
		//ÿ����ͨ���������Ϣ
		int x = stats.at<int>(i, CC_STAT_LEFT);
		int y = stats.at<int>(i, CC_STAT_TOP);
		int width = stats.at<int>(i, CC_STAT_WIDTH);
		int height = stats.at<int>(i, CC_STAT_HEIGHT);
		int area = stats.at<int>(i, CC_STAT_AREA);
		//Ȧ������λ��
		circle(image, Point(center_x, center_y), 2, Scalar(0, 255, 0), 2, 8, 0);
		//��Ӿ���
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
		//���ƶ����
		Vec2i pre = input.at<Vec2i>(p);
		Vec2i post = input.at<Vec2i>((p + 1) % input.rows);	//preΪ���һ����ʱ��post��Ϊ�������
		line(image, pre, post, Scalar(0, 0, 255), 2, 8, 0);
	}

}

void drawLines(Mat& image, vector<Vec2f> lines, Scalar scalar, int thickness)
{
	//image����ֱ�ߵ�ͼ�Σ�lines����ֱ�ߵ����ݣ� ͼ���У��У� ����ֱ�ߵ���ɫ�� �߿�
	Point p1, p2;
	for (int i = 0; i < lines.size(); i++)
	{
		double rho = lines[i][0];	//ֱ�߾������ԭ��ľ���
		double theta = lines[i][1];	//ֱ�߹�����ԭ�㴹����x��ļн�
		double a = cos(theta);	//�нǵ�����ֵ
		double b = sin(theta);	//�нǵ�����ֵ
		double x0 = rho * a, y0 = rho * b;	//��ԭ�㴹����ֱ�ߵĽ���
		double length = max(image.rows, image.cols);	//ͼ��߶ȵ����ֵ
		//ֱ���ϵ�һ��(���������뽹��Զ)
		p1.x = cvRound(x0 + length * (-b));
		p1.y = cvRound(y0 + length * a);
		//ֱ���ϵ���һ��(���������뽹��Զ)
		p2.x = cvRound(x0 - length * (-b));
		p2.y = cvRound(y0 - length * a);
		//����һ��
		line(image, p1, p2, scalar, thickness);
	}
}

void orb_features(Mat& image, vector<KeyPoint>& keyPoints, Mat& descriptions)
{
	//���� ORB �����������
	Ptr<ORB> orb = ORB::create(1000,	//��������Ŀ
		1.2f,	//�������㼶֮������ű���
		8,	//����������ϵ��
		31,	//��Ե��ֵ
		0,	//ԭͼ�ڽ������еĲ���
		2,	//����������ʱ��Ҫ�õ����ص���Ŀ
		ORB::HARRIS_SCORE,	//ʹ�� HARRIS ��������������
		31,	//����������ʱ�ؼ�����Χ����ߴ�
		20	//���� FAST �ǵ�ʱ����ֵ��ֵ����ֵ
	);
	//����ؼ���
	orb->detect(image, keyPoints);
	//���� ORB ������
	orb->compute(image, keyPoints, descriptions);
}

void match_min(vector<DMatch>& matches, vector<DMatch>& good_matches)
{
	//ͨ����������ɸѡƥ����
	double min_dist = 10000, max_dist = 0;
	for (int i = 0; i < matches.size(); i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist)	min_dist = dist;
		if (dist > max_dist)	max_dist = dist;
	}
	//���������С�ĺ�������
	cout << "min_dist = " << min_dist << endl;
	cout << "max_dist = " << max_dist << endl;
	//�����������Զ��ɾ��
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 20.0))
			good_matches.push_back(matches[i]);
	}
	cout << "good_matches = " << good_matches.size() << endl;	//ʣ����������Ŀ
}

void ransac(vector<DMatch>& matches, vector<KeyPoint>& queryKeyPoint, vector<KeyPoint>& trainKeyPoint, vector<DMatch>& matches_ransac)
{
	//���屣��ƥ��������
	vector<Point2f> srcPoints(matches.size()), dstPoints(matches.size());
	//����ӹؼ�������ȡ��ƥ��������
	for (int i = 0; i < matches.size(); i++)
	{
		srcPoints[i] = queryKeyPoint[matches[i].queryIdx].pt;
		dstPoints[i] = trainKeyPoint[matches[i].trainIdx].pt;
	}
	//ƥ���Խ���ransac����
	vector<int> inliersMask(srcPoints.size());
	//Mat homography;
	//homography = findHomography(srcPoints, dstPoints, RANSAC, 5, inliersMask);
	findHomography(srcPoints, dstPoints, RANSAC, 5, inliersMask);
	//�ֶ�����RANSAC���˺��ƥ����
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