#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Aria.h"
#include <termios.h>	// POSIX terminal control definitionss
#include <pthread.h>
#include <string.h>
#include <unistd.h>		// UNIX standard function definitions
#include <fcntl.h>		// File control definitions
#include <fstream>

using namespace cv;
using namespace std;

#ifndef PI
#define PI 3.1415926f
#endif

Mat imgOriginal, imgHSV, element, imgThresholded;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

//��ʼ��Ҫ���˵���ɫ����ɫ
int iLowH = 0;
int iHighH = 10;

int iLowS = 90;
int iHighS = 255;

int iLowV = 35;
int iHighV = 200;

/// Function header
void *hsv_thread(void* ha);

// open camera
VideoCapture cap(1);

//���ÿ��Ʊ�־
bool fstop = false;

//������
float sub_width[2], sub_height[2], sub_pos_real[2];
float avg_width, avg_height, pos_real, dis_real, ang_real;
unsigned int Ellipse_count = 0;

//�ο�����
float height_real = 0.045;  //Բ��ʵ�ʸ߶�
float dis_ref = 0.5, ang_ref = 0.52, pos_ref = 320;

//��õĲ������ڹ�����ʹ��
float offset_real;  //ʵ�����쵼����С��
float th1, th2;      //���������쵼������֮��нǣ������߷ֱ����쵼�ߣ������쵼��֮��ļн�
float xe, ye, the;    //�Ը�����Ϊ�ο�ϵ�µ������쵼�ߵ�����
float dis_virtual;  //�������������쵼��ʱ��ľ���

float kx = 0.25, ky = 1, kth = 1, kd = 3;  //ϵ�����ɵ�
float v_control, w_control;  //��������

//data saving
ofstream oData;

/** @function main */
int main(int argc, char** argv)
{
	//�ж�����ͷ�������
	if (!cap.isOpened())
	{
		return -1;
	}

	//create a window called "Control"
	namedWindow("Image", CV_WINDOW_AUTOSIZE);

	//amigo���ӳ�ʼ��
	ArRobot robot;
	Aria::init();

	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArLog::log(ArLog::Terse, "Be careful! You can not afford it!!!");
	// ArRobotConnector connects to the robot, get some initial data from it such as type and name,
	// and then loads parameter files for this robot.
	ArRobotConnector robotConnector(&parser, &robot);
	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (parser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
			return 1;
		}
	}
	if (!Aria::parseArgs())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}

	//oData.open("data.txt", ios::out | ios::trunc);
	//oData << "dis_error\tpos_error\theight\twidth" << endl;
	// add pthread
	pthread_t pthread_camera;
	pthread_create(&pthread_camera, NULL, hsv_thread, NULL);


	ArLog::log(ArLog::Normal, "Connected.");
	// Start the robot processing cycle running in the background.
	// True parameter means that if the connection is lost, then the
	// run loop ends.
	robot.runAsync(true);
	// Print out some data from the SIP.
	// We must "lock" the ArRobot object
	// before calling its methods, and "unlock" when done, to prevent conflicts
	// with the background thread started by the call to robot.runAsync() above.
	// See the section on threading in the manual for more about this.
	// Make sure you unlock before any sleep() call or any other code that will
	// take some time; if the robot remains locked during that time, then
	// ArRobot's background thread will be blocked and unable to communicate with
	// the robot, call tasks, etc.
	robot.lock();
	ArLog::log(ArLog::Normal, "Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
		robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
	robot.unlock();

	// Sleep for 3 seconds.
	ArLog::log(ArLog::Normal, "Will start driving in 3 seconds...");
	ArUtil::sleep(3000);

	// The main part!!
	ArLog::log(ArLog::Normal, "Go test...");
	robot.lock();
	robot.enableMotors();
	while (fabsf(dis_real - dis_ref)>0.02 || fabsf(pos_ref - pos_real)>10)  //���10�ȣ�2����
	{
		//offset_real = height_real * (pos_ref-pos_real) / avg_height;
		//th1 = asinf(offset_real/dis_real);
		//dis_virtual = sqrtf(dis_real*dis_real+dis_ref*dis_ref-2*dis_ref*dis_real*cosf(ang_real));
		//th2 = asinf(sinf(ang_real)*dis_ref/dis_virtual);
		//the = th1-th2;
		//xe = dis_virtual * cosf(the);
		//ye = dis_virtual * sinf(the);
		//v_control = kx * (1-expf(-0.5*xe))/(1+expf(-0.5*xe)) + kd * (1-expf(-0.5*(dis_real-dis_ref)))/(1+expf(-0.5*(dis_real-dis_ref)));
		//w_control = ky * (1-expf(-0.5*ye))/(1+expf(-0.5*ye)) + kth * (1-expf(-0.5*the))/(1+expf(-0.5*the));
		v_control = kd * (1 - expf(-0.5*(dis_real - dis_ref))) / (1 + expf(-0.5*(dis_real - dis_ref)));
		w_control = kx * (1 - expf(-0.5*(pos_ref - pos_real) / 100)) / (1 + expf(-0.5*(pos_ref - pos_real) / 100));
		//if(fabsf(pos_ref-pos_real) < 100)
		//	w_control += kth * (1-expf(-0.5*(ang_real-ang_ref)))/(1+expf(-0.5*(ang_real-ang_ref)));
		robot.setVel(v_control * 1000);//v_control*1000
		robot.setRotVel(w_control * 180 / PI);//w_control*180/PI

		robot.unlock();
		ArUtil::sleep(10);
		robot.lock();
	}

	robot.setVel(0);
	robot.setRotVel(0);
	robot.unlock();
	ArUtil::sleep(1000);

	ArLog::log(ArLog::Normal, "Stopping.");
	robot.lock();
	robot.stop();
	robot.unlock();
	ArUtil::sleep(1000);

	ArLog::log(ArLog::Normal, "Ending robot thread...");
	robot.stopRunning();
	// wait for the thread to stop
	robot.waitForRunExit();
	// exit
	ArLog::log(ArLog::Normal, "Exiting...");
	Aria::exit(0);

	return(0);
}

void *hsv_thread(void* ha = NULL)
{
	while (!fstop)
	{
		// read a new frame from video
		bool bSuccess = cap.read(imgOriginal);
		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		//Convert the captured frame from BGR to HSV
		vector<Mat> hsvSplit;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		//��Ϊ���Ƕ�ȡ���ǲ�ɫͼ��ֱ��ͼ���⻯��Ҫ��HSV�ռ���
		split(imgHSV, hsvSplit);
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		merge(hsvSplit, imgHSV);

		//Threshold the image
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

		//������ (ȥ��һЩ���)
		//element = getStructuringElement(MORPH_RECT, Size(5, 5));
		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

		//�ղ��� (����һЩ��ͨ��)
		morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

		//Ѱ�������ͨͼ
		vector<Point> maxcont;
		float maxarea = 0;
		findContours(imgThresholded, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));
		for (unsigned i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > maxarea)
			{
				maxarea = contourArea(contours[i]);
				maxcont = contours[i];
			}
		}

		//�������ο�
		Rect rect = boundingRect(maxcont);
		rectangle(imgThresholded, rect, Scalar(255, 0, 255), 3);

		//�Ծ��ο����
		pos_real = rect.x + rect.width/2.0;
		dis_real = (rect.height*(-1/6.0)+72)/100;
		printf("%f\n",dis_real);

		//show the thresholded image
		imshow("Image", imgThresholded);

		if (waitKey(30) >= 0)
			fstop = true;
	}
	return NULL;
}
