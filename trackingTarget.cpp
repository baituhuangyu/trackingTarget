#include<iostream>
#include<opencv2/opencv.hpp>
//#include<opencv2/tracking/tracker.hpp>
#include<string>
#include<fstream>
#include<time.h>
#include <chrono>
#include "fdssttracker.hpp"

using namespace std;
using namespace cv;


Point previousPoint, currentPoint;
Rect2d bbox;
Point center;
//动态数组存储坐标点
vector<Point2d> points;

void draw_rectangle(int event, int x, int y, int flags, void*);
Mat tmp,frame,dst;


int main(int argc,char *argv[])
{
	VideoCapture cap;
	int count = 1;
	//按照视频实际路径更改
	//若是图片序列，自行修改其文件名遍历格式
	//例如：图像路径imgPath
    // "img00001.jpg"
	char name[10];
    std::sprintf(name, "img%05d", count);
//	std::string imgPath = "/media/cf/96A2CA30A2CA1521/downloads/Tracker_20230722/Tracker/fDSST/sequences/dog1/imgs/";
	std::string imgPath = "/mnt/sd/imgs/";
//    std::string imgPath = "/home/khadas/Desktop/imgs/";
	std::string imgFinalPath = imgPath + std::string(name) + ".jpg";
    printf("imgFinalPath:%s\n", imgFinalPath.c_str());

//	string filename = "/media/cf/96A2CA30A2CA1521/530/3.mp4";
//	cap.open(filename);
//	if (!cap.isOpened())
//	{
//		cout << "###############视频打开失败###############" << endl;
//		return -1;
//	}
//	cap.read(frame);
    frame = cv::imread(imgFinalPath);
	cvtColor(frame,dst, cv::COLOR_BGR2GRAY);
//    cv::imshow("output", frame);
//    cv::waitKey();

//	if (!frame.empty())
//	{
//		namedWindow("output", 0);
//		imshow("output", dst);
//		setMouseCallback("output", draw_rectangle, 0);
//		waitKey();
//	}

	/*********************Opencv目标追踪算法模板函数***************************/
	//Ptr<TrackerMIL> tracker=TrackerMIL::create();
	//Ptr<TrackerTLD> tracker=TrackerTLD::create();
	//Ptr<TrackerMedianFlow> tracker=TrackerMedianFlow::create();
	//Ptr<TrackerKCF> tracker=TrackerKCF::create();
	//Ptr<TrackerBoosting> tracker=TrackerBoosting::create();
	//Ptr<TrackerCSRT> tracker = TrackerCSRT::create();
	//Ptr<TrackerGOTURN> tracker = TrackerGOTURN::create();
	/***********************************************************************/

	/************************FDSST目标跟踪算法*********************************/
	/************************************************************************/

	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool SILENT = true;
	bool LAB = false;
	// Create DSSTTracker tracker object
	FDSSTTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	//ofstream infile;
	//infile.open("./coordxy.txt");
	double duration = 0;
	for(;;)
	{	
//		cap.read(frame);
//		if (!cap.read(frame))
//		{
//			break;
//		}
        if (count > 1350)
            break;
        std::sprintf(name, "img%05d", count);
        imgFinalPath = imgPath + std::string(name) + ".jpg";
        frame = cv::imread(imgFinalPath);


		cvtColor(frame, dst, cv::COLOR_BGR2GRAY);//fdsst源程序fhog只能传入灰度图片，因此对图片做一个灰度转化

		//计时打点开始
		auto t_start = clock();
        std::chrono::time_point<std::chrono::high_resolution_clock> p0 = std::chrono::high_resolution_clock::now();
		//字符数组存储坐标点输出名称
		//char target[30];
		if (frame.empty())
		{
			break;
		}
		if (count==1)
		{
			//传入首帧鼠标框选初始跟踪框bbox
		
			/*opencv自带算法*/
			//tracker->init(frame, bbox);
			/*改进算法*/
			//FDSST
//            bbox = {904, 611, 66, 50};
            bbox = {139,112,51,36};
			tracker.init(bbox, dst);
		}
		else {
			/*opencv自带算法*/
			//tracker->update(frame, bbox);
			/*改进算法*/
			//FDSST
			bbox = tracker.update(dst);
			
		}
		//计时函数计时打点结束
		auto t_end = clock();
        std::chrono::time_point<std::chrono::high_resolution_clock> p1 = std::chrono::high_resolution_clock::now();
        float cost_t = (float)std::chrono::duration_cast<std::chrono::microseconds>(p1-p0).count() / 1000.0f;
		duration += (double)(t_end - t_start) / CLOCKS_PER_SEC;
		//求FPS:
		cout << "FPS: " << count / duration << "\n";
		cout << "cost_t: " << cost_t << "ms" << "\n";
		count++;
		rectangle(frame, bbox, Scalar(255, 255, 0), 2, 1);
		center.x = bbox.x + bbox.width / 2;
		center.y = bbox.y + bbox.height / 2;
		points.push_back(center);
		//绘制轨迹跟踪框中心点图线
		for (int j = 1; j < points.size(); j++)
		{
			Point pre, last;
			if (points.size() < 3)
			{
				pre = points[j];
				last = points[j];
			}
			else
			{
				pre = points[j - 1];
				last = points[j];
				line(frame, pre, last, Scalar(0, 255, 0), 1, 8);
			}
		}
		circle(frame, center, 4, Scalar(0, 255, 0), -1);
		/*显示输出检测中心点位置坐标*/
		//sprintf(target, "检测框中心位置(%d,%d)", center.x, center.y);
		//cout << target << endl;
		/********************将检测框中心写入txt文件中*************************/
		/*
		infile << center.x << "\t" << center.y << endl;
		*/

//		namedWindow("tracking", 0);
//		imshow("tracking", frame);
//		int delayms = 1;
//		if (waitKey(delayms) == 27)
//			break;
		

	}
	//infile.close();
	return 0;
}
void draw_rectangle(int event, int x, int y, int flags, void*)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		previousPoint = Point(x, y);
		cout << "(" << previousPoint.x << "," << previousPoint.y << ")" << endl;
	}
	else if (event == EVENT_MOUSEMOVE && (flags&EVENT_FLAG_LBUTTON))
	{
		Mat tmp;
		char str[20];
		frame.copyTo(tmp);
		currentPoint = Point(x, y);
		sprintf(str, "(%d,%d)", currentPoint.x, currentPoint.y);
		putText(tmp, str, currentPoint, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 2, 8);
		rectangle(tmp, previousPoint, currentPoint, Scalar(0, 255, 0),2, 1, 0);
		imshow("output", tmp);
	}
	else if (event == EVENT_LBUTTONUP)
	{
		bbox.x = previousPoint.x;
		bbox.y = previousPoint.y;
		bbox.width = abs(previousPoint.x - currentPoint.x);
		bbox.height = abs(previousPoint.y - currentPoint.y);

	}
	else if (event == EVENT_RBUTTONUP)
		destroyWindow("output");
}
