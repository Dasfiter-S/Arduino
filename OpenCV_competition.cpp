#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
//#include "hiredis/hiredis.h"
//#include <execinfo.h>
//#include <signal.h>
#include <stdlib.h>
//#include <unistd.h>

#define DEBUG

using namespace std;
using namespace cv;

#define PI 3.14159265
//redisContext *c;
//#define REDIS_SERVER "127.0.0.1"
//#define VIDEO_SOURCE "http://192.168.43.206/cam_pic_new.php?pDelay=100000&x.mjpg"
//#define VIDEO_SOURCE "C:\\Users\\Dasfiter\\Videos\\vi_0003_20160317_155550-320.m4v"
#define VIDEO_SOURCE "C:\\Users\\Dasfiter\\Videos\\vi_0004_20160331_145026.mp4"


// 768*576
/*
void redisTest() {
	redisReply *reply;
	reply = (redisReply *)redisCommand(c, "SET %s %s", "key", "hello world");
	printf("SET: %s\n", reply->str);
	freeReplyObject(reply);

	reply = (redisReply *)redisCommand(c, "GET key");
	printf("GET key: %s\n", reply->str);
	freeReplyObject(reply);

	reply = (redisReply *)redisCommand(c, "PUBLISH VISION UPDATE");
	freeReplyObject(reply);
}

void redis() {
	struct timeval timeout = { 1, 500000 }; // 1.5 seconds
	c = redisConnectWithTimeout(REDIS_SERVER, 6379, timeout);
	if (c == NULL || c->err) {
		if (c) {
			printf("Connection error: %s\n", c->errstr);
			redisFree(c);
		}
		else {
			printf("Connection error: can't allocate redis context\n");
		}
		exit(1);
	}
}
*/
#ifdef DEBUG

void createDebugWindows(int width, int height) {
	const int borderX = 5;
	const int borderY = 50;
	int w = width + borderX;
	int h = height + borderY;
	int currX = 0;
	int currY = 0;
	namedWindow("thresholds", WINDOW_AUTOSIZE); // Create a window for display.
	cvMoveWindow("thresholds", 400, 0);

	namedWindow("human", WINDOW_AUTOSIZE);
	cvMoveWindow("human", currX, currY);

	currX += w;
	namedWindow("hsv", WINDOW_AUTOSIZE);
	cvMoveWindow("hsv", currX, currY);

	currX += w;
	namedWindow("grayscale", WINDOW_AUTOSIZE);
	cvMoveWindow("grayscale", currX, currY);

	currX += w;
	namedWindow("binary", WINDOW_AUTOSIZE);
	cvMoveWindow("binary", currX, currY);

	currX = 0;
	currY += h;
	namedWindow("guide", WINDOW_AUTOSIZE);
	cvMoveWindow("guide", currX, currY);

	currX += w;
	namedWindow("line", WINDOW_AUTOSIZE);
	cvMoveWindow("line", currX, currY);

	currX += w;
	namedWindow("box", WINDOW_AUTOSIZE);
	cvMoveWindow("box", currX, currY);
}
#endif

void showHistogram(Mat& img) {
	int bins = 256; // number of bins
	int nc = img.channels(); // number of channels

	vector<Mat> hist(nc); // histogram arrays

	// Initalize histogram arrays
	for (int i = 0; i < hist.size(); i++)
		hist[i] = Mat::zeros(1, bins, CV_32SC1);

	// Calculate the histogram of the image
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			for (int k = 0; k < nc; k++) {
				uchar val = nc == 1 ? img.at<uchar>(i, j) : img.at<Vec3b>(i, j)[k];
				hist[k].at<int>(val) += 1;
			}
		}
	}

	// For each histogram arrays, obtain the maximum (peak) value
	// Needed to normalize the display later
	int hmax[3] = { 0, 0, 0 };
	for (int i = 0; i < nc; i++) {
		for (int j = 0; j < bins - 1; j++)
			hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
	}

	const char* wname[3] = { "blue", "green", "red" };
	Scalar colors[3] = { Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255) };

	vector<Mat> canvas(nc);

	// Display each histogram in a canvas
	for (int i = 0; i < nc; i++) {
		canvas[i] = Mat::ones(125, bins, CV_8UC3);

		for (int j = 0, rows = canvas[i].rows; j < bins - 1; j++) {
			line(
				canvas[i],
				Point(j, rows),
				Point(j, rows - (hist[i].at<int>(j) * rows / hmax[i])),
				nc == 1 ? Scalar(200, 200, 200) : colors[i],
				1, 8, 0
				);
		}

		imshow(nc == 1 ? "value" : wname[i], canvas[i]);
	}
}

double angle(Vec4i line) {
	// Calculate each line's angle
	double a = abs(line[0] - line[2]);
	if (a == 0) {
		return 0;
	}
	else {
		double angle = atan((line[1] - line[3]) / a) * 180 / PI;
		return angle;
	}
}

/**
* https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
*/
double distancePL(int x, int y, Vec4i line) {
	int x1 = line[0];
	int y1 = line[1];
	int x2 = line[2];
	int y2 = line[3];

	int numerator = abs(x * (y2 - y1) - y * (x2 - x1) + x2 * y1 + y2 * x1);
	double denominator = sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1));

	return numerator / denominator;
}

double distanceP2P(int x1, int y1, int x2, int y2){
	double distance = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
	return distance;
}

bool isLineExist(Vec4i line) {
	if (line[0] == 0 && line[1] == 0 && line[2] == 0 && line[3] == 0) {
		return false;
	}
	else {
		return true;
	}
}

int countLine(vector<Vec4i> lines) {
	int count = 0;
	for (size_t i = 0; i < lines.size(); i++) {
		if (isLineExist(lines[i])) {
			count++;
		}
	}
	return count;
}

Vec4i mergeLines(Vec4i* line1, Vec4i* line2) {
	//-------------------For x Lowest
	int xLowest, yLowest, xHighest, yHighest; xLowest = 0; yLowest = 0; xHighest = 0; yHighest = 0;
	if (line1->val[0] < line2->val[0]) { //Finding which value is larger
		xLowest = line1->val[0];
	}
	if (line1->val[0] > line2->val[0]) {//same
		xLowest = line2->val[0];
	}
	//--------------------For y Lowest
	if (line1->val[1] < line2->val[1]) { //Finding which value is larger
		yLowest = line1->val[1];
	}
	if (line1->val[1] > line2->val[1]) {//same
		yLowest = line2->val[1];
	}
	//--------------------For x Highest
	if (line1->val[2] > line2->val[2]) { //Finding which value is larger
		xHighest = line1->val[2];
	}
	if (line1->val[2] < line2->val[2]) {//same
		xHighest = line2->val[2];
	}
	//--------------------For y Highest
	if (line1->val[3] > line2->val[3]) { //Finding which value is larger
		yHighest = line1->val[3];
	}
	if (line1->val[3] < line2->val[3]) {//same
		yHighest = line2->val[3];
	}
	Vec4i* lineResult;
	lineResult->val[0] = xLowest;
	lineResult->val[1] = yLowest;
	lineResult->val[2] = xHighest;
	lineResult->val[3] = yHighest;

	return *lineResult;
}

void deleteLine(Vec4i* line) {
	line->val[0] = 0;
	line->val[1] = 0;
	line->val[2] = 0;
	line->val[3] = 0;
}
/*
void handler(int sig) {
	void *array[10];
	size_t size;

	// get void*'s for all entries on the stack
	size = backtrace(array, 10);

	// print out all the frames to stderr
	fprintf(stderr, "Error: signal %d:\n", sig);
	backtrace_symbols_fd(array, size, STDERR_FILENO);
	exit(1);
}
*/
int main(int argc, char** argv) {
	//signal(SIGSEGV, handler);
	//    redis();
	//    redisTest();
	VideoCapture cap;
	cap.open(VIDEO_SOURCE);

	if (cap.isOpened()) {
		cout << "Video: " <<
			"nframes=" << cap.get(CAP_PROP_FRAME_COUNT) <<
			", format=" << cap.get(CV_CAP_PROP_FORMAT) << endl;
	}
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	createDebugWindows(dWidth, dHeight);


	int lowThreshold = 100;
	int const max_lowThreshold = 100;
	int ratio = 3;
	int kernel_size = 3;

	int binary_low = 64;
	int binary_high = 255;
	createTrackbar("Canny:", "thresholds", &lowThreshold, max_lowThreshold);
	createTrackbar("Binary Low:", "thresholds", &binary_low, 255);
	createTrackbar("Binary High:", "thresholds", &binary_high, 255);

	Mat frame;
	for (;;) {

		if (!cap.read(frame))
			break;

		cv::imshow("human", frame);

		cv::Mat hsv;
		cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
		cv::imshow("hsv", hsv);

		cv::Mat grayscale;
		cv::cvtColor(frame, grayscale, cv::COLOR_BGR2GRAY);
		cv::imshow("grayscale", grayscale);

		cv::Mat binary;
		cv::medianBlur(frame, binary, 11);
		cv::threshold(binary, binary, binary_low, binary_high, THRESH_BINARY);
		cv::imshow("binary", binary);

		// Remove channels
		//        Mat channel[3];
		//        Mat laser;
		//        split(frame, channel);
		//        channel[0] = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Set blue channel to 0
		//        channel[1] = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Set blue channel to 0
		//        merge(channel, 3, laser);

		cv::Mat redbox;
		cv::Mat lower_red_hue_range;
		cv::Mat upper_red_hue_range;
		cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
		cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
		cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, redbox);
		//cv::GaussianBlur(redbox, redbox, cv::Size(9, 9), 2, 2);
		cv::imshow("box", redbox);

		cv::Mat blackline;
		cv::threshold(grayscale, blackline, 50, 255, CV_THRESH_BINARY_INV);
		cv::imshow("line", blackline);

		cv::Mat guide = frame.clone();  {
			Mat contour = grayscale.clone();
			vector<Vec4i> lines;
			Canny(contour, contour, lowThreshold, lowThreshold*ratio, kernel_size);
			HoughLinesP(contour, lines, 1, CV_PI / 180, 60, 30, 10);
			//cornerHarris(contour, lines,);
			
			int redLineCount = 0;
			for (size_t i = 0; i < lines.size(); i++) {
				if (isLineExist(lines[i])) {
					redLineCount++;
					line(guide, Point(lines[i][0], lines[i][1]),
						Point(lines[i][2], lines[i][3]), Scalar(0, 255, 0), 3, 8);
				}
			}
			// Merge lines that are close to each other( angle within 4 degrees, distance within 15px )
			
			int lineCount = lines.size();
			int lineCount2;
			while (true) {
				for (size_t i = 0; i < lines.size(); i++) {
					double angle1 = angle(lines[i]);
					for (size_t j = i + 1; j < lines.size(); j++) {
						if (!isLineExist(lines[j])) {
							continue;
						}
						double angle2 = angle(lines[j]);
						if (abs(angle1 - angle2) > 2.0) {
							continue;
						}
						if (distanceP2P(lines[i][0], lines[i][1], lines[j][0], lines[j][1]) > 50 &&
							distanceP2P(lines[i][0], lines[i][1], lines[j][2], lines[j][3]) > 50 &&
							distanceP2P(lines[i][2], lines[i][3], lines[j][0], lines[j][1]) > 50 &&
							distanceP2P(lines[i][2], lines[i][3], lines[j][2], lines[j][3]) > 50) {
							continue;
						}
						cout << "merged" << endl;
						lines[i] = mergeLines(&lines[i], &lines[j]);
						deleteLine(&lines[j]);
					}
				}
				lineCount2 = countLine(lines);
				if (lineCount == lineCount2) {
					break;
				}
				else {
					lineCount = lineCount2;
				}
			}

			int greenLineCount = 0;
			for (size_t i = 0; i < lines.size(); i++) {
				if (isLineExist(lines[i])) {
					greenLineCount++;
					line(guide, Point(lines[i][0], lines[i][1]),
						Point(lines[i][2], lines[i][3]), Scalar(0, 255, 0), 3, 8);
				}
			}

			cout << "Red: " << redLineCount << "\t" << "Green: " << greenLineCount;
			if (redLineCount != greenLineCount){
				cout << " \t !!!!!!!!!!!!!!!!!!!!!!";
			}
			cout << endl;
			
		}
		cv::imshow("guide", guide);

		char key = cvWaitKey(10);
		if (key == 27) // ESC
			break;
		else if (key == 32) {
			cvWaitKey();
		}
	}
	//redisFree(c);
	return 0;
}
