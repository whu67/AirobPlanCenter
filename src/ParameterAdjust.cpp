/*
 * ParameterAdjust.cpp
 *
 *  Created on: May 25, 2016
 *      Author: root
 */
#include "ParameterAdjust.h"


#include <queue>
#include <set>
#include <map>
#include <opencv/cv.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <math.h>
#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>
#include "EstRXY.h"
using namespace cv;

//#define DEBUGPHOTO
//#define DEBUGSAVE
#define DEBUGPHOTOSAVE

#define USETENCENT
//#define USESCUT

CvCapture* capture;
float maxError = 0.11f;
std::map<int, Mat> PhotoMap;
std::map<int, Mat> PhotoAdjustMap;

ParameterAdjust::ParameterAdjust()
{
	// TODO Auto-generated constructor stub
	pthread_mutex_init(&AdjustThreadmutex, NULL);
	pthread_cond_init(&AdjustThreadEvent, NULL);
	pthread_mutex_init(&RevAdjustOrderQueuemutex, NULL);
	Adjustcount = 0;
}

ParameterAdjust::~ParameterAdjust()
{
	// TODO Auto-generated destructor stub
}

int ParameterAdjust::open(CommHelper* lp)
{
//	m_MainProgram = lp;
	m_CommHelperPtr = lp;

	pthread_attr_t child_thread_attr;
	if (0 != pthread_attr_init(&child_thread_attr))
		printf("Adjust pthread_attr_init failed\n");

	if (0 != pthread_attr_setdetachstate(&child_thread_attr,
			PTHREAD_CREATE_DETACHED)) {//此处设置成分离线程内存，一旦线程结束，立刻回收内存
		perror("Adjust pthread_attr_setdetachstate failed");
		return -1;
	}

	int result = pthread_create(&m_AdjustThreadHandle, &child_thread_attr,
			ParameterAdjustThreadFunc, (void*) this);
	if (result == -1) {
		printf("Adjust work pthread create failed\n");
	}
	fprintf(stdout, "Create ParameterAdjustThreadFunc thread successfully!\n");
	return 1;
}

bool ParameterAdjust::init()
{
	PhotoMap.clear();
	PhotoAdjustMap.clear();
	maxError = 0.06f;
	ResultOutStr.open("/home/fa/Photo/result.out", ios::out|ios::trunc);

//	capture = cvCreateCameraCapture(CV_CAP_ANY);
//	if(NULL == capture)
//		return false;
//	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 1280);
//	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 720);
	return true;
}

void ParameterAdjust::ClearPhotoMap()
{
	PhotoMap.clear();
	PhotoAdjustMap.clear();
}

void ParameterAdjust::AddToAdjustQueue(const char* data, int data_len, int priority)
{
	AdjustData m_RevAdjustData;
	m_RevAdjustData.data = new char[data_len + 1];
	Hexstrncpy(m_RevAdjustData.data, data, data_len);
	m_RevAdjustData.data[data_len] = 0;
	m_RevAdjustData.data_len = data_len;
	m_RevAdjustData.priority = priority;
	pthread_mutex_lock(&RevAdjustOrderQueuemutex);
	RevAdjustDataQueue.push(m_RevAdjustData);
	pthread_mutex_unlock(&RevAdjustOrderQueuemutex);

	pthread_mutex_lock(&AdjustThreadmutex);
	pthread_cond_signal(&AdjustThreadEvent);
	pthread_mutex_unlock(&AdjustThreadmutex);
	return;
}

bool ParameterAdjust::GetTmpPhoto(unsigned char Xindex, unsigned char Yindex, unsigned char position)
{
	capture = cvCreateCameraCapture(CV_CAP_ANY);
	if(NULL == capture)
		return false;
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 1280);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 720);

//	usleep(1000*1000);
	IplImage* frame;
	frame = cvQueryFrame(capture);
	if(!frame)
	{
		//return failure of get photo to robot
		fprintf(stdout, "Get adjusted photo failed!\n");
		ResultOutStr<<"Get adjusted photo failed!"<<endl;
		cvReleaseCapture(&capture);
		return false;
	}
	else
	{
#ifdef DEBUGPHOTO
		cvShowImage("vedio",frame);
		cvReleaseCapture(&capture);
		return true;
#endif
#ifdef DEBUGSAVE
		Adjustcount++;
		char savename[50];
		sprintf(savename, "//home//fa//Photo//%d.jpg", Adjustcount);
		cvSaveImage(savename,frame);
		fprintf(stdout, "SavePath is %s.\n", savename);
#endif
		Mat img(frame,true);
		cvtColor(img, img, CV_BGR2GRAY);//change mat to gray
		int pos = position;
		int x = Xindex;
		int y = Yindex;
		char indexbuffer[10];
		int index;
		sprintf(indexbuffer, "%d%d%d", pos, x, y);
		index = atoi(indexbuffer);
		int xyindex;
		sprintf(indexbuffer, "%d%d", x, y);
		xyindex = atoi(indexbuffer);
		fprintf(stdout, "Adjust photo index is %d_%d.\n", pos, xyindex);
		ResultOutStr<<"Adjust photo save in map,the index is : "<<dec<<index<<endl;
//		ResultOutStr<<"Adjust photo save in map,the index is : "<<index<<endl;
		PhotoAdjustMap.insert(std::pair<int, Mat>(index, img));
#ifdef DEBUGPHOTOSAVE
		char savename[50];
		sprintf(savename, "//home//fa//Photo//%d_%d_a.jpg", pos, xyindex);
		cvSaveImage(savename,frame);
		fprintf(stdout, "Adjust photo SavePath is %s.\n", savename);
#endif
		cvReleaseCapture(&capture);
	}
	cvReleaseCapture(&capture);
	return true;
}

bool ParameterAdjust::GetPhoto(unsigned char Xindex, unsigned char Yindex, unsigned char position)
{
	capture = cvCreateCameraCapture(CV_CAP_ANY);
	if(NULL == capture)
		return false;
	fprintf(stdout, "cvSetCaptureProperty value is  %d.\n", cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 1280));
	fprintf(stdout, "cvSetCaptureProperty value is  %d.\n", cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 720));
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 1280);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 720);

//	usleep(1000*1000);
	IplImage* frame;
	frame = cvQueryFrame(capture);
	if(!frame)
	{
		//return failure of get photo to robot
		fprintf(stdout, "Get first photo failed!\n");
		ResultOutStr<<"Get original photo failed!"<<endl;
		cvReleaseCapture(&capture);
		return false;
	}
	else
	{
#ifdef DEBUGPHOTO
		cvShowImage("vedio",frame);
		cvReleaseCapture(&capture);
		return true;
#endif
#ifdef DEBUGSAVE
		cvSaveImage("//home//fa//Photo//0_o.jpg",frame);
#endif
		Mat img(frame,true);
		cvtColor(img, img, CV_BGR2GRAY);//change mat to gray
		int pos = position;
		int x = Xindex;
		int y = Yindex;
		char indexbuffer[10];
		int index;
		sprintf(indexbuffer, "%d%d%d", pos, x, y);
		index = atoi(indexbuffer);
		int xyindex;
		sprintf(indexbuffer, "%d%d", x, y);
		xyindex = atoi(indexbuffer);
		fprintf(stdout, "Original photo index is %d_%d.\n", pos, xyindex);
		ResultOutStr<<"Original photo save in map,the index is : "<<dec<<index<<endl;
//		ResultOutStr<<"Original photo save in map,the index is : "<<index<<endl;
		PhotoMap.insert(std::pair<int, Mat>(index, img));
#ifdef DEBUGPHOTOSAVE
		char savename[50];
		sprintf(savename, "//home//fa//Photo//%d_%d_o.jpg", pos, xyindex);
		cvSaveImage(savename,frame);
		fprintf(stdout, "Original photo SavePath is %s.\n", savename);
#endif
		cvReleaseCapture(&capture);
	}
	cvReleaseCapture(&capture);
	return true;
}

#ifdef USETENCENT
//--adjust code begin by tencent--//
void KeypointsToPoints(const std::vector<KeyPoint> &keyPoints, std::vector<Point2f> &points)
{
	for (int i = 0; i < keyPoints.size(); i++)
	{
		points.push_back(keyPoints[i].pt);
	}
}

float CalLength(Point3f p)
{
	return  sqrtf(p.x *p.x + p.y * p.y + p.z *p.z);
}

inline float sign(float t)
{
	if (t >= 0.0f)
		return 1.0f;
	else
		return -1.0f;
}

std::vector<float> CalBarycentric(const std::vector<Point2f> &points, Point2f tf)
{
	Point3f p1 = Point3f(points[0].x, points[0].y, 0.0f);
	Point3f p2 = Point3f(points[1].x, points[1].y, 0.0f);
	Point3f p3 = Point3f(points[2].x, points[2].y, 0.0f);

	Point3f f(tf.x, tf.y, 0.0f);
	Point3f f1 = p1 - f;
	Point3f f2 = p2 - f;
	Point3f f3 = p3 - f;

	Point3f va = (p1 - p2).cross(p1 - p3);
	Point3f va1 = f2.cross(f3);
	Point3f va2 = f3.cross(f1);
	Point3f va3 = f1.cross(f2);

	float a = CalLength(va);
	float a1 = CalLength(va1) / a * sign(va.dot(va1));
	float a2 = CalLength(va2) / a * sign(va.dot(va2));
	float a3 = CalLength(va3) / a * sign(va.dot(va3));

	std::vector<float> w;
	w.push_back(a1);
	w.push_back(a2);
	w.push_back(a3);
	return w;
}

inline float Clamp(float v, float min, float max)
{
	v = v > min ? v : min;
	v = v < max ? v : max;
	return v;
}

void tCalRotationAndTranslation(const std::vector<KeyPoint> &object0, const std::vector<KeyPoint> &object1, const std::vector< DMatch > &matches,
				  float &rotationAngle, cv::Point2f &translation)
{
	std::vector<DMatch> copyMatches(matches);

	std::cout << copyMatches[0].distance << "  " << copyMatches[1].distance << std::endl;
	std::vector<float> thetaValue;
	for (int i = 0; i < 3; i++)
	{
		if (copyMatches[i + 1].distance > maxError)
			break;
		cv::Point2f d0 = (object0[copyMatches[i].queryIdx].pt - object0[copyMatches[i+1].queryIdx].pt);
		cv::Point2f d1 = (object1[copyMatches[i].trainIdx].pt - object1[copyMatches[i+1].trainIdx].pt);
		double len = sqrtf(d0.x * d0.x + d0.y * d0.y);
		d0.x /= len;
		d0.y /= len;

		double len1 = sqrtf(d1.x * d1.x + d1.y * d1.y);
		d1.x /= len1;
		d1.y /= len1;
		//std::cout << len << "  " << len1 << std::endl;

		double cosTheta = d0.dot(d1);
		double sinTheta = d0.cross(d1);

		float tTheta = acos(Clamp(cosTheta,0.0f,1.0f)) / 3.1415926 * 180.0;
		if (sinTheta < 0.0f)
			tTheta = -tTheta;
		thetaValue.push_back(tTheta);

	}

	float sumTheta = 0.0f;
	for (int i = 0; i < thetaValue.size(); i++)
	{
		sumTheta += thetaValue[i];
	}

	rotationAngle = sumTheta / thetaValue.size();

	cv::Point2f centerPos(320, 180);

	int count = 0;
	translation.x = 0.0f;
	translation.y = 0.0f;
	for (int i = 0; i < 3; i++)
	{
		if (copyMatches[i].distance > maxError)
			continue;

		cv::Point2f dd0 = object0[copyMatches[i].queryIdx].pt - centerPos;

		float angleInRadian = rotationAngle / 180.0f * 3.1415926f;
		cv::Point2f nDD0;
		nDD0.x = dd0.x * cosf(angleInRadian) - dd0.y * sinf(angleInRadian);
		nDD0.y = dd0.x * sinf(angleInRadian) + dd0.y * cosf(angleInRadian);

		cv::Point2f newDD0 = centerPos + nDD0;
		cv::Point2f tv = newDD0 - object1[copyMatches[i].trainIdx].pt;
		//std::cout << "Sub_translation:  " << tv << std::endl;
		translation += tv;
		count++;
	}
	translation.x /= count;
	translation.y /= count;

	std::cout << "translation:  " << translation << std::endl;

//	return translation;
}


void CalMatchesThroughOpticalFlk(const Mat &img1, const Mat &img2 ,
						         const std::vector<KeyPoint> &left_keypoints, const std::vector<KeyPoint> &right_keypoints,
								 std::vector<DMatch> &matches)
{
	std::vector<Point2f> left_points;
	KeypointsToPoints(left_keypoints, left_points);

	std::vector<Point2f> right_points(left_points.size());

	std::vector<uchar> vstatus;
	std::vector<float> verror;

	cv::calcOpticalFlowPyrLK(img1, img2, left_points, right_points, vstatus, verror);

	std::vector<Point2f> right_points_to_find;
	std::vector<int> right_points_to_find_back_index;

	for (unsigned int i = 0; i < vstatus.size(); i++)
	{
		if (vstatus[i] && verror[i] < 8.0)
		{
			right_points_to_find_back_index.push_back(i);
			right_points_to_find.push_back(right_points[i]);
		}
		else{
			vstatus[i] = 0;
		}
	}

	Mat right_points_to_find_flat = Mat(right_points_to_find).reshape(1, right_points_to_find.size());

	std::vector<Point2f> right_features;
	KeypointsToPoints(right_keypoints, right_features);

	Mat right_features_flat = Mat(right_features).reshape(1, right_features.size());

	BFMatcher matcher(CV_L2);
	std::vector<std::vector<DMatch> >nearest_neighbors;
	matcher.radiusMatch(right_points_to_find_flat, right_features_flat, nearest_neighbors, 2.0f);

	std::set<int> found_in_right_points;
	for (int i = 0; i < nearest_neighbors.size(); i++)
	{
		DMatch _m;
		if (nearest_neighbors[i].size() == 1)
		{
			_m = nearest_neighbors[i][0];
		}
		else if (nearest_neighbors[i].size() > 1)
		{
			double ratio = nearest_neighbors[i][0].distance / nearest_neighbors[i][1].distance;

			if (ratio < 0.3){
				_m = nearest_neighbors[i][0];
			}
			else{
				continue;
			}
		}
		else
		{
			continue;
		}

		if (found_in_right_points.find(_m.trainIdx) == found_in_right_points.end())
		{
			_m.queryIdx = right_points_to_find_back_index[_m.queryIdx];
			matches.push_back(_m);
			found_in_right_points.insert(_m.trainIdx);
		}
	}
}


void CalMatchesThroughMatcher(const Mat &img1, const Mat &img2,
	std::vector<KeyPoint> &left_keypoints, std::vector<KeyPoint> &right_keypoints,
	std::vector<DMatch> &matches)
{
	SurfDescriptorExtractor surfDesc;
	Mat descriptros1, descriptros2;
	surfDesc.compute(img1, left_keypoints, descriptros1);
	surfDesc.compute(img2, right_keypoints, descriptros2);

	BruteForceMatcher<L2<float> > testMatcher;

	BFMatcher newMatcher(NORM_L2, false);
	/*
	testMatcher.match(descriptros1, descriptros2, matches);
	if(matches.size()>9)
	{
		std::nth_element(matches.begin(), matches.begin() + 8, matches.end());
		matches.erase(matches.begin() + 9, matches.end());
	}
	*/
	std::vector<vector<DMatch> > knnMatches;
	const int k = 2;
	const float minRatio = 1.f / 2.5f;
	newMatcher.knnMatch(descriptros1, descriptros2, knnMatches, k);
	for (size_t i = 0; i < knnMatches.size(); i++) {
		const DMatch& bestMatch = knnMatches[i][0];
		const DMatch& betterMatch = knnMatches[i][1];

		float  distanceRatio = bestMatch.distance / betterMatch.distance;
		if (distanceRatio < minRatio)
			matches.push_back(bestMatch);
	}
}

void GetBestMatches(const std::vector<DMatch> &matches1,const std::vector<DMatch> &matches2 ,std::vector<DMatch> &bestMatch)
{
	for (int i = 0; i < matches1.size(); i++)
	{
		for (int j = 0; j < matches2.size(); j++)
		{
			if (matches1[i].trainIdx == matches2[j].trainIdx && matches1[i].queryIdx == matches2[j].queryIdx)
			{
				bestMatch.push_back(matches1[i]);
				break;
			}
		}
	}
}
//--adjust code end--//
#endif

void ParameterAdjust::RemoveFirstPhoto()
{
	std::map<int, Mat>::iterator iter;
	iter = PhotoMap.find(0);
	if(iter != PhotoMap.end())
	{
		PhotoMap.erase(iter);
	}
	return;
}

int ParameterAdjust::AdjustProcess(const AdjustData* msg)
{
	struct timeval tpstart,tpend;
	float timeuse;
	unsigned char Replybuffer[16];
	bool Sendtag = true;
	memset(Replybuffer, 0x00, 16);
	Replybuffer[0] = 0xF5;
	Replybuffer[1] = 0xF5;
	gettimeofday(&tpstart, NULL);

	Mat OrignalImg;
	Mat AdjustImg;

	float rotation = (int)msg->data[3];
	int pos = msg->data[2];
	int x = msg->data[4];
	int y = msg->data[5];
	char indexbuffer[10];
	int index;
	sprintf(indexbuffer, "%d%d%d", pos, x, y);
	index = atoi(indexbuffer);
	ResultOutStr<<"Receive GetPhot Data : [ ";
	for(int i = 0; i < 10; i++)
	{
		ResultOutStr<<hex<<(int)(msg->data[i])<<" ";
	}
	ResultOutStr<<"]"<<endl;

	ResultOutStr<<"adjust orignal picture is "<<dec<<index<<"_o.jpg fix picture is "<<dec<<index<<"_a.jpg, result is :"<<endl;
	std::map<int, Mat>::iterator iter;
	iter = PhotoMap.find(index);
	if(iter != PhotoMap.end())
	{
		OrignalImg = iter->second;
	}
	else
	{
		//error : can not find orginal mat
		fprintf(stdout, "error : can not find orginal mat of index : %d .\n", index);
		ResultOutStr<<"error : can not find orginal mat"<<endl;
		gettimeofday(&tpend, NULL);
		timeuse = 1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
		timeuse /= 1000000;
		fprintf(stdout, "Adjust used time is :%f seconds\n", timeuse);
		m_CommHelperPtr->SendData((char*)Replybuffer, 16);
		return 0;
	}

	iter = PhotoAdjustMap.find(index);
	if(iter != PhotoAdjustMap.end())
	{
		AdjustImg = iter->second;
		PhotoAdjustMap.erase(iter);
	}
	else
	{
		//error : can not find adjust mat
		fprintf(stdout, "error : can not find adjust mat of index : %d .\n", index);
		ResultOutStr<<"error : can not find adjust mat"<<endl;
		gettimeofday(&tpend, NULL);
		timeuse = 1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
		timeuse /= 1000000;
		fprintf(stdout, "Adjust used time is :%f seconds\n", timeuse);
		m_CommHelperPtr->SendData((char*)Replybuffer, 16);
		return 0;
	}

	ResultOutStr<<"Real rotation is : "<<dec<<rotation/10<<endl;
	//Adjust by scut
#ifdef USESCUT
	Mat img1_orin = OrignalImg;//imread(".\\topS\\Segment Fault1\\0.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	Mat img2_orin = AdjustImg;//imread(".\\topS\\Segment Fault1\\1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	EstRXY e(img1_orin.cols, img1_orin.rows);
	double r;
	Point2f result;
	float angle ,dis;
	fprintf(stdout, "run into scut adjust code \n!");
	if (e.estRXYVector(img1_orin, img2_orin, r, result))
	{
		fprintf(stdout, "\nrotation:%lf\n", r);
		fprintf(stdout, "(x,y): (%f, %f)\n", result.x, result.y);
		ResultOutStr<<"rotation:"<<(float)r<<" (x,y):"<<result.x<<" , "<<result.y<<endl;
	}
	else
	{
		perror("\nFalse detection.\n");
		Sendtag = false;
		ResultOutStr<<"False detection."<<endl;
	}
	fprintf(stdout, "run out scut adjust code \n!");
	//Adjust end
#endif

#ifdef USETENCENT
//--Adjust by Tencent--//
	Mat img1 = OrignalImg;
	Mat img2 = AdjustImg;
	cv::Point2f result;
	float angle ,dis;
	resize(OrignalImg, img1, Size(640, 360), 0, 0, CV_INTER_LINEAR);
	resize(AdjustImg, img2, Size(640, 360), 0, 0, CV_INTER_LINEAR);

	fprintf(stdout, "begin to get adjust parameter\n");
	int minHessian = 100;
	SurfFeatureDetector detector(minHessian);

	std::vector<KeyPoint> left_keypoints, right_keypoints;
	std::vector<DMatch> matches;
	detector.detect(img1, left_keypoints);
	detector.detect(img2, right_keypoints);

	std::vector<DMatch> matches1,matches2;

	CalMatchesThroughMatcher(img1, img2, left_keypoints, right_keypoints, matches);

	std::vector<Point2f> imgpts1, imgpts2;
	for (unsigned int i = 0; i < matches.size(); i++)
	{
		imgpts1.push_back(left_keypoints[matches[i].queryIdx].pt);
		imgpts2.push_back(right_keypoints[matches[i].trainIdx].pt);
	}

	std::vector<DMatch> bestMatches;

	if (matches.size() > 7)
	{
		std::vector<uchar> status;
		Mat F = findFundamentalMat(imgpts1, imgpts2, FM_RANSAC, 0.01f, 0.9f, status);

		std::cout << F << std::endl;

		for (int i = 0; i < matches.size(); i++)
		{
			if (status[i])
			{
				bestMatches.push_back(matches[i]);
			}
		}

	}
	else
	{
		bestMatches = matches;
	}

	if (bestMatches.size() == 0)
	{
		std::cout << "Can't get a goodResult!" << std::endl;
		ResultOutStr<<"Can't get a goodResult!"<<endl;
		Sendtag = false;
	}
	else
	{
		std::sort(bestMatches.begin(), bestMatches.end());
		if (std::max(bestMatches[0].distance , bestMatches[1].distance) >= maxError)
		{
			fprintf(stdout, "Can't get a goodResult!");
			ResultOutStr<<"Can't get a goodResult!"<<endl;
			Sendtag = false;
		}
		else
		{
			fprintf(stdout, "Get a pretty good result!");
			ResultOutStr<<"Get a pretty good result!"<<endl;
			tCalRotationAndTranslation(left_keypoints, right_keypoints, bestMatches, angle, result);
			fprintf(stdout, "rotation angle is %f\n", angle);
			ResultOutStr<<"rotation angle is "<<angle<<endl;
		}
	}

	fprintf(stdout, "end to get adjust parameter\n");
//--end of tencent--//
#endif

	//--send adjust data--//
	if(Sendtag)
	{
		int sendangle;
		int sendx;
		int sendy;
		unsigned char* AdjustSendBuffer = new unsigned char[16];
		memset(AdjustSendBuffer, 0x00, 16);
		AdjustSendBuffer[0] = 0xFE;
		AdjustSendBuffer[1] = 0xFE;

		if(angle<0);
			angle += 360;

		sendangle = (int)(angle*10);
		AdjustSendBuffer[2] = sendangle/256;
		AdjustSendBuffer[3] = sendangle%256;

		if(result.x<0)
		{
			AdjustSendBuffer[5] = 0x01;
			sendx = result.x*(-10);
		}
		else
		{
			sendx = result.x*10;
		}
		AdjustSendBuffer[6] = sendx/256;
		AdjustSendBuffer[7] = sendx%256;

		if(result.y<0)
		{
			AdjustSendBuffer[8] = 0x01;
			sendy = result.y*(-10);
		}
		else
		{
			sendy = result.y*10;
		}
		AdjustSendBuffer[9] = sendy/256;
		AdjustSendBuffer[10] = sendy%256;

		unsigned char CRC = AdjustSendBuffer[0];
		for(int i = 1; i < 15; i++)
		{
			CRC = CRC ^ AdjustSendBuffer[i];
		}
		AdjustSendBuffer[15] = CRC;
		m_CommHelperPtr->SendData((char*)AdjustSendBuffer, 16);
	}
	else
	{
//Can not adjust, Send bad result by serial
		m_CommHelperPtr->SendData((char*)Replybuffer, 16);
	}
//	m_MainProgram->m_CommHelper.SendData((char*)AdjustSendBuffer, 16);

	//--end send data--//
	gettimeofday(&tpend, NULL);
	timeuse = 1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
	timeuse /= 1000000;
	fprintf(stdout, "Adjust used time is :%f seconds\n", timeuse);
	ResultOutStr<<"Adjust used time is : "<<timeuse<<" seconds ,END of Adjust"<<endl<<endl;
	return 1;
}

void *ParameterAdjust::ParameterAdjustThreadFunc(void * lparam)
{
	ParameterAdjust *pAdjust;
	//得到ParameterAdjust实例指针
	pAdjust = (ParameterAdjust*) lparam;

	while (true)
	{

		if (pAdjust->RevAdjustDataQueue.size() <= 0)
		{
			pthread_mutex_lock(&pAdjust->AdjustThreadmutex);
			pthread_cond_wait(&pAdjust->AdjustThreadEvent, &pAdjust->AdjustThreadmutex);
			pthread_mutex_unlock(&pAdjust->AdjustThreadmutex);

		}
		else
		{
			while (!pAdjust->RevAdjustDataQueue.empty())
			{
				pthread_mutex_lock(&pAdjust->RevAdjustOrderQueuemutex);
				pAdjust->AdjustProcess(&pAdjust->RevAdjustDataQueue.top());
				if (pAdjust->RevAdjustDataQueue.top().data != NULL)
				{
					delete (pAdjust->RevAdjustDataQueue.top().data);
				}
				pAdjust->RevAdjustDataQueue.pop();

				pthread_mutex_unlock(&pAdjust->RevAdjustOrderQueuemutex);
			}
		}

	}
	pthread_exit(NULL);
	return 0;
}

bool ParameterAdjust::CreateAdjustThread()
{
	int ret = pthread_create(&m_AdjustThreadHandle, NULL, ParameterAdjustThreadFunc, (void*) this);
	if (ret)
	{
		fprintf(stdout, "Create ParameterAdjustThreadFunc thread error!\n");
		return false;
	}
	fprintf(stdout, "Create ParameterAdjustThreadFunc thread successfully!\n");
	return true;
}
