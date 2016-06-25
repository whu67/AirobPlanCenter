#pragma once
#ifndef ESTRXY_H
#define ESTRXY_H

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <stdlib.h>
#include <string.h>
#define NormalizeInputImages

using namespace std;
using namespace cv;

struct ImageSet {
	std::vector<std::string> dir;//frame directory
	std::vector<cv::Mat> fc;//frame contect
};

class EstRXY {
public:

	/*default use a boundary metric in validation with
	ransacT=4
	boundary value for rotation  20бу
	*/
	EstRXY(int img_cols, int img_rows);

	/*ransacThresh: allow more points used in estimation with a bigger value, usually range from 1 - 9
	r_bound: absolute boundary for the rotation estimation
	*/
	EstRXY(int img_cols, int img_rows, double ransacThresh,
		double r_bound = (20.0));

	/*Must be called after estRXY function*/
	double getRotationEst();

	/*Must be called after calXYVectorWithTheSamePlain function*/
	cv::Point2f getXYPixelEstWithinSamePlain();

	/*Estimate the rotaion, x, y difference between two images.
	Return true with output in parameters if the estimation assume to be reasonable.
	Otherwise, return false with no parameters being set.
	src_in: image after movement, should be in gray scale
	dst_in: image before movement, should be in gray scale
	e_rotation_out: rotation estimation
	e_x_out: x estimation
	e_y_out: y estimation*/
	//bool estRXY(cv::Mat& src_in, cv::Mat& dst_in,
	//	double& e_rotation_out/*, double& e_x_out, double& e_y_out*/);

	/*Estimate the rotaion, vector (x,y) between two images(should be in the same plain).
	Return true with output in parameters if the estimation assume to be reasonable.
	Otherwise, return false with no parameters being set.
	src_in: image after movement, should be in gray scale
	dst_in: image before movement, should be in gray scale
	e_rotation_out: rotation estimation
	xy_vector: (x,y) pixel vector estimation
	Return values:
    -1: failed to estimate
	0: the result might not be reliable
	1: the result is less reliable
	2: the result is reliable*/
	int estRXYVector(cv::Mat& src_in, cv::Mat& dst_in,
		double& e_rotation_out, Point2f& xy_vector_out);

	/*Estimate the rotaion, vector (x,y) between two images(should be in the same plain).
	Return true with output in parameters if the estimation assume to be reasonable.
	Otherwise, return false with no parameters being set.
	src_in: image after movement, should be in gray scale
	dst_in: image before movement, should be in gray scale
	e_rotation_out: rotation estimation
	xy_vector: (x,y) pixel vector estimation
	Return values:
	-1: failed to estimate
	0: not a good result
	1: good result*/
	int estRXYVector2(cv::Mat& src_in, cv::Mat& dst_in,
		double& e_rotation_out, Point2f& xy_vector_out);


private:
	int RegionMaximum;//define the maximum region matched point count
	int MinimunKeyPointsRequired;//define the minimum key point number required
	int MinimunMatchedPointsRequired;//define the minimum matched point number required
	double r;//rotation difference estimation from src to dst
	cv::Point2f center;//the center coordinate of the images
	cv::Point2f xy_vector_within_the_same_plain;//(x,y) vector of pixel difference between two input images
	double rbound;//rotation boundary value to validate the final estimation
	double ransacT;//ransac threshold
	float maxError;//distance error
	/*convert img1 and img2 to gray images and save the data back in original matrix,
	if img1 and img2 are  BGR images. And perform equalHist on img1 and img2 if they are gray image.*/
	void normImages(cv::Mat& img1, cv::Mat& img2);

	/*use SURF descriptor to find loosely matched points*/
	int useSurfDescriptors(cv::Mat& src_in, cv::Mat& dst_in,
		std::vector<Point2f>& ckp1_out, std::vector<Point2f>& ckp2_out);

	/*filt out the ummatched points*/
	void  filtOutInvalidPoints(std::vector<Point2f>& p1in, std::vector<Point2f>& p2in, 
		std::vector<Point2f>& p1out, std::vector<Point2f>& p2out, 
		std::vector<unsigned char>& match_mask);
	/*filt out the ummatched points ver2*/
	/*void  filtOutInvalidPointsV2(std::vector<KeyPoint>& k1in, std::vector<KeyPoint>& k2in,
		std::vector<Point2f>& k1out, std::vector<Point2f>& k2out, std::vector<DMatch>& fm);*/

	/*return true if the transformMatrix is a 2*3 matrix and calculate the estimations*/
	bool calEstRXY(cv::Mat& transformMatrix);

	/*return true if and only if the rotation estimation is within the boundary*/
	bool validateRotationEstimation();

	/*return true if the sizes of cf1 and cf2 are equal and both greater than 0, 
	along with the calculation of xy_vector.
	Assuming all the key points of two images are in the same plain or parallel plains.
	Return value:
	-1: means the failure in calculating xy_vector_within_the_same_plain, no result return
	0: means the xy_vector_within_the_same_plain bearly moved, but the result might be not reliable
	1: means the xy_vector_within_the_same_plain is less reliable
	2: means the xy_vector_within_the_same_plain is reliable
	*/
	int calXYVectorWithTheSamePlain(vector<Point2f>&cf1, vector<Point2f>&cf2);

	/*codes from tencent guy*/
	void tCalRotationAndTranslation(const std::vector<KeyPoint> &object0, const std::vector<KeyPoint> &object1, const std::vector< DMatch > &matches,
		double &rotationAngle, Point2f& xy_v);
};

#endif // !ESTRXY_H
