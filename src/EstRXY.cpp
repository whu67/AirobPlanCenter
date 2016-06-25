#include "EstRXY.h"

EstRXY::EstRXY(int img_cols, int img_rows) {
//	this->RegionMaximum = 3;
//	this->MinimunKeyPointsRequired = 6;
//	this->MinimunMatchedPointsRequired = 3;
//	this->ransacT = 4;
//	this->rbound = 20;
//	this->center.x = (float)img_cols / 2;
//	this->center.y = (float)img_rows / 2;
//	this->maxError = 0.06;
	this->RegionMaximum = 5;
	this->MinimunKeyPointsRequired = 300;
	this->MinimunMatchedPointsRequired = 100;
	this->ransacT = 4;
	this->rbound = 20;
	this->center.x = (float)img_cols / 2;
	this->center.y = (float)img_rows / 2;
	this->maxError = 0.06;
}

EstRXY::EstRXY(int img_cols, int img_rows, double ransacThresh, double r_bound/*, double x_bound, double y_bound*/) {
//	this->RegionMaximum = 3;
//	this->MinimunKeyPointsRequired = 6;
//	this->MinimunMatchedPointsRequired = 3;
//	this->ransacT = ransacThresh;
//	this->rbound = r_bound;
//	this->center.x = (float)img_cols / 2;
//	this->center.y = (float)img_rows / 2;
//	this->maxError = 0.06;
	this->RegionMaximum = 5;
	this->MinimunKeyPointsRequired = 300;
	this->MinimunMatchedPointsRequired = 100;
	this->ransacT = ransacThresh;
	this->rbound = r_bound;
	this->center.x = (float)img_cols / 2;
	this->center.y = (float)img_rows / 2;
	this->maxError = 0.06;
}

double EstRXY::getRotationEst() {
	return this->r;
}

cv::Point2f EstRXY::getXYPixelEstWithinSamePlain() {
	return this->xy_vector_within_the_same_plain;
}

//double EstRXY::getXEst() {
//	return this->x;
//}
//
//double EstRXY::getYEst() {
//	return this->y;
//}

void EstRXY::normImages(Mat& img1, Mat& img2) {
	if (img1.channels() == 3) {
		cvtColor(img1, img1, CV_BGR2GRAY);
	}
	if (img2.channels() == 3) {
		cvtColor(img2, img2, CV_BGR2GRAY);
	}
	if (img1.channels() == 1) {
		equalizeHist(img1, img1);
	}
	if (img2.channels() == 1) {
		equalizeHist(img2, img2);
	}
}

//bool EstRXY::estRXY(Mat& src_in, Mat& dst_in,
//	double& e_rotation_out/*, double& e_x_out, double& e_y_out*/) {
//	vector<Point2f> cf1, cf2;
//	Mat am;
//#ifdef ResizeToHalf
//	resize(src_in, src_in, Size(src_in.cols/2, src_in.rows/2), 0, 0, CV_INTER_LINEAR);
//	resize(dst_in, dst_in, Size(dst_in.cols/2, dst_in.rows/2), 0, 0, CV_INTER_LINEAR);
//#endif // ResizeToHalf
//	if (!useSurfDescriptors(src_in, dst_in, cf1, cf2)) {
//		return false;
//	}
//	am = estimateRigidTransform(cf1, cf2, false);
////#ifdef showProcess
////	printf("The estimated RigidTransform matrix is:");
////	cout << endl << am << endl << endl;
////#endif // showProcess
//	if (calEstRXY(am)) {
//		if (validateRotationEstimation()) {
//			e_rotation_out = this->r;
//			/*e_x_out = this->x;
//			e_y_out = this->y;*/
//			return true;
//		}
//	}
//	return false;
//}

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
int EstRXY::estRXYVector(cv::Mat& src_in, cv::Mat& dst_in,
	double& e_rotation_out, Point2f& xy_vector_out) {
	vector<Point2f> cf1, cf2;
	Mat am;
	//normImages(src_in, dst_in);
	int flag = useSurfDescriptors(src_in, dst_in, cf1, cf2);
	if (flag<0) {
		return -1;//false detection
	}
	am = estimateRigidTransform(cf1, cf2, false);

	if (calEstRXY(am)) {
		validateRotationEstimation();
		e_rotation_out = this->r;
		xy_vector_out = this->xy_vector_within_the_same_plain;
		return flag;
	}
	return -1;
}

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
int EstRXY::estRXYVector2(cv::Mat& src_in, cv::Mat& dst_in,
	double& e_rotation_out, Point2f& xy_vector_out) {
	vector<Point2f> cf1, cf2;
	Mat am;
	//normImages(src_in, dst_in);
	SurfFeatureDetector det(60);
	vector<KeyPoint> kps1, kps2;
	Mat desc1, desc2;
	det.detect(src_in, kps1);
	det.detect(dst_in, kps2);
	if (kps1.size() < this->MinimunKeyPointsRequired || kps2.size() < this->MinimunKeyPointsRequired) {
		printf("\n******************\nError: The input images are not meeting the minimun requirement of key point numbers.\n");
		return -1;
	}
	SurfDescriptorExtractor ext;
	ext.compute(src_in, kps1, desc1);
	ext.compute(dst_in, kps2, desc2);
	//FlannBasedMatcher matcher;
	BFMatcher matcher(NORM_L2, true);
	vector<DMatch> matches;
	matcher.match(desc1, desc2, matches);
	if (matches.size() > 9) {
		std::nth_element(matches.begin(), matches.begin() + 8, matches.end());//find 9 matches with smallest distances
		matches.erase(matches.begin() + 9, matches.end());//erase the less larger distance
	}
	vector<Point2f> cp1, cp2;
	for (int i = 0; i < matches.size(); i++) {
		cp1.push_back(kps1[matches[i].queryIdx].pt);
		cp2.push_back(kps2[matches[i].trainIdx].pt);
	}
	/*vector<unsigned char> match_mask;
	findHomography(cp1, cp2, CV_RANSAC, this->ransacT, match_mask);
	filtOutInvalidPoints(cp1, cp2, p1_out, p2_out, match_mask);*/
	vector<DMatch> bestMatches;
	if (matches.size() > 7){
		std::vector<uchar> status;
		Mat F = findFundamentalMat(cp1, cp2, FM_RANSAC, 0.01f, 0.9f, status);
		//std::cout << F << std::endl;
		for (int i = 0; i < matches.size(); i++){
			if (status[i]){
				bestMatches.push_back(matches[i]);
			}
		}
	}
	else {
		bestMatches = matches;
	}
	std::sort(bestMatches.begin(), bestMatches.end());
	tCalRotationAndTranslation(kps1, kps2, bestMatches, e_rotation_out, xy_vector_out);
	if (max(bestMatches[0].distance, bestMatches[1].distance) >= (0.06f)){
		std::cout << "Can't get a goodResult!" << std::endl;
		return 0;
	}else {
		std::cout << "Get a pretty good result!" << std::endl;
		return 1;
	}
}

int EstRXY::useSurfDescriptors(Mat& src_in, Mat& dst_in,
	vector<Point2f>& p1_out, vector<Point2f>& p2_out) {
	//int minHessian = 400;
	SurfFeatureDetector det(400);
	vector<KeyPoint> kps1, kps2;
	Mat desc1, desc2;
	det.detect(src_in, kps1);
	det.detect(dst_in, kps2);
	if (kps1.size() < this->MinimunKeyPointsRequired || kps2.size() < this->MinimunKeyPointsRequired) {
		printf("\n******************\nError: The input images are not meeting the minimun requirement of key point numbers.\n");
		return -1;
	}
	SurfDescriptorExtractor ext;
	ext.compute(src_in, kps1, desc1);
	ext.compute(dst_in, kps2, desc2);

	//FlannBasedMatcher matcher;
	BFMatcher matcher(NORM_L2, true);
	vector<DMatch> matches;
	matcher.match(desc1, desc2, matches);
	int sizem = matches.size();
	vector<Point2f> cp1, cp2;
	for (int i = 0;i < sizem;i++) {
		cp1.push_back(kps1[matches[i].queryIdx].pt);
		cp2.push_back(kps2[matches[i].trainIdx].pt);
	}
	vector<unsigned char> match_mask;
	findHomography(cp1, cp2, CV_RANSAC, this->ransacT, match_mask);
	filtOutInvalidPoints(cp1, cp2, p1_out, p2_out, match_mask);

	/*vector<DMatch> fm;
	for (int i = 0; i < match_mask.size(); i++) {
		if (match_mask[i] > 0) {
			fm.push_back(matches[i]);
		}
	}
	filtOutInvalidPointsV2(kps1, kps2, p1_out, p2_out, fm);
	calXYVectorWithTheSamePlainV2(kps1, kps2, fm);
	tm.stop();
	std::printf("Time comsuming: %lf\n", tm.getTimeSec());*/
//#ifdef showProcess
//	vector<DMatch> fm;
//	for (int i = 0; i < match_mask.size(); i++) {
//		if (match_mask[i] > 0) {
//			fm.push_back(matches[i]);
//		}
//	}
//	Mat imatches;
//	drawMatches(src_in, kps1, dst_in, kps2, fm, imatches, Scalar::all(-1), Scalar::all(-1),
//		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//	imshow("Final Matches for the input images", imatches);
//	waitKey();
//#endif // showProcess
	return calXYVectorWithTheSamePlain(p1_out, p2_out);
}

void EstRXY::filtOutInvalidPoints(vector<Point2f>& k1in, vector<Point2f>& k2in, vector<Point2f>& k1out, vector<Point2f>& k2out, vector<unsigned char>& match_mask) {
	int size = match_mask.size();
	k1out.clear();
	k2out.clear();
	for (int i = 0;i < size;i++) {
		if (match_mask[i] > 0) {
			k1out.push_back(k1in[i]);
			k2out.push_back(k2in[i]);
		}
	}
}

//void  EstRXY::filtOutInvalidPointsV2(std::vector<KeyPoint>& k1in, std::vector<KeyPoint>& k2in,
//	std::vector<Point2f>& k1out, std::vector<Point2f>& k2out, std::vector<DMatch>& fm) {
//	int size = fm.size();
//	k1out.clear();
//	k2out.clear();
//	for (int i = 0; i < size; i++) {
//		k1out.push_back(k1in[fm[i].queryIdx].pt);
//		k2out.push_back(k2in[fm[i].trainIdx].pt);
//	}
//}

bool EstRXY::calEstRXY(cv::Mat& transformMatrix) {
//	if (transformMatrix.cols == 3 && transformMatrix.rows == 2) {
//		double xcos = transformMatrix.at<double>(0, 0);
//		double xsin = transformMatrix.at<double>(0, 1);
//		/*double ax = transformMatrix.at<double>(0, 2);
//		double ay = transformMatrix.at<double>(1, 2);*/
//		double sqr = sqrt(xcos*xcos + xsin*xsin);
//		double sinv = xsin / sqr;
//		double cosv = xcos / sqr;
//
//		this->r = 22430.0 - 500.1*sinv - 22430.0*cosv - 11300.0*sinv*sinv + 600.0*sinv*cosv;//filter out 6 points
//																							   //e.rotation = 27780.0 - 518.5*sinv - 27780.0*cosv - 14000.0*sinv*sinv + 617.9*sinv*cosv;//filter out 7 points, poly21
//		this->r = this->r/2+0.1822;// bias adjustment
//		return true;
//	}
//	return false;
	if (transformMatrix.cols == 3 && transformMatrix.rows == 2) {
		double xcos = transformMatrix.at<double>(0, 0);
		double xsin = transformMatrix.at<double>(0, 1);

		double sqr = sqrt(xcos*xcos + xsin*xsin);
		double sinv = xsin / sqr;
		double cosv = xcos / sqr;

		this->r = 11215.0 - 250.05*sinv - 11215.0*cosv - 5650.0*sinv*sinv + 300.0*sinv*cosv;
		return true;
	}
	return false;
}

bool EstRXY::validateRotationEstimation() {
	if (abs(this->r) < this->rbound) {
		return true;
	}
	std::printf("\nWARNINGRotation estimation exceeds the boundary.\n");
	return false;
}

int EstRXY::calXYVectorWithTheSamePlain(vector<Point2f>&cf1, vector<Point2f>&cf2) {
	int size = cf1.size();
	if (size == cf2.size() && size > this->MinimunMatchedPointsRequired) {
		/*Scalar tm = sum(cf2) - sum(cf1);
		this->xy_vector_within_the_same_plain.x = (float)(tm[0]) / (float)size;
		this->xy_vector_within_the_same_plain.y = (float)(tm[1]) / (float)size;*/

		float lu, ru, ll, rl, dis;
		lu = ru = ll = rl = 0;
		vector<float> eva_distribution;//evaluate the distribution of matched points
		eva_distribution.resize(4);
		eva_distribution[0] = 0;
		eva_distribution[1] = 0;
		eva_distribution[2] = 0;
		eva_distribution[3] = 0;
		Point2f t;//c1 c2 c3 c4 represent the matched points with longest distance to the center inside each region
		vector<Point2f> c, cp;
		c.resize(4);
		cp.resize(4);
		for (int i = 0; i < size; i++) {
			t = cf1[i] - this->center;
			if (t.x == 0 && t.y == 0) {//found the corespongding center point in img2
				this->xy_vector_within_the_same_plain = this->center - cf2[i];
				return 0;
			}else if(t.x<=0){
				dis = t.dot(t);
				if (t.y <= 0) {
					/*upper region on the left, region 1*/
					if (dis > lu) {
						lu = dis;
						c[0]= cf1[i];
						cp[0] = cf2[i];
						eva_distribution[0] += 1;
					}
				}
				else {
					/*upper region on the right, region 2*/
					if (dis > ru) {
						ru = dis;
						c[1] = cf1[i];
						cp[1] = cf2[i];
						eva_distribution[1] += 1;
					}
				}
			}else {
				if (t.y <= 0) {
					/*lower region on the left, region 3*/
					if (dis > ll) {
						ll = dis;
						c[2] = cf1[i];
						cp[2] = cf2[i];
						eva_distribution[2] += 1;
					}
				}
				else {
					/*lower region on the right, region 4*/
					if (dis > rl) {
						rl = dis;
						c[3] = cf1[i];
						cp[3] = cf2[i];
						eva_distribution[3] += 1;
					}
				}
			}
		}
		vector<Point2f> v1, v2;
		for (int j = 0; j < 4; j++) {//normalize the distribution
			if (eva_distribution[j] > this->RegionMaximum){
				eva_distribution[j] = this->RegionMaximum;
				v1.push_back(c[j]);
				v2.push_back(cp[j]);
			}else if(eva_distribution[j]>0){
				v1.push_back(c[j]);
				v2.push_back(cp[j]);
			}
		}
		int psize = v1.size();
		vector<Point2f> xyr;
		float xr,yr,t12, t13, xm2, ym2, xm3, ym3, xv21, yv21;
		if (psize > 2) {
			for (int s1 = 0; s1 < psize; s1++) {
				for (int s2 = s1+1; s2 < psize; s2++) {
					for (int s3 = s2 + 1; s3 < psize; s3++) {
						xm2 = v2[s2].x - v2[s1].x;
						xm3 = v2[s3].x - v2[s1].x;
						ym2 = v2[s2].y - v2[s1].y;
						ym3 = v2[s3].y - v2[s1].y;
						xv21 = v2[s1].x;
						yv21 = v2[s1].y;
						t12 = (this->center - v1[s1]).dot(v1[s2] - v1[s1]);
						t13 = (this->center - v1[s1]).dot(v1[s3] - v1[s1]);
						if (ym2 == 0 && ym3 != 0 && xm2!=0) {
							xr = t12 / xm2 + xv21;
							yr = (t13 - xm3*(xr - xv21)) / ym3 + yv21;
							xyr.push_back(Point2f(xr, yr));
						}
						else if (ym3 == 0 && ym2 != 0 && xm3 != 0) {
							xr = t13 / xm3 + xv21;
							yr = (t12 - xm2*(xr - xv21)) / ym2 + yv21;
							xyr.push_back(Point2f(xr, yr));
						}
						else if (ym2 != 0 && (ym2*xm3-ym3*xm2)!=0) {
							xr = ym2*(t13 + xm3*xv21 + ym3*yv21) - ym3*(t12 + xm2*xv21 + ym2*yv21);
							xr = xr / (ym2*xm3 - ym3*xm2);
							yr = (t12 + xm2*xv21 + ym2*yv21 - xm2*xr) / ym2;
							xyr.push_back(Point2f(xr, yr));
						}
					}
				}
			}
			Scalar teva = sum(xyr);
			if (xyr.size() > 0) {
				this->xy_vector_within_the_same_plain.x = (float)teva[0] / (float)xyr.size();
				this->xy_vector_within_the_same_plain.y = (float)teva[1] / (float)xyr.size();
				this->xy_vector_within_the_same_plain = this->center - this->xy_vector_within_the_same_plain;
			}else{
				return -1;//unreliable case
			}
			teva = sum(eva_distribution);
			if (teva[0] > 3*this->RegionMaximum) {
				return 2;//reliable case
			}
			else if (teva[0] > 2 * this->RegionMaximum) {
				return 1;//less reliable case
			}
			else {
				return 0;//might be not reliable
			}
		}
		else {
			return -1;//unreliable case
		}
	}
	else {
		return -1;//unreliable case
	}
}

//codes from tencent guy, with slightly changes
void EstRXY::tCalRotationAndTranslation(const std::vector<KeyPoint> &object0, const std::vector<KeyPoint> &object1, const std::vector< DMatch > &matches,
	double &rotationAngle, Point2f& xy_v){
	vector<DMatch> copyMatches(matches);
	//cout << copyMatches[0].distance << "  " << copyMatches[1].distance << std::endl;
	vector<float> thetaValue;
	for (int i = 0; i < 3; i++)
	{
		if (copyMatches[i + 1].distance > this->maxError)
			break;
		Point2f d0 = (object0[copyMatches[i].queryIdx].pt - object0[copyMatches[i + 1].queryIdx].pt);
		Point2f d1 = (object1[copyMatches[i].trainIdx].pt - object1[copyMatches[i + 1].trainIdx].pt);
		double len = sqrtf(d0.x * d0.x + d0.y * d0.y);
		d0.x /= len;
		d0.y /= len;

		double len1 = sqrtf(d1.x * d1.x + d1.y * d1.y);
		d1.x /= len1;
		d1.y /= len1;

		double cosTheta = d0.dot(d1);
		double sinTheta = d0.cross(d1);

		float tTheta = acos(cosTheta) / 3.1415926 * 180.0;
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
	Point2f centerPos(320, 180);
	Point2f translation;
	int count = 0;
	for (int i = 0; i < 3; i++)
	{
		if (copyMatches[i].distance > maxError)
			continue;

		Point2f dd0 = object0[copyMatches[i].queryIdx].pt - centerPos;
		float angleInRadian = rotationAngle / 180.0f * 3.1415926f;
		Point2f nDD0;
		nDD0.x = dd0.x * cosf(angleInRadian) - dd0.y * sinf(angleInRadian);
		nDD0.y = dd0.x * sinf(angleInRadian) + dd0.y * cosf(angleInRadian);
		Point2f newDD0 = centerPos + nDD0;
		Point2f tv = newDD0 - object1[copyMatches[i].trainIdx].pt;
		//std::cout << "Sub_translation:  " << tv << std::endl;
		translation += tv;
		count++;
	}
	translation.x /= count;
	translation.y /= count;
	xy_v = translation;
	//std::cout << "translation:  " << translation << std::endl;
}
