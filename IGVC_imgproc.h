// IGVC Image Processing Functions
// Author - cestock
// Date - 2/22/17

#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <math.h>


using namespace cv;

Point igvc_findNearestPoint(vector<Point> contour, int Xres, int Yres){

	// INPUT:	A set of points making up a single contour. X and Y resolution of the camera.
	// OUTPUT:	The point on the contour with the that is nearest to the vehicle.
	// NOTE:	The 'nearest' point is the point with the minimum distance to (Xres/2, Yres).

	// Create the starting point that is the farthest away from the desired point on the X and Y axis.
	float bestDistance = 99999, testDistance;
	Point result = (0.0, 0.0);
	vector<Point>::iterator it;

	for(it=contour.begin(); it < contour.end(); it++){
		// First calculate the distance between (Xres/2, Yres) and the current point being tested.
		testDistance = sqrt( ((Xres/2) - it->x)*((Xres/2) - it->x) + ( Yres - it->y)*( Yres - it->y) );

		// Then, compare with the latest 'best distance' to see if this point is closer.
		if(testDistance < bestDistance){
			result.x = it->x;
			result.y = it->y;
		}
	}

	return result;
}

void igvc_ContourMinimalDistance( vector<Point> &minimums, vector<vector<Point>> contours, int Xres, int Yres){
	 
	// INPUT:	The set of all contours for a given camera.
	// OUTPUT:	No direct output, just adjusts the set of minimum distance points by reference.
	// NOTE:	Mostly a helper function for the igvc_findNearestPoint function.

	// First clear the existing minimums.
	minimums.clear();

	// Iterate on the contours, passing to the igvc_findNearestPoint function.
	// Pushback a mimumum value on the minimums vector beforehand.
	vector<vector<Point>>::iterator it;
	for(it = contours.begin(); it < contours.end(); it++){
		minimums.push_back(Point(0.0, 0.0));
		minimums.back() = igvc_findNearestPoint(*it, Xres, Yres);
	}

	return;
}

/*
igvc_quadrants_bestFit CURRENTLY WORK IN PROGRESS -- Translates vector<Point> to best fit line/rect/etc. for the bottom quadrants.
*/
void igvc_quadrants_bestFit(vector<Point> &all_points, vector<vector<Point>> contours, int Xres, int Yres){

	// INPUT: Set of all image points(reference) and contours, Xresolution, Yresolution.
	// OUTPUT:

	// First clear the all_points vector.
	all_points.clear();

	//Iterate through the set of all contours, iterate through each contour's points and push them into all_points vector.
	vector<vector<Point>>::iterator it;
	vector<Point>::iterator it2;
	for(it=contours.begin(); it < contours.end(); it++){
		for(it2=it->begin(); it2 < it->end(); it2++){
			all_points.push_back(Point(it2->x, it2->y));
		}
	}

	// Set up the quadrants of the screen that will be the areas to calculate the best fit polynomial curve.
	// Need 'bottom left' and 'bottom right' quadrants. (0,0) is the top left of the image.
	int Xsection = Xres/3, //Left third, Right third = Xres-Xsection
		Ysection = 2*Yres/3; //Lower third
	vector<Point> bottom_left_points, bottom_right_points;

	// Iterate through all_points and calssify each point as belonging to either bottom_left_points or bottom_right_points.
	// depending on thier pixel location in the image.
	for(it2=all_points.begin(); it2 < all_points.end(); it2++){
		if((it2->x < Xsection) && (it2->y > Ysection)){
			bottom_left_points.push_back(*it2);
		}
		else if((it2->x > Xres-Xsection) && (it2->y > Ysection)){
			bottom_right_points.push_back(*it2);
		}
	}

	// Create the curve vectors
	//vector<Point> bottom_left_curve, bottom_right_curve;
	//vector<vector<Point>> left_line, right_line;
	Vec4f left_line, right_line;
	float lineLength = 50;
	Mat canvas = Mat::zeros(Size(Xres,Yres), CV_8UC3);
	if(bottom_left_points.size() != 0){
		//approxPolyDP(bottom_left_points, bottom_left_curve, 5, false);
		//left_line.push_back(bottom_left_curve);
		//drawContours(canvas, left_line, -1, Scalar(0,0,255));
		fitLine(bottom_left_points, left_line, CV_DIST_L2, 0, 0.01, 0.01);
		line(canvas, Point(left_line[2], left_line[3]), Point(left_line[2]+left_line[0]*lineLength, left_line[3]+left_line[1]*lineLength), Scalar(0,0,255));
	}
	if(bottom_right_points.size() != 0){
		//approxPolyDP(bottom_right_points, bottom_right_curve, 5, false);
		//right_line.push_back(bottom_right_curve);
		//drawContours(canvas, right_line, -1, Scalar(255,0,0));
		fitLine(bottom_right_points, right_line, CV_DIST_L2, 0, 0.01, 0.01);
		line(canvas, Point(right_line[2], right_line[3]), Point(right_line[2]+right_line[0]*lineLength, right_line[3]+right_line[1]*lineLength), Scalar(255,0,0));
	}

	imshow("botleft_botright_lines", canvas);

	return;
}

/*
igvc_findCenterOfContour IS NOT CURRENTLY BEING USED -- BETTER ALTERNATIVES.
*/
void igvc_findCenterOfContour(vector<vector<Point> > contours, vector<float> &radius, vector<Point2f> &center, Mat &img){

	for (size_t i = 0; i < contours.size(); i++){
		center.push_back(Point2f(0.0, 0.0));
		radius.push_back(float(0.0));
		minEnclosingCircle(contours[i], center[i], radius[i]);

		// Draw the circle
		// circle(img, center[i], cvRound(radius[i]), Scalar(0,255,255), 1);

		// Draw the centers
		circle(img, center[i], 3, Scalar(0,0,255), CV_FILLED);
	}

	return;
}

/*
igvc_findContours is currently being used to translate image to vector<Point> representing the edges.
*/
void igvc_findContours(Mat &input, int thresh, vector<vector<Point> > &contours, vector<Vec4i> &hierarchy){

	//INPUT: Image matrix (BGR Colorspace)
	//OUTPUT: Edits contours and hierarchy via reference.

	// Create the necessary image matrices.
	Mat source = input, source_gray, canny_out;
	// Convert to greyscale.
	cvtColor(source, source_gray, COLOR_BGR2GRAY);
	// Blur the greyscale image to remove detail that could effect the contouring algorithm.
	blur(source_gray, source_gray, Size(8, 8));
	// Apply the canny edge detection algorithm.
	Canny(source_gray, canny_out, thresh, thresh*3, 3); // Edge detection;
	//  Apply find contours, which converts edges to vectors of Point2f's.
	findContours(canny_out, contours, hierarchy, RETR_TREE, CHAIN_APPROX_TC89_L1, Point(0,0));

	// Create an image matrix for visualizing the edges using RNG'd colors.
	Mat drawing = Mat::zeros(canny_out.size(), CV_8UC3);
	// Seed the RNG value for coloring.
	RNG rng(12345);
	for( size_t i = 0; i< contours.size(); i++ )
    {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
    }

	input += drawing;

	return;
}

/*
igvc_morphology is currently being used as a noise removal algorithm and is very effective post-threshold.
*/
void igvc_morphology(Mat &input, int morphCoeff){

	//INPUT: Image matrix
	//OUTPUT: Edits the input matrix via reference.
	//NOTE: Morphology is designed to be used on a binary (threshold) image for cleaning up unwanted noise.

	morphologyEx(input, input, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(morphCoeff,morphCoeff)));

	//blur(input, input, Size(morphCoeff,morphCoeff));

	morphologyEx(input, input, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(morphCoeff,morphCoeff)));

	return;
}

/*
igvc_threshold is being used as a filtering algorithm for both whites and light greens.
*/
void igvc_threshold(Mat &input){

	//INPUT: Image matrix (BGR)
	//OUTPUT: Edits input matrix via reference.

	Mat HSV_BASE, HSV_ALL_MASK, HSV_WHITE_MASK, HSV_GREEN_MASK, IMG_OUT;
	Scalar hsv_w_low(0, 0, 150), hsv_w_high(180, 70, 255), // Passing
		   hsv_g_low(0,70,0), hsv_g_high(180,255,150); // Excluding

	// Thresholding on main image with contrast adjustment (convertTo).
	cvtColor(input, HSV_BASE, COLOR_BGR2HSV);
	inRange(HSV_BASE, hsv_g_low, hsv_g_high, HSV_GREEN_MASK);
	inRange(HSV_BASE, hsv_w_low, hsv_w_high, HSV_WHITE_MASK);

	//imshow("white mask", HSV_WHITE_MASK);
	//imshow("green mask", HSV_GREEN_MASK);

	// Mix the masks
	HSV_ALL_MASK = HSV_WHITE_MASK - HSV_GREEN_MASK;

	// Apply mask to the unput image matrix.
	bitwise_and(HSV_BASE, HSV_BASE, IMG_OUT, HSV_ALL_MASK);
	cvtColor(IMG_OUT, IMG_OUT, COLOR_HSV2BGR);
	IMG_OUT.copyTo(input);

	return;
}

/*
igvc_CLAHE IS CURRRENTLY NOT USED -- INTRODUCES TOO MUCH GRAINULARITY TO THE IMAGE!
*/
void igvc_CLAHE(Mat &input){
	
	// This is a correction algorithm to adapt an image to various lighting conditions.
	// INPUT:	Image matrix.
	// OUTPUT:	Corrected image matrix by reference.

	// Read the BGR image, create a LAB matrix, and convert BGR to LAB.
	Mat bgr_image = input;
	Mat lab_image;
	cvtColor(bgr_image, lab_image, COLOR_BGR2Lab);

	// Sift out the Luminance (L) channel.
	vector<Mat> lab_planes(3);
	split(lab_image, lab_planes); // Luminance channel is in lab_planes[0].

	// Apply the CLAHE (Contrast Limited Adaptive Histogram Equalization) method.
	Ptr<CLAHE> clahe = createCLAHE();
	clahe->setClipLimit(4);
	Mat dst;
	clahe->apply(lab_planes[0], dst);

	// New re-merge the color planes back into a LAB image.
	dst.copyTo(lab_planes[0]);
	merge(lab_planes, lab_image);

	// Convert LAB to BGR.
	cvtColor(lab_image, input, CV_Lab2BGR);
}