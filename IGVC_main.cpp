#include "IGVC_imgproc.h"

using namespace cv;

int main(){

	String filepath = "C:/Users/student16/Desktop/IGVC_Test.mov";

	// OpenCV video capture class, required for camera interaction.
	const int num_cameras = 1;
	VideoCapture captures [num_cameras];
	Mat cam_frames [num_cameras];
	string labels [num_cameras];
	vector<vector<Point> > contours [num_cameras];
	vector<Vec4i> hierarchy [num_cameras];
	vector<Point> all_points [num_cameras];
	//vector<Point> nearestToCamera [num_cameras];
	// vector<float> radius [num_cameras];
	// vector<Point2f> center [num_cameras];

	int Xres = 432; // Pixels
	int Yres = 240; // Pixels

	for(int idx = 0; idx < num_cameras; idx++){
		captures[idx].open(filepath);
		//captures[idx].set(CV_CAP_PROP_FRAME_WIDTH, Xres);
		//captures[idx].set(CV_CAP_PROP_FRAME_HEIGHT, Yres);
		//captures[idx].set(CV_CAP_PROP_FPS, 30);

		if(!captures[idx].isOpened()){
			CV_Assert("Camera failed to open!");
		}

		labels[idx] = "Camera " + std::to_string(idx);
	}
	
	// UNCOMMENT THIS TO OPEN CAMERA DRIVER WINDOW, ONE AT A TIME!
	//captures[0].set(CV_CAP_PROP_SETTINGS, 1);

	// Loop continuously until ESC is pressed.
	while(waitKey(1) != 27){
		for(int cam = 0; cam < num_cameras; cam++){

			// Frame capture
			Mat original, after_clahe, after_morphology, after_threshold;
			captures[cam] >> cam_frames[cam];
			resize(cam_frames[cam], cam_frames[cam], Size(Xres, Yres), 0, 0, INTER_CUBIC);
			
			cam_frames[cam].copyTo(original);

			// Contrast Limited Adaptive Histogram Equalization.
			//igvc_CLAHE(cam_frames[cam]);
			//cam_frames[cam].copyTo(after_clahe);

			// Thresholding
			igvc_threshold(cam_frames[cam]);
			cam_frames[cam].copyTo(after_threshold);

			// Morphology
			igvc_morphology(cam_frames[cam], 3);
			cam_frames[cam].copyTo(after_morphology);

			// Find contours
			igvc_findContours(cam_frames[cam], 40, contours[cam], hierarchy[cam]);

			// Polynomial curve fitting in the bottom left and bottom right quadrants.
			igvc_quadrants_bestFit(all_points[cam], contours[cam], Xres, Yres);
			std::cout << "all_points length: " << all_points[cam].size() << "\n";


			// Find minmum distance from object contours to the camera.
			//igvc_ContourMinimalDistance(nearestToCamera[cam], contours[cam], Xres, Yres);

			// Draw minimum distance lines (unnecessary).
			//vector<Point>::iterator it;
			//for(it = nearestToCamera[cam].begin(); it < nearestToCamera[cam].end(); it++){
			//	circle(cam_frames[cam], Point(it->x, it->y), 3, Scalar(0,0,255));
			//}

			// std::cout << "Size of: minimums: " << nearestToCamera[cam].size() << " | Contours: " << contours[cam].size() << "\n";

			// Show output images (unnecessary).
			imshow("Original", original);
			//imshow("After CLAHE", after_clahe);
			imshow("After Morph", after_morphology);
			imshow("After Threshold", after_threshold);
			//imshow(labels[cam], cam_frames[cam]);
		}
	}

	for(int cam = 0; cam < num_cameras; cam++){
		captures[cam].release();
	}
	return 0;
}