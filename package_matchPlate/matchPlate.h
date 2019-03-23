#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <math.h>
#include <string.h>

using namespace cv;
using namespace std;

class matchPlate
{
	private:
		int thresh = 50;
		int N = 11;			// visit N times;
		int maxPlateArea = 4000;    // suppose to be 77*30*150%
		int minPlateArea = 1800;	// suppose to be 80*35*70%
		string wndname;
		vector< vector<Point> > squares;
		Mat image;
	public:
		bool debug;
		bool setImage(const Mat & image);
		bool findSquares();
		void drawSquares();
		matchPlate();
		~matchPlate();
	private:
		double angle(Point pt1, Point pt2, Point pt0);
		bool filtSquares();
		double distanceToIdealLine(cv::Point p);    // called in 
};