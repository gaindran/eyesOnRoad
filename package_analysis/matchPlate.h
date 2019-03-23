//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string.h>

class matchPlate
{
	private:
		int thresh = 50;
		int N = 11;			// visit N times;
		int maxPlateArea = 4000;    // suppose to be 77*30*150%
		int minPlateArea = 1800;	// suppose to be 80*35*70%
		std::string wndname;
		std::vector< std::vector<cv::Point> > squares;
		cv::Mat image;
	public:
		bool debug;
		bool setImage(const cv::Mat & image);
		bool findSquares();
		void drawSquares();  // draw all of squares
		cv::Rect getRect(size_t ith);
		cv::Rect getRect();
		cv::Point leftTopCorner(size_t ith);
		matchPlate();
		~matchPlate();
	private:
		double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
		bool filtSquares();
		double distanceToIdealLine(cv::Point p);    // called in filtSquares() 
		cv::Point rightBottomCorner(size_t ith);    // called in getRect(ith);  Rect of i_th squares;
};
