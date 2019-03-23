#include "matchPlate.h"
#include <algorithm>   // for max/min
using namespace cv;
using namespace std;
// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double matchPlate::angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
bool matchPlate::setImage(const Mat & img)
{
	if(img.empty())
		return(false);
	else
		image = img;
}
// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
bool matchPlate::findSquares()
{
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > minPlateArea &&
					fabs(contourArea(Mat(approx))) < maxPlateArea &&
                    isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
	return(filtSquares());
}

// the function draws all the squares in the image
void matchPlate::drawSquares()
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 1, CV_AA);
		if (debug)
		{
			std::cout << i << ": [x= " << squares[i][0].x << ", y=" << squares[i][0].y << "]";
			std::cout << i << ": [x= " << squares[i][1].x << ", y=" << squares[i][1].y << "]";
			std::cout << i << ": [x= " << squares[i][2].x << ", y=" << squares[i][2].y << "]";
			std::cout << i << ": [x= " << squares[i][3].x << ", y=" << squares[i][3].y << "]" << endl;
		}
    }

	if (debug)
	{
		imshow(wndname, image);
	}
		
}
matchPlate::matchPlate():debug(false) {
	std::cout << "matchPlate()" << std::endl;
	wndname = "Matching plate Debug";
	debug = true;
}
matchPlate::~matchPlate() {
	squares.clear();
	std::cout << "~matchPlate()" << std::endl;
}
// called in findSquares, filting sqaures to only ONE which is nearest and bigest plate.
bool matchPlate::filtSquares()
{
	if (squares.empty())
		return false;
	// filting best one
	vector<double> weight(squares.size(), 0);   // compute the [1]distance between left top corner location and y=330-x/2
												//             [2] ratio of plate area over 77*30 pixels in percent%
												// weight = [1] + [2]

	for (size_t i = 0; i < squares.size(); i++)
	{
		int index = (int)i;
		// find the left top corner of rectangle
		int n = (int)squares[i].size();
		int x = 100000;    // suppose 100000 is the max location of x
		int y = 100000;
		for (int j = 0; j < n; j++)
		{
			// std::cout << "comparing :" << squares[i].at(j).x << " and " << x;
			x = min(squares[i][j].x, x);
		}
		// std::cout << "Answer x is " << x << std::endl;
		for (int j = 0; j < n; j++)
		{
			// std::cout << "comparing :" << squares[i].at(j).y << " and " << y << std::endl;
			y = min(squares[i][j].y, y);
		}
		// std::cout << "Answer y is " << y << std::endl;
		cv::Point p(x, y);     // p is the left top corner, which minimun of (x,y)s
		double distance = distanceToIdealLine(p);
		// std::cout << "Power(distance,2) to line is" << distance << std::endl;
		double ratio = contourArea(squares[i], false) / (77 * 30) * 100;
		// std::cout << "Percent of ratio is" << ratio << std::endl;
		weight[index] = distance + ratio;
		// std::cout << "Weight is :" << weight[index] << std::endl;
	}
	// find the maximun weight value, erase all but maximun 
	vector<double>::iterator biggest = std::max_element(std::begin(weight), std::end(weight));
	int i = std::distance(std::begin(weight), biggest);
	std::cout << "idx of max is >" << i << std::endl;
	vector<Point> answer = squares.at(i);
	squares.clear();
	squares.push_back(answer);
	return true;
}
// called in filtSquares()
double matchPlate::distanceToIdealLine(Point p)
{   // distance from point to line(ax+by+c=0) is |ax+by+c|/sqrt(a2 + b2)
	double t1 = p.x * 0.76;
	double t2 = t1 + (double)p.y - 330;
	double t3 = abs(t2) / sqrt(0.76*0.76 + 1);
	t3 = t3 < 60 ? 0 : t3;     // if distance less then 60 , we take this distance as NO difference;
	return(-1*( t3 * t3 ));    // shorter distance is better , so multply the minus sign
}
/*
int main(int argc, char** argv)
{
	//char* names = "c:/images/2998274.jpg";   // Lexus
	//char* names = "C:/images/Mastering OpenCV/test/12345.jpg";  // Benz
	//char* names = "C:/images/Mastering OpenCV/test/2998275.jpg";  // Vios
	//char* names = "C:/images/photo_1342_39350.jpg";  // Audi
	char* names = "C:/images/images.jpg";  // Audi
    namedWindow( wndname, 1 );

	// 長方形邊
    vector<vector<Point> > squares;

	Mat image = imread(names, 1);
    if( image.empty() )
    {
		cout << "Couldn't load " << names << endl;
		return 1;
    }

    findSquares(image, squares);
    drawSquares(image, squares);

    waitKey(0);

    return 0;
}
*/
