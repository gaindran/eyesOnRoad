#include <algorithm>
#include "VehicleCouting.h"

namespace FAV1
{
  IplImage* img_input1 = 0;
  IplImage* img_input2 = 0;
  int roi_x0 = 0;
  int roi_y0 = 0;
  int roi_x1 = 0;
  int roi_y1 = 0;
  int numOfRec = 0;
  int startDraw = 0;
  bool roi_defined = false;
  bool use_roi = true;
  void VehicleCouting_on_mouse(int evt, int x, int y, int flag, void* param)
  {
    if(!use_roi)
      return;
  
    if(evt == CV_EVENT_LBUTTONDOWN)
    {
      if(!startDraw)
      {
        roi_x0 = x;
        roi_y0 = y;
        startDraw = 1;
      }
      else
      {
        roi_x1 = x;
        roi_y1 = y;
        startDraw = 0;
        roi_defined = true;
      }
    }

    if(evt == CV_EVENT_MOUSEMOVE && startDraw)
    {
      //redraw ROI selection
      img_input2 = cvCloneImage(img_input1);
      cvLine(img_input2, cvPoint(roi_x0,roi_y0), cvPoint(x,y), CV_RGB(255,0,255));
      cvShowImage("VehicleCouting", img_input2);
      cvReleaseImage(&img_input2);
    }
  }
}

VehicleCouting::VehicleCouting(): firstTime(true), showOutput(true), key(0), countAB(0), countBA(0), showAB(0)
{
  std::cout << "VehicleCouting()" << std::endl;
}

VehicleCouting::~VehicleCouting()
{
  std::cout << "~VehicleCouting()" << std::endl;
}

void VehicleCouting::setInput(const cv::Mat &i)
{
  //i.copyTo(img_input);
  img_input = i;
}

void VehicleCouting::setTracks(const cvb::CvTracks &t)
{
  tracks = t;
}

VehiclePosition VehicleCouting::getVehiclePosition(const CvPoint2D64f centroid)
{
  VehiclePosition vehiclePosition = VP_NONE;

  if(laneOrientation == LO_HORIZONTAL)
  {
    if(centroid.x < FAV1::roi_x0)
    {
      cv::putText(img_input, "STATE: A", cv::Point(10,img_h/2), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      vehiclePosition = VP_A;
    }
    
    if(centroid.x > FAV1::roi_x0)
    {
      cv::putText(img_input, "STATE: B", cv::Point(10,img_h/2), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      vehiclePosition = VP_B;
    }
  }

  if(laneOrientation == LO_VERTICAL)
  {
    if(centroid.y > FAV1::roi_y0)
    {
      cv::putText(img_input, "STATE: A", cv::Point(10,img_h/2), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      vehiclePosition = VP_A;
    }
    
    if(centroid.y < FAV1::roi_y0)
    {
      cv::putText(img_input, "STATE: B", cv::Point(10,img_h/2), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      vehiclePosition = VP_B;
    }
  }

  return vehiclePosition;
}
// Must call process() before mytest()
void VehicleCouting::mytest()
{
  if(img_input.empty())
    return;
  for(std::map<cvb::CvID, cvb::CvTrack*>::iterator it = tracks.begin();
      it != tracks.end(); it ++)
  {

    cvb::CvID id = (*it).first;
    cvb::CvTrack* track = (*it).second;
    CvPoint2D64f centroid = track->centroid;
    std::cout<< " - - - - - - - - - - - - - - - - - - - - - - - - - -" << std::endl;
    std::cout<< "id:" << id << " (" << centroid.x << "," << centroid.y << ")" << std::endl;
    std::cout<< (*it).first << " ==> track labe: " << (*it).second->label << std::endl;
  }
}
bool VehicleCouting::process()
{
  if(img_input.empty())
    return(false);
  bool f_snapshot = false;
  img_w = img_input.size().width;
  img_h = img_input.size().height;

  loadConfig();

  //--------------------------------------------------------------------------

  if(FAV1::use_roi == true && FAV1::roi_defined == false && firstTime == true)
  {
    do
    {
      cv::putText(img_input, "Please, set the counting line", cv::Point(10,15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
      cv::imshow("VehicleCouting", img_input);
      FAV1::img_input1 = new IplImage(img_input);
      cvSetMouseCallback("VehicleCouting", FAV1::VehicleCouting_on_mouse, NULL);
      key = cvWaitKey(0);
      delete FAV1::img_input1;

      if(FAV1::roi_defined)
      {
        std::cout << "Counting line defined (" << FAV1::roi_x0 << "," << FAV1::roi_y0 << "," << FAV1::roi_x1 << "," << FAV1::roi_y1 << ")" << std::endl;
        break;
      }
      else
        std::cout << "Counting line undefined!" << std::endl;
    }while(1);
  }

  if(FAV1::use_roi == true && FAV1::roi_defined == true)
    cv::line(img_input, cv::Point(FAV1::roi_x0,FAV1::roi_y0), cv::Point(FAV1::roi_x1,FAV1::roi_y1), cv::Scalar(0,0,255));
  
  bool ROI_OK = false;
  
  if(FAV1::use_roi == true && FAV1::roi_defined == true)
    ROI_OK = true;

  if(ROI_OK)
  {
    laneOrientation = LO_NONE;

    if(abs(FAV1::roi_x0 - FAV1::roi_x1) < abs(FAV1::roi_y0 - FAV1::roi_y1))
    {
      if(!firstTime)
        cv::putText(img_input, "HORIZONTAL", cv::Point(10,15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      laneOrientation = LO_HORIZONTAL;

      cv::putText(img_input, "(A)", cv::Point(FAV1::roi_x0-32,FAV1::roi_y0), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      cv::putText(img_input, "(B)", cv::Point(FAV1::roi_x0+12,FAV1::roi_y0), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
    }

    if(abs(FAV1::roi_x0 - FAV1::roi_x1) > abs(FAV1::roi_y0 - FAV1::roi_y1))
    {
      if(!firstTime)
        cv::putText(img_input, "VERTICAL", cv::Point(10,15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      laneOrientation = LO_VERTICAL;

      cv::putText(img_input, "(A)", cv::Point(FAV1::roi_x0,FAV1::roi_y0+22), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      cv::putText(img_input, "(B)", cv::Point(FAV1::roi_x0,FAV1::roi_y0-12), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
    }
  }

  //--------------------------------------------------------------------------

  for(std::map<cvb::CvID,cvb::CvTrack*>::iterator it = tracks.begin() ; it != tracks.end(); it++)
  {
    //std::cout << (*it).first << " => " << (*it).second << std::endl;
    cvb::CvID id = (*it).first;
    cvb::CvTrack* track = (*it).second;

    CvPoint2D64f centroid = track->centroid;
/*
    std::cout << "---------------------------------------------------------------" << std::endl;
    std::cout << "0)id:" << id << " (" << centroid.x << "," << centroid.y << ")" << std::endl;
    std::cout << "track->active:" << track->active << std::endl;
    std::cout << "track->inactive:" << track->inactive << std::endl;
    std::cout << "track->lifetime:" << track->lifetime << std::endl;
*/

    //----------------------------------------------------------------------------

    if(track->inactive == 0)
    {
      if(positions.count(id) > 0)
      {
        std::map<cvb::CvID, VehiclePosition>::iterator it2 = positions.find(id);
        VehiclePosition old_position = it2->second;

        VehiclePosition current_position = getVehiclePosition(centroid);

        if(current_position != old_position)
        {
          if(old_position == VP_A && current_position == VP_B)
          {
            countAB++;
          }
          if(old_position == VP_B && current_position == VP_A)
          {
            countBA++;
          }
          f_snapshot = checkSnapshot(current_position);
          positions.erase(positions.find(id));
        }
      }
      else
      {
        VehiclePosition vehiclePosition = getVehiclePosition(centroid);

        if(vehiclePosition != VP_NONE)
          positions.insert(std::pair<cvb::CvID, VehiclePosition>(id,vehiclePosition));
      }
    }
    else
    {
      if(positions.count(id) > 0)
        positions.erase(positions.find(id));
    }

    //----------------------------------------------------------------------------

    if(points.count(id) > 0)
    {
      std::map<cvb::CvID, std::vector<CvPoint2D64f>>::iterator it2 = points.find(id);
      std::vector<CvPoint2D64f> centroids = it2->second;
      
      std::vector<CvPoint2D64f> centroids2;
      if(track->inactive == 0 && centroids.size() < 30)
      {
        centroids2.push_back(centroid);
      
        for(std::vector<CvPoint2D64f>::iterator it3 = centroids.begin() ; it3 != centroids.end(); it3++)
        {
          centroids2.push_back(*it3);
        }

        for(std::vector<CvPoint2D64f>::iterator it3 = centroids2.begin() ; it3 != centroids2.end(); it3++)
        {
          cv::circle(img_input, cv::Point((*it3).x,(*it3).y), 3, cv::Scalar(255,255,255), -1);
        }
      
        points.erase(it2);
        points.insert(std::pair<cvb::CvID, std::vector<CvPoint2D64f>>(id,centroids2));
      }
      else
      {
        points.erase(it2);
      }
    }
    else
    {
      if(track->inactive == 0)
      {
        std::vector<CvPoint2D64f> centroids;
        centroids.push_back(centroid);
        points.insert(std::pair<cvb::CvID, std::vector<CvPoint2D64f>>(id,centroids));
      }
    }
    // cv::waitKey(0);
  }
  
  //--------------------------------------------------------------------------

  std::string countABstr = "A->B: " + std::to_string(countAB);
  std::string countBAstr = "B->A: " + std::to_string(countBA);

  if(showAB == 0)
  {
    cv::putText(img_input, countABstr, cv::Point(10, img_h - 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(img_input, countBAstr, cv::Point(10, img_h - 8), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
  }
  
  if(showAB == 1)
    cv::putText(img_input, countABstr, cv::Point(10, img_h - 8), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
  
  if(showAB == 2)
    cv::putText(img_input, countBAstr, cv::Point(10, img_h - 8), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));

  if(showOutput)
    cv::imshow("VehicleCouting", img_input);

  if(firstTime)
    saveConfig();

  firstTime = false;
  return(f_snapshot);
}

void VehicleCouting::saveConfig()
{
  CvFileStorage* fs = cvOpenFileStorage("config/VehicleCouting.xml", 0, CV_STORAGE_WRITE);

  cvWriteInt(fs, "showOutput", showOutput);
  cvWriteInt(fs, "showAB", showAB);
  
  cvWriteInt(fs, "fav1_use_roi", FAV1::use_roi);
  cvWriteInt(fs, "fav1_roi_defined", FAV1::roi_defined);
  cvWriteInt(fs, "fav1_roi_x0", FAV1::roi_x0);
  cvWriteInt(fs, "fav1_roi_y0", FAV1::roi_y0);
  cvWriteInt(fs, "fav1_roi_x1", FAV1::roi_x1);
  cvWriteInt(fs, "fav1_roi_y1", FAV1::roi_y1);
  
  cvReleaseFileStorage(&fs);
}

void VehicleCouting::loadConfig()
{
  CvFileStorage* fs = cvOpenFileStorage("config/VehicleCouting.xml", 0, CV_STORAGE_READ);

  showOutput = cvReadIntByName(fs, 0, "showOutput", true);
  showAB = cvReadIntByName(fs, 0, "showAB", 1);
  
  FAV1::use_roi = cvReadIntByName(fs, 0, "fav1_use_roi", true);
  FAV1::roi_defined = cvReadIntByName(fs, 0, "fav1_roi_defined", false);
  FAV1::roi_x0 = cvReadIntByName(fs, 0, "fav1_roi_x0", 0);
  FAV1::roi_y0 = cvReadIntByName(fs, 0, "fav1_roi_y0", 0);
  FAV1::roi_x1 = cvReadIntByName(fs, 0, "fav1_roi_x1", 0);
  FAV1::roi_y1 = cvReadIntByName(fs, 0, "fav1_roi_y1", 0);
  
  cvReleaseFileStorage(&fs);
}

bool VehicleCouting::checkSnapshot(VehiclePosition p)
{
  switch(showAB)
  {
    case 0: // both size
      if(p != VP_NONE)
          return true;
      else
          return false;
      break;
    case 1: // A->B
      if(p == VP_B)
          return true;
      else
          return false;
      break;
    case 2: // B->A
      if(p == VP_A)
          return true;
      else
          return false;
      break;
  }
}
// - - - - - - - -matchPlate class - - - - - - - - - - - - -
double matchPlate::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
bool matchPlate::setImage(const cv::Mat & img)
{
	if(img.empty())
		return(false);
	image = img;
	return(true);
}
// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
bool matchPlate::findSquares()
{
    squares.clear();

    cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    std::vector< std::vector<cv::Point> > contours;

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
                dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            std::vector<cv::Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(cv::Mat(approx))) > minPlateArea &&
		    fabs(contourArea(cv::Mat(approx))) < maxPlateArea &&
                    isContourConvex( cv::Mat(approx)) )
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
        const cv::Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, cv::Scalar(0,255,0), 1, CV_AA);
	if (debug)
	{
		std::cout << i << ": [x= " << squares[i][0].x << ", y=" << squares[i][0].y << "]";
		std::cout << "- [x= " << squares[i][1].x << ", y=" << squares[i][1].y << "]";
		std::cout << "- [x= " << squares[i][2].x << ", y=" << squares[i][2].y << "]";
		std::cout << "- [x= " << squares[i][3].x << ", y=" << squares[i][3].y << "]" << std::endl;
	}
    }

    if (debug)
    {
        imshow(wndname, image);
    }
}
cv::Rect matchPlate::getRect()
{
	return(getRect((size_t) 0));
}
cv::Rect matchPlate::getRect(size_t ith)
{
	cv::Point lt = leftTopCorner(ith);
	cv::Point rb = rightBottomCorner(ith);
	int width = rb.x - lt.x;
	int height = rb.y - lt.y;
	cv::Rect rect;
	int temp = 0;
	if(width > 0 && height > 0 && width <= image.size().width && height <= image.size().height) 
	{
		rect.x = lt.x;
		rect.y = lt.y;
		temp = rb.x - lt.x;
		rect.width = (temp>0)? temp:0;
		temp = rb.y - lt.y;
		rect.height = (temp>0)? temp:0;
	}
	else
	{
		cv::Rect rect( 0,0,0,0 );
	}
	return(rect);
}
cv::Point matchPlate::leftTopCorner(size_t ith=0)
{
	cv::Point p(0,0);
	if(squares.empty())
		return(p);
	int x = 100000;
	int y = 100000;
	int n = (int)squares[ith].size();
	for (int j = 0; j < n; j++)
	{
		x = std::min(squares[ith][j].x, x);
	}
	for (int j = 0; j < n; j++)
	{
		y = std::min(squares[ith][j].y, y);
	}
	p.x = x;
	p.y = y;
	return(p);
}
cv::Point matchPlate::rightBottomCorner(size_t ith)
{
	cv::Point p(0,0);
	if(squares.empty())
		return(p);
	int x = 0;
	int y = 0;
	int n = (int)squares[ith].size();
        for (int j = 0; j < n; j++)
        {
                x = std::max(squares[ith][j].x, x);
        }
        for (int j = 0; j < n; j++)
        {
                y = std::max(squares[ith][j].y, y);
        }
	p.x = x;
	p.y = y;
        return(p);
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
	std::vector<double> weight(squares.size(), 0);   // compute the [1]distance between left top corner location and y=330-x/2
							//             [2] ratio of plate area over 77*30 pixels in percent%
							// weight = [1] + [2]

	for (size_t i = 0; i < squares.size(); i++)
	{
		int index = (int)i;
		cv::Point p = leftTopCorner(i);
		double distance = distanceToIdealLine(p);
		// std::cout << "Power(distance,2) to line is" << distance << std::endl;
		double ratio = contourArea(squares[i], false) / (77 * 30) * 100;
		// std::cout << "Percent of ratio is" << ratio << std::endl;
		weight[index] = distance + ratio;
		// std::cout << "Weight is :" << weight[index] << std::endl;
	}
	// find the maximun weight value, erase all but maximun 
	std::vector<double>::iterator biggest = std::max_element(std::begin(weight), std::end(weight));
	int i = std::distance(std::begin(weight), biggest);
	// std::cout << "idx of max is >" << i << std::endl;
	std::vector<cv::Point> answer = squares.at(i);
	squares.clear();
	squares.push_back(answer);
	return true;
}
// called in filtSquares()
double matchPlate::distanceToIdealLine(cv::Point p)
{   // distance from point to line(ax+by+c=0) is |ax+by+c|/sqrt(a2 + b2)
	double t1 = p.x * 0.76;
	double t2 = t1 + (double)p.y - 330;
	double t3 = std::abs(t2) / std::sqrt(0.76*0.76 + 1);
	t3 = t3 < 60 ? 0 : t3;     // if distance less then 60 , we take this distance as NO difference;
	return(-1*( t3 * t3 ));    // shorter distance is better , so multply the minus sign
}
