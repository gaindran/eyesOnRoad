#include <fstream>  // for ofstream within std::string
#include <iostream>
#include <string>   // for string ...
#include <sys/stat.h> // for mkdir()
#include <math.h>   // for floor, fmod <-- mod of float/double
#include <opencv2/opencv.hpp>

#include "package_bgs/PBAS/PixelBasedAdaptiveSegmenter.h"
#include "package_tracking/BlobTracking.h"
#include "package_analysis/VehicleCouting.h"
// #include "package_analysis/matchPlate.h"
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
bool checkRect(cv::Rect & rect)
{
	return( rect.x >=0 && rect.y >= 0 && rect.width>0 && rect.height>0);
}
int mkDir(std::string str)
{
	if(str!="")
	{
		const int errNo = mkdir(str.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		return(errNo);   // -1 mean failed!
	}
	else
		return(-1);
}
bool isPathExist(const std::string &str)
{
  struct stat buffer;
  return (stat (str.c_str(), &buffer) == 0);
}
std::string videoFile;   // Global var. from parameter 1:input video file name
std::string vFileWOext;  // Global var. :video filename without extension
std::string dirName;     // Global var. parameter 2: user defined directory name for snapshots
std::string outputTxtFile; // Global var.  parameter3 : user defined filename of output text file
double fps;
int preprocess(int argc, char ** argv)
{
  if(argc < 2)
  {
    std::cout << "This Program Needs One parameters at Least!" << std::endl;
    std::cout << "Usage: parameter 1 is filename of video" << std::endl;
    std::cout << "      (parameter 2) is directories of snapshots output" << std::endl;
    std::cout << "      (parameter 3) is filename of the output text file" << std::endl;
    return(-1);
  }
  if(argc >= 2 )
  {
    videoFile = std::string(argv[1]);
    int s_at = videoFile.find("/",0);
    int s_length = videoFile.length();
    if(s_at >= 0 && s_at < s_length)  // found "/" in the videoFile string
    {
      videoFile = videoFile.assign(videoFile, s_at+1, s_length);
    }
    // trim .ext from videoFile , stored to vFileWOext, example: image.mp4 -> image
    vFileWOext = videoFile.assign(videoFile,0,videoFile.find(".",0) );
  }
  if(argc >= 3)
  {
    dirName = std::string(argv[2]);
  }
  else
  {
    std::cout << "Set Directory name :" << dirName << "by default";
  }
  if(! isPathExist(dirName))
  {
    int errNo=mkDir(dirName);
    if(errNo==-1)
    {
      std::cout << "Making Directory :" <<  dirName << "Failed" << std::endl;
      return(-1);
    }
    std::cout  << "Making Directory :" << dirName << "...Success!" << std::endl;
  }
  if(argc == 4 )
  {
    outputTxtFile = std::string(argv[3]);
  }
  else
  {
    outputTxtFile = vFileWOext + ".txt";
  }
  std::cout << "Plate OCR result stored in :" << dirName << "/" << outputTxtFile << std::endl;
  if(argc > 4)
    std::cout << "Caution! Numbers of parameters are Greater than 4(" << argc << ")" << std::endl;
  return(argc);
}
std::string  frame2HHMMSS(int frame)
{
  double seconds = frame/fps;
  int hour = floor((seconds/60) / 60);
  int minute = fmod( floor(seconds/60), 60);
  int second = fmod( seconds, 60);
  std::string hourStr(std::to_string(hour) );
  std::string minStr(std::to_string(minute));
  std::string secStr(std::to_string(second));
  std::string answer = hourStr + " " + minStr + " " + secStr + " ";
  return(answer);
}
void saveSnapshot(int frame, cv::Mat & img)
{
  double seconds = frame/fps;
  int hour = floor((seconds/60) / 60);         // == frame/fps/3600
  int minute = fmod( floor(seconds/60) ,60);  // means integer version (seconds/60)%60
  int second = fmod( seconds ,60);           // means integer version seconds%60
  std::string hourStr(std::to_string(hour) );
  std::string minStr(std::to_string(minute) );
  std::string secStr(std::to_string(second) );
  std::string iStr(std::to_string(frame) );
  std::string filename(dirName+"/"+hourStr+"-"+minStr+"-"+secStr+"-"+iStr+".jpg");
  std::cout << "Saving :<" << filename << ">" << std::endl;
  cv::imwrite(filename.c_str(),img);
}
int main(int argc, char ** argv)
{
  if(preprocess(argc, argv) == -1)
    return(-1);
  int i = 0;   // for counting video frame
  int plateCounter = 0;
  int truckCounter = 0;
  bool f_reSnapshot = false;  // signal  to take 2nd snapshot
  bool f_takeSnapshot2 = false;  // flag for time to take 2nd snapshot at next frame
  std::string answer;
  fps = 0;
  std::cout << "Using OpenCV " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << std::endl;
  /* Prepare for tesseract */
  tesseract::TessBaseAPI *api = new tesseract::TessBaseAPI();
  api->Init(NULL,"eng", tesseract::OEM_DEFAULT);
  /* Open video file */
  CvCapture *capture = 0;
  capture = cvCaptureFromAVI(argv[1]);
  if(!capture){
    std::cerr << "Cannot open video!" << std::endl;
    return 1;
  }
  else
  {
    fps = cvGetCaptureProperty(capture, cv::CAP_PROP_FPS);
    std::cout << "fps of " << argv[1] <<  " is :" << fps << std::endl;
  }
  matchPlate mPlateAPI;
  /* Background Subtraction Algorithm */
  IBGS *bgs;
  bgs = new PixelBasedAdaptiveSegmenter;
  /* Blob Tracking Algorithm */
  cv::Mat img_blob;
  BlobTracking* blobTracking;
  blobTracking = new BlobTracking;
  // blobTracking->debugTrack = true;
  // blobTracking->debugBlob = true;
  /* Vehicle Counting Algorithm */
  VehicleCouting* vehicleCouting;
  vehicleCouting = new VehicleCouting;

  // std::cout << "Press 'q' to quit..." << std::endl;
  int key = 0;
  IplImage *frame;
  while(key != 'q')
  {
    i++;
    frame = cvQueryFrame(capture);
    // if(!frame) break;
    if( ! frame) 
      break;
    cv::Mat img_input = cv::cvarrToMat(frame);

    // bgs->process(...) internally process and show the foreground mask image
    cv::Mat img_mask;
    bgs->process(img_input, img_mask);
    if(f_takeSnapshot2)  // save snapshot 2
    {
      // std::cout << "Saving 2nd img_input" << std::endl;
      saveSnapshot(i, img_input);
      f_takeSnapshot2 = false;
    }
    if(!img_mask.empty())
    {
      // Perform blob tracking
      blobTracking->process(img_input, img_mask, img_blob);

      // Perform vehicle counting
      vehicleCouting->setInput(img_blob);
      vehicleCouting->setTracks(blobTracking->getTracks());
      if(vehicleCouting->process())
      {
        truckCounter++;
        cv::Mat img_copy;
        img_input.copyTo(img_copy);
        // std::cout << "img_copy size is " << img_copy.size().width << "*" << img_copy.size().height << "+" << img_copy.channels() << std::endl;
        cv::Mat sub = img_copy(cv::Rect(1,401, img_copy.size().width/2, img_copy.size().height-401-1)); // origin is 1280*720
        // std::cout << "sub size is " << sub.size().width << "*" << sub.size().height << "+" << sub.channels() << std::endl;
	// cv::imwrite("sub.jpg",sub);
	mPlateAPI.setImage(sub);
	if(mPlateAPI.findSquares())
	{
	  mPlateAPI.drawSquares();
	  cv::Rect rect = mPlateAPI.getRect();
	  if(checkRect(rect))  // check if rect of plate square available?
	  {
            plateCounter++;
	    cv::Mat plate = sub(rect);
	    // std::cout << "plate size is" << plate.size().width << "*" << plate.size().height << std::endl;
            saveSnapshot(i, img_input);
            cv::Mat freshPlate = cv::Mat::zeros(plate.size(), plate.type());
            plate.convertTo(freshPlate, -1, 2, 50);
            saveSnapshot(i-1, freshPlate);
	    api->SetImage((uchar*)freshPlate.data, freshPlate.size().width, freshPlate.size().height, freshPlate.channels(), freshPlate.step1());
	    char *outText = api->GetUTF8Text();   // get text in image
            // check outText pointer is NULL or '\0' , means found empty string
            if( (outText != NULL) && (outText[0]!='\0') )
            {
              f_reSnapshot = true;
              // std::cout << "|->" << outText  << std::endl;
              std::string timeStr = frame2HHMMSS(i);
              answer = timeStr + std::string(outText);
            }
	    delete [] outText;
	  }
	  // vehicleCouting->mytest();
	}
      }
      if(f_reSnapshot)
      {
        // std::cout << "Calling to take 2nd snapshot" << std::endl;
        f_takeSnapshot2 = true;
        f_reSnapshot = false;
      }
    }
    key = cvWaitKey(1);
  }
  // Export outText answer to  outputTxtFile
  if(answer!="")
  {
    std::string wholePath=dirName+"/"+outputTxtFile;
    std::ofstream fileIO(wholePath.c_str());
    std::cout << answer << std::endl;
    fileIO << answer;
    fileIO.close();
  }

  delete vehicleCouting;
  delete blobTracking;
  delete bgs;

  cvDestroyAllWindows();
  cvReleaseCapture(&capture);
  // Only for tesseract
  api->End();
  return 0;

}
