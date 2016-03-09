/**
 * @file main.cpp
 * @brief This program spawns threads to grab and process images and serves 
 * requests from the RoboRIO. In order to run as fast as possible, image 
 * grabbing and image processing is split into two different threads. The 
 * grabbing thread runs as fast as possible, updating the shared Mat variable. 
 * Then the processing thread checks if there is a new image to process and 
 * runs the processing commands if so. Therefore, since image processing is 
 * less time-intensive than downloading the image, the processing command is 
 * run once right after a new image is downloaded and then paused until another 
 * image is acquired.
 */

/**
 * Define PRINT to allow benchmarking on stdout by the image grabbing and 
 * processing threads.
 */

#define PRINT

/**
 * Define DISPLAY to allow display of windows containing intermediate steps
 * in the image processing thread.
 */

//#define DISPLAY

#include "serveRoboRIO.h"

#include <thread>
#include <mutex>

#include <string>
#include <cstring>
#include <vector>
#include <unistd.h>

#ifdef PRINT
#include <chrono>
#include <fstream>
#endif

#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

const int graythresh = 100;
const int erosionsize = 2;
const double minratio = 0.0;
const double maxratio = 0.5;
const int maxedgecount = 20;

/// Log file
std::ofstream logFile ("/var/log/vision.log");

/// Mutex lock for lateral
std::mutex lateralMtx;
/// Horizontal distance to target
double lateral;

/// Mutex lock for image
std::mutex imageMtx;
/// Imaged grabbed from the camera to be processed for targets
Mat image;

/// Mutex lock for freshImage
std::mutex freshImageMtx;
/// Signals to the processing thread whether it has a new image to read
bool freshImage = false;

/// Mutex lock for distanceToTarget
std::mutex distanceMtx;
/// Distance to target received from RoboRIO
double distanceToTarget;

std::mutex dataMtx;
visionData data;

/**
 * Compare the area bounded by two sets of points.
 *
 * @param a First vector of points
 * @param b Second vector of points
 * @return True if Area(a) is greater than Area(b)
 */

bool compareArea(std::vector<Point>, std::vector<Point>);

/**
 * Find the center of the vision targets in a given image
 *
 * @param tmp The image to process for vision targets
 * @return The mean point of the vision targets
 */

Point2f findTarget(Mat);

/**
 * Serve TCP/IP socket connections from the RoboRIO to receive distances
 * from the range finder and return the horizontal distance from the vision
 * targets.
 */

void ServeRoboRIO();

/**
 * Retrieve image from the webcam and copy it to the global Mat as fast as
 * possible.
 */

void GrabImage();

/**
 * Process the image, extracting targets and totes from the image.
 */

void ProcessImage();

int main() {
  distanceToTarget = 20.0;

  // Create threads for each distinct process
  std::thread (ServeRoboRIO).detach();
  std::thread (GrabImage).detach();
  std::thread (ProcessImage).detach();

  // Infinite loop to allow threads to run
  while(1) {}

  return 0;
}

bool compareArea(const std::vector<Point> a, const std::vector<Point> b) {
  return (contourArea(a) > contourArea(b));
}

Point2f findTarget(Mat tmp) {
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;
  Mat colors [3];
  Mat edges;
  double lat = 0;
  Point2f mean (0, 0);

#ifdef PRINT
  std::chrono::high_resolution_clock::time_point begin, end;
  begin = std::chrono::high_resolution_clock::now();

  logFile << "Data in Mat: " << (tmp.data ? "true" : "false") << std::endl;
#endif

  // If there is, in fact, data in the Mat
  if(tmp.data) {
    /*#ifdef DISPLAY
    imshow("original", tmp);
    #endif*/

    Mat hsv, upperRed, lowerRed, final;
    cvtColor(tmp, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(0, 75, 160), Scalar(35, 255, 255), lowerRed);
    inRange(hsv, Scalar(170, 75, 160), Scalar(179, 255, 255), upperRed);
    addWeighted(lowerRed, 1.0, upperRed, 1.0, 0.0, final);

    /*    split(tmp, colors);
#ifdef DISPLAY
    imshow("colors", colors[2]);
    #endif*/
    /*#ifdef DISPLAY
    imshow("red hue", final);
    #endif*/
#ifdef PRINT
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Split: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
    begin = end;
#endif

    //threshold(colors[2], edges, graythresh, 255, THRESH_BINARY);

    GaussianBlur(final, edges, Size(7,7), 1.5, 1.5);

    Mat element = getStructuringElement(MORPH_RECT, Size(2*erosionsize+1,
							 2*erosionsize+1),
					Point(erosionsize, erosionsize));
    erode(edges, edges, element);

#ifdef DISPLAY
    imshow("thresh", edges);
#endif
	
#ifdef PRINT
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Preprocessing: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
    begin = end;
#endif

    findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

#ifdef PRINT
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Found contours: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
    begin = end;
#endif
	
    std::vector<std::vector<Point> > targets;
    std::vector<std::vector<Point> > blobs;
    Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
    for( int i = 0; i<contours.size(); i++ ) {
      if(contourArea(contours[i]) < 200) continue;
      std::vector<Point> approx;
      approxPolyDP(contours[i], approx, arcLength(Mat(contours[i]), true) * 0.01, true);
	  
      int vtc = approx.size();
	  
      RotatedRect r = minAreaRect(approx);
      double blobsize = contourArea(approx);
      double box = r.size.width * r.size.height;

      if (blobsize / box > minratio && blobsize / box < maxratio && vtc < maxedgecount && box > 100) {
	blobs.push_back(approx);
	Point2f verts [4];
	r.points(verts);
	for(int y=0; y<4; y++) {
	  line(drawing, verts[y], verts[(y+1)%4], Scalar(255,0,0));
	}
	Scalar color = Scalar(0,0,255);
	std::vector<std::vector<Point> > cons;
	cons.push_back(approx);
	drawContours( drawing, cons, 0, color, 2, 8, hierarchy, 0, Point() );
      }

      else {
	Scalar color = Scalar(0,255,0);
	std::vector<std::vector<Point> > cons;
	cons.push_back(approx);
	drawContours( drawing, cons, 0, color, 2, 8, hierarchy, 0, Point() );	      
      }
    }

    double pitch = 0.0;
    double yaw = 0.0;
    double distance = 0.0;

    if (blobs.size() > 0) {
      std::sort(blobs.begin(), blobs.end(), compareArea);
      std::vector<Point> target = blobs[0];
      Moments mu;
      mu = moments(target, false);
      Point2f center = Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
      circle(drawing, center, 4, Scalar(255, 0, 255), -1, 8, 0);

      RotatedRect r = minAreaRect(target);
      yaw = center.x - tmp.cols/2;
      pitch = center.y - tmp.rows/2;
      distance = 1000 / sqrt(r.size.width * r.size.height);
#ifdef PRINT
      std::cout << center.x << std::endl;
#endif
    }

    /*lateralMtx.lock();
    lateral = -lat;
    lateralMtx.unlock();*/

    dataMtx.lock();
    data.pitch = pitch;
    data.yaw = yaw;
    data.distance = distance;
    dataMtx.unlock();

#ifdef DISPLAY
    imshow("edges", drawing);
#endif
  }
#ifdef PRINT

  std::cout << "Final: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl << std::endl;
#endif

  // Return the point representing the estimated center of the targets in the image
  return mean;
}

void ServeRoboRIO() {
  ServeRoboRIOUtil(dataMtx, data);
}

void GrabImage() {
  VideoCapture vcap;
  while(!vcap.open(0)) {}

#ifdef PRINT
  std::cout << "Connected to camera" << std::endl;
#endif

  vcap.set(CV_CAP_PROP_BRIGHTNESS, 0.01);
  vcap.set(CV_CAP_PROP_SATURATION, 1.0);
  vcap.set(CV_CAP_PROP_CONTRAST, 1.1);

  std::cout << vcap.get(CV_CAP_PROP_SATURATION) << std::endl;
  std::cout << vcap.get(CV_CAP_PROP_CONTRAST) << std::endl;
  std::cout << vcap.get(CV_CAP_PROP_BRIGHTNESS) << std::endl;

  Mat tmp;

#ifdef PRINT
  std::chrono::high_resolution_clock::time_point begin, end;
#endif

  while(1) {
#ifdef PRINT
    begin = std::chrono::high_resolution_clock::now();
#endif

    vcap.read(tmp);
    imageMtx.lock();
    tmp.copyTo(image);
    imageMtx.unlock();
    
    freshImageMtx.lock();
    freshImage = true;
    freshImageMtx.unlock();

#ifdef PRINT
    end = std::chrono::high_resolution_clock::now();
    logFile << "Read: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
#endif

  }
}

void ProcessImage() {
  Mat tmp;
  bool fresh = false;
  int dist, lat;

#ifdef PRINT
  std::chrono::high_resolution_clock::time_point begin, end;
#endif

  while(1) {
    freshImageMtx.lock();
    fresh = freshImage;
    freshImageMtx.unlock();

    if(fresh) {
      imageMtx.lock();
      image.copyTo(tmp);
      imageMtx.unlock();

#ifdef PRINT
      begin = std::chrono::high_resolution_clock::now();
#endif

      Point2f mean = findTarget(tmp);

#ifdef PRINT
      lateralMtx.lock();
      logFile << "Lateral distance: " << lateral << std::endl;
      lateralMtx.unlock();
#endif

#ifdef DISPLAY
      imshow("tmp", tmp);
      if(waitKey(30) >= 0) {break;}
#endif
      
      freshImageMtx.lock();
      freshImage = false;
      freshImageMtx.unlock();
    }
  }
}
