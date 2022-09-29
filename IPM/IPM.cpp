#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

double range = 10;
double gridsize = 0.05;
int L = (int) (2 * range / gridsize);

vector<Point2f> capPoints;
vector<Point2f> capPoints_ipm;

Mat dst;
Mat img;

void onMouse(int event, int x, int y, int flags, void *param) {
  switch (event) {
    case EVENT_LBUTTONDOWN:

      if (capPoints.size() < 4) {
        capPoints.push_back(Point(x, y));
        line(img, Point2f(x, 0), Point2f(x, img.rows), Scalar(0, 255, 0));
        line(img, Point2f(0, y), Point2f(img.cols, y), Scalar(0, 255, 0));
        imshow("img", img);
        std::cout << "at(" << x << "," << y << ")" << std::endl;
        cout << "capPoints size = " << capPoints.size() << endl;
      } else {
        cout << "no more!" << endl;
      }
      break;
  }
}

void onMouse_IPM(int event, int x, int y, int flags, void *param) {
  switch (event) {
    case EVENT_LBUTTONDOWN:

      if (capPoints_ipm.size() < 2) {
        capPoints_ipm.push_back(Point(x, y));
        //line(img,Point2f(x,0),Point2f(x,img.rows),Scalar(0,255,0));
        //line(img,Point2f(0,y),Point2f(img.cols,y),Scalar(0,255,0));
        circle(dst, Point(x, y), 2, Scalar(0, 255, 0));
        imshow("dst", dst);
        std::cout << "at(" << x << "," << y << ")" << std::endl;
        cout << "capPoints_ipm size = " << capPoints_ipm.size() << endl;
      }
      if (capPoints_ipm.size() == 2) {
        double dx = capPoints_ipm[1].x - capPoints_ipm[0].x;
        double dy = capPoints_ipm[1].y - capPoints_ipm[0].y;
        cout << "Length = " << gridsize * sqrt((double) (dx * dx + dy * dy)) << endl;
        capPoints_ipm.clear();
      }
  }
}

int main(int argc, char **argv) {

  if (argc < 2) {
    cout << argc << " Input parameter error." << endl;
    exit(-1);
  }
  string addr = argv[1];
  //cout << addr << endl;
  img = imread(addr);
  Mat img_proc = img.clone();

  if (img.empty()) {
    cout << "Read image error." << endl;
    exit(-1);
  }
  namedWindow("img", 1);
  setMouseCallback("img", onMouse);
  imshow("img", img);
  waitKey(0);

  cout << "The chosen points are:" << endl;
  for (auto m: capPoints)
    cout << m << endl;

  vector<Point2f> corners;

  // Change the Coordinate
  Point2f p1 = Point2f(((double) range - 1) / gridsize, (2.0 * (double) range - 3.5) / gridsize);
  Point2f p2 = Point2f(((double) range + 1) / gridsize, (2.0 * (double) range - 3.5) / gridsize);
  Point2f p3 = Point2f(((double) range + 1) / gridsize, (2.0 * (double) range - 1.5) / gridsize);
  Point2f p4 = Point2f(((double) range - 1) / gridsize, (2.0 * (double) range - 1.5) / gridsize);

  corners.push_back(p1);
  corners.push_back(p2);
  corners.push_back(p3);
  corners.push_back(p4);

  Mat tp = cv::getPerspectiveTransform(capPoints, corners);

  cout << "tp = " << endl << tp << endl;
  warpPerspective(img_proc, dst, tp, Size(L, L));

  vector<Point2f> ground_pts, image_pts;
  ground_pts.push_back(p1);
  ground_pts.push_back(p2);
  ground_pts.push_back(p3);
  ground_pts.push_back(p4);

  perspectiveTransform(ground_pts, image_pts, tp.inv());

  circle(img_proc, image_pts[0], 5, Scalar(0, 0, 255), -1);
  circle(img_proc, image_pts[1], 5, Scalar(0, 0, 255), -1);
  circle(img_proc, image_pts[2], 5, Scalar(0, 0, 255), -1);
  circle(img_proc, image_pts[3], 5, Scalar(0, 0, 255), -1);
  imshow("img_proc", img_proc);

  imshow("dst", dst);
  imwrite("output.png", dst);
  setMouseCallback("dst", onMouse_IPM);

  waitKey(0);

  return 0;
}