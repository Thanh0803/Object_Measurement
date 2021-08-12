#include <iostream>
#include <sstream>
#include <stdio.h>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <thread>
#include <pthread.h>
#include <unistd.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <atomic>
#include <mutex>
#include <vector>
#include <utility>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <arpa/inet.h>
#include <fstream>
#include <sys/time.h>
#include <experimental/filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuple>
#include <string>
#include <cstdlib>


using namespace cv;
using namespace std;
#define pdd pair<int, int>
rs2_intrinsics intr;
Mat color_line;

vector<Point> canny_out;
vector<Point> colected;
vector<Point> colected2;
vector<Vec4i> lines;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy; 
vector<Rect> bbox;
Vec4i max_l = 0;
vector<String> filenames;
vector<Mat> images;
vector<String> fn;
vector<String> fn1;


Mat crop_Image, canny_output, nonzeroCoordinates, depth_thresh;
Mat img, depth_gray, RGB;


int j, indi, indi2, indi3;
float _x1,_y1,_x2,_y2,_u1,_v1,_u2,_v2;

void readxml(Mat &depth_thresh, int &i)
{
  // string filename = "/home/thanhngauxi/Desktop/opencv/Do_Vat/New_Data/depth/depth_0002.xml";
  FileStorage fs;
  fs.open(fn1[i], FileStorage::READ);
  fs["depth_thresh"] >> depth_thresh;
  fs.release();
}

Mat houghline(Mat crop_Image)
{
  blur(crop_Image, crop_Image, Size(3,3));
  Canny(crop_Image, canny_output, 50, 300, 3);
  findNonZero(canny_output, nonzeroCoordinates);
  for (int i=0; i< nonzeroCoordinates.total();i++)
  {
    canny_out.push_back(nonzeroCoordinates.at<Point>(i));
  }
  HoughLinesP(canny_output, lines, 1, CV_PI/180, 50, 50, 10 );
  return canny_output;
}

int perpendicular_line( float a, float b, float e, float f)
{
  float c;
  if(a==0 && b !=0)
  {
    c = -b*f;
  }
  else if(a != 0 && b ==0)
  {
    c = -a*e;
  }
  else 
  {
    c = -(a*e +b*f);
  }
  return c;
}

int draw_width_line (float a, float b, float c, float e, float f ,vector<Point> &canny_out)
{
  for (int i=0; i<canny_out.size(); i++)
  {
    float d = (float) abs((a*canny_out[i].x + b*canny_out[i].y +c)/sqrt(a*a+b*b));
    if (d<1)
    {
      colected.push_back(canny_out[i]);
    }

  }
  int max = -1;
  for (int i=0; i<colected.size();i++)
  {
    float distance = (float)sqrt((pow((float)(colected[i].x-e),2))+pow((float)(colected[i].y-f),2));
    if(max < distance)
    {
      max=distance;
      indi = i;
    }
  }
  line(crop_Image, Point(e,f), Point(colected[indi].x, colected[indi].y), Scalar(0,255,0),3, CV_AA);
  return indi;
}

Mat draw_length_line (float aa, float bb, float cc, float ee, float ff, vector<Point> &canny_out)
{
  for (int i=0; i<canny_out.size(); i++)
  {
    float dd = (float) abs(((aa*canny_out[i].x + bb*canny_out[i].y +cc))/sqrt(aa*aa + bb*bb));
    if(dd<1)

    {
      colected2.push_back(canny_out[i]);
    }
  }
  for (int i=0; i< colected2.size(); i++)
  {
    if ( colected2[i].x == ee && colected2[i].y < ff )
    {
      float max =0;
      float distance = sqrt(pow((colected2[i].x -ee),2)+pow((colected2[i].y - ff),2));
      if (distance > max)
      {
        max = distance;
        indi2 =i;
      }
    }
    if ( colected2[i].x == ee && colected2[i].y>ff)
    {
      float maxx=0;
      float distance_1 = sqrt(pow((colected2[i].x -ee),2)+pow((colected2[i].y - ff),2));
      if (distance_1 > maxx)
      {
        maxx = distance_1;
        indi3 = i;
      }
    }
    if ( colected2[i].x < ee && colected2[i].y == ff )
    {
      float max =0;
      float distance = sqrt(pow((colected2[i].x -ee),2)+pow((colected2[i].y - ff),2));
      if ( distance > max)
      {
        max = distance;
        indi2 = i;
      }
    }
    if (colected2[i].x > ee && colected2[i].y == ff)
    {
      float maxx =0;
      float distance_1 = sqrt(pow((colected2[i].x -ee),2)+pow((colected2[i].y - ff),2));
      if(distance_1 >maxx)
      {
        maxx = distance_1;
        indi3 =i;
      }
    }
    if (colected2[i].x > ee && colected2[i].y > ff)
    {
      float max =0;
      float distance = sqrt(pow((colected2[i].x -ee),2)+pow((colected2[i].y - ff),2));
      if(distance > max )
      {
        max = distance;
        indi2 =i;
      }
    }
    if (colected2[i].x < ee && colected2[i].y < ff)
    {
      float maxx =0;
      float distance_1 = sqrt(pow((colected2[i].x -ee),2)+pow((colected2[i].y - ff),2));
      if(distance_1 > maxx)
      {
        maxx = distance_1;
        indi3 =i;
      }
    }
    if (colected2[i].x > ee && colected2[i].y < ff)
    {
      float max =0;
      float distance = sqrt(pow((colected2[i].x -ee),2)+pow((colected2[i].y - ff),2));
      if(distance >max)
      {
        max = distance;
        indi2 =i;
      }
    }
    if (colected2[i].x < ee && colected2[i].y > ff)
    {
      float maxx =0;
      float distance_1 = sqrt(pow((colected2[i].x -ee),2)+pow((colected2[i].y - ff),2));
      if(distance_1 >maxx)
      {
        maxx = distance_1;
        indi3 =i;
      }
    }
  }
  line(crop_Image, Point(ee, ff), Point(colected2[indi2].x, colected2[indi2].y), Scalar(0,0,255),5, CV_AA);
  line(crop_Image, Point(ee, ff), Point(colected2[indi3].x, colected2[indi3].y), Scalar(0,0,255),5, CV_AA);
    return crop_Image;
}
float calculate_Width_Auto_vector(int x1, int y1, int x2, int y2)
{
  Vec4i Line_AB;
  Line_AB[0] = x1;
  Line_AB[1] = y1;
  Line_AB[2] = x2;
  Line_AB[3] = y2;
  // float AB = DistanceLine(Line_AB); //tinh doan AB
  float AB = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
  float cos_alpha = abs(y2 - y1) / AB;
  float sin_alpha = abs(x2 - x1) / AB;
  float upixel_cal[2];
  float vpixel_cal[2]; // the hien toa do pixel 2D
  float upoint_cal[3];
  float vpoint_cal[3];

  int i;
  float x1_ = x1;
  float x2_ = x2;
  float y1_ = y1;
  float y2_ = y2;
  upixel_cal[0] = x1;
  upixel_cal[1] = y1;
  vpixel_cal[0] = x2;
  vpixel_cal[1] = y2;
  // if (x1 < 0 || x1 > 1279 || x2 < 0 || x2 > 1279 ||y1 <0 || y1 > 719 || y2 < 0 || y2 > 719)
  // {
  //  return 0;
  // }
  auto udist_1 = depth_thresh.at<double>(y1, x1); // doc file xml, lay ra depth owr toa do (x1,y1)
  auto udist_2 = depth_thresh.at<double>(y2, x2);

  while (1)
  {
    // cout << x1_ << "--" <<y1_ << "--"<<x2_ <<"--"<<y2_ <<endl;
    if (x1_ < 0 || x1_ > 1279 || x2_ < 0 || x2_ > 1279 || y1_ < 0 || y1_ > 719 || y2_ < 0 || y2_ > 719)
    {
      break;
    }
    circle(color_line, Point(x1_, y1_), 2, Scalar(0, 255, 0), -1, 2, 0); // ve cac diem noi suy tren anh màu
    circle(color_line, Point(x2_, y2_), 2, Scalar(0, 255, 0), -1, 2, 0); // ve cac diem noi suy tren anh màu
    auto udist_new_1 = depth_thresh.at<double>(y1_, x1_);        // x_ de thoat khoi vong lap gtri ko thay doi.
    auto udist_new_2 = depth_thresh.at<double>(y2_, x2_);
    if (udist_new_1 != 0 && udist_new_2 != 0)
    {
      break;
    }
    // cout <<"log-------633"<<endl;
    if (udist_new_1 == 0)
    {
      // cout << "yyyyy" <<endl;
      // cout << x1 << "--" <<y1 << "--"<<x2 <<"--"<<y2 <<endl;
      if (x1 <= x2 && y1 < y2)
      {
        x1_ = x1_ + sin_alpha; // vi sao cong vs luong ntn?
        y1_ = y1_ + cos_alpha;
        // cout << x1_ << "--" <<y1_ <<endl;
      }
      if (x1 >= x2 && y1 > y2)
      {
        x1_ = x1_ - sin_alpha;
        y1_ = y1_ - cos_alpha;
        // cout << x1_ << "--" <<y1_ <<endl;
      }
      if (x1 < x2 && y1 >= y2)
      {
        x1_ = x1_ + sin_alpha;
        y1_ = y1_ - cos_alpha;
        // cout << x1_ << "--" <<y1_ <<endl;
      }
      if (x1 > x2 && y1 <= y2)
      {
        x1_ = x1_ - sin_alpha;
        y1_ = y1_ + cos_alpha;
        // cout << x1_ << "--" <<y1_ <<endl;
      }
    }
    if (udist_new_2 == 0)
    {
      // cout << "xxxx" <<endl;
      if (x2 <= x1 && y2 < y1)
      {
        x2_ = x2_ + sin_alpha;
        y2_ = y2_ + cos_alpha;
        // cout << x2_ << "--" <<y2_ <<endl;
      }
      if (x2 >= x1 && y2 > y1)
      {
        x2_ = x2_ - sin_alpha;
        y2_ = y2_ - cos_alpha;
        // cout << x2_ << "--" <<y2_ <<endl;
      }
      if (x2 < x1 && y2 >= y1)
      {
        x2_ = x2_ + sin_alpha;
        y2_ = y2_ - cos_alpha;
        // cout << x2_ << "--" <<y2_ <<endl;
      }
      if (x2 > x1 && y2 <= y1)
      {
        x2_ = x2_ - sin_alpha;
        y2_ = y2_ + cos_alpha;
        // cout << x2_ << "--" <<y2_ <<endl;
      }
    }
  }
  // cout << "thoattttttttttttttttttttttttttttttttttttttttt" << endl;
  float length_moi;
  if (x1_ < 0 || x1_ > 1279 || x2_ < 0 || x2_ > 1279 || y1_ < 0 || y1_ > 719 || y2_ < 0 || y2_ > 719)
  {
    length_moi = 0;
  }
  else
  {
    // cout <<"log-------695"<<endl;
    auto udist_new_1 = depth_thresh.at<double>(y1_, x1_); // y1,x1 co tu dua la int???
    auto udist_new_2 = depth_thresh.at<double>(y2_, x2_);
    // cout << "udist_new_1: " << udist_new_1 << " --udist_new_2: " << udist_new_2 <<endl;
    rs2_deproject_pixel_to_point(upoint_cal, &intr, upixel_cal, udist_new_1); // qui doi toa do 2D--->3D.
    rs2_deproject_pixel_to_point(vpoint_cal, &intr, vpixel_cal, udist_new_2); // upoint_cal: toa do 3D; upixel_call: toa do 2D; &intr: matran cameralirbrary
    // cout <<"log-------702"<<endl;
    length_moi = sqrt(pow(upoint_cal[0] - vpoint_cal[0], 2) + pow(upoint_cal[1] - vpoint_cal[1], 2) + pow(upoint_cal[2] - vpoint_cal[2], 2));
  } // do dai 3D
  return length_moi;
}
float calculate_Height_Auto_vector( int x1, int y1, int x2, int y2)
{
  Vec4i Line_AB;
  Line_AB[0] = x1;
  Line_AB[1] = y1;
  Line_AB[2] = x2;
  Line_AB[3] = y2;
  // float AB = DistanceLine(Line_AB);
  float AB = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
  float cos_alpha = abs(y2 - y1) / AB;
  float sin_alpha = abs(x2 - x1) / AB;
  float upixel_cal[2];
  float vpixel_cal[2];
  float upoint_cal[3];
  float vpoint_cal[3];

  int i;
  float x1_ = x1;
  float x2_ = x2;
  float y1_ = y1;
  float y2_ = y2;

  float moving_x1 = 0;
  float moving_y1 = 0;
  float moving_x2 = 0;
  float moving_y2 = 0;
  upixel_cal[0] = x1;
  upixel_cal[1] = y1;
  vpixel_cal[0] = x2;
  vpixel_cal[1] = y2;
  if (x1 < 0 || x1 > 1279 || x2 < 0 || x2 > 1279 || y1 < 0 || y1 > 719 || y2 < 0 || y2 > 719)
  {
    return 0;
  }

  auto udist_1 = depth_thresh.at<double>(y1, x1);

  auto udist_2 = depth_thresh.at<double>(y2, x2);
  while (1)
  {
    // cout << x1_ << "--" <<y1_ << "--"<<x2_ <<"--"<<y2_ <<endl;
    if (x1_ < 0 || x1_ > 1279 || x2_ < 0 || x2_ > 1279 || y1_ < 0 || y1_ > 719 || y2_ < 0 || y2_ > 719)
    {
      break;
    }
    circle(color_line, Point(x1_, y1_), 2, Scalar(0, 255, 0), -1, 2, 0); // ve cac diem noi suy tren anh màu
    circle(color_line, Point(x2_, y2_), 2, Scalar(0, 255, 0), -1, 2, 0); // ve cac diem noi suy tren anh màu
    auto udist_new_1 = depth_thresh.at<double>(y1_, x1_);
    auto udist_new_2 = depth_thresh.at<double>(y2_, x2_);
    if (udist_new_1 != 0 && udist_new_2 != 0)
    {
      break;
    }
    // cout <<"log-------633"<<endl;
    if (udist_new_1 == 0)
    {
      // cout << "yyyyy" <<endl;
      // cout << x1 << "--" <<y1 << "--"<<x2 <<"--"<<y2 <<endl;
      if (x1 <= x2 && y1 < y2)
      {
        moving_x1 = moving_x1 + sin_alpha;
        moving_y1 = moving_y1 + cos_alpha;
        x1_ = x1_ + sin_alpha;
        y1_ = y1_ + cos_alpha;
        // cout << x1_ << "--" <<y1_ <<endl;
      }
      if (x1 >= x2 && y1 > y2)
      {
        moving_x1 = moving_x1 - sin_alpha;
        moving_y1 = moving_y1 - cos_alpha;
        x1_ = x1_ - sin_alpha;
        y1_ = y1_ - cos_alpha;
        // cout << x1_ << "--" <<y1_ <<endl;
      }
      if (x1 < x2 && y1 >= y2)
      {
        moving_x1 = moving_x1 + sin_alpha;
        moving_y1 = moving_y1 - cos_alpha;
        x1_ = x1_ + sin_alpha;
        y1_ = y1_ - cos_alpha;
        // cout << x1_ << "--" <<y1_ <<endl;
      }
      if (x1 > x2 && y1 <= y2)
      {
        moving_x1 = moving_x1 - sin_alpha;
        moving_y1 = moving_y1 + cos_alpha;
        x1_ = x1_ - sin_alpha;
        y1_ = y1_ + cos_alpha;
        // cout << x1_ << "--" <<y1_ <<endl;
      }
    }
    if (udist_new_2 == 0)
    {
      // cout << "xxxx" <<endl;
      if (x2 <= x1 && y2 < y1)
      {
        moving_x2 = moving_x2 + sin_alpha;
        moving_y2 = moving_y2 + cos_alpha;
        x2_ = x2_ + sin_alpha;
        y2_ = y2_ + cos_alpha;
        // cout << x2_ << "--" <<y2_ <<endl;
      }
      if (x2 >= x1 && y2 > y1)
      {
        moving_x2 = moving_x2 - sin_alpha;
        moving_y2 = moving_y2 - cos_alpha;
        x2_ = x2_ - sin_alpha;
        y2_ = y2_ - cos_alpha;
        // cout << x2_ << "--" <<y2_ <<endl;
      }
      if (x2 < x1 && y2 >= y1)
      {
        moving_x2 = moving_x2 + sin_alpha;
        moving_y2 = moving_y2 - cos_alpha;
        x2_ = x2_ + sin_alpha;
        y2_ = y2_ - cos_alpha;
        // cout << x2_ << "--" <<y2_ <<endl;
      }
      if (x2 > x1 && y2 <= y1)
      {
        moving_x2 = moving_x2 - sin_alpha;
        moving_y2 = moving_y2 + cos_alpha;
        x2_ = x2_ - sin_alpha;
        y2_ = y2_ + cos_alpha;
        // cout << x2_ << "--" <<y2_ <<endl;
      }
    }
  }
  // cout << "thoattttttttttttttttttttttttttttttttttttttttt" << endl;
  float length_moi;
  if (x1_ < 0 || x1_ > 1279 || x2_ < 0 || x2_ > 1279 || y1_ < 0 || y1_ > 719 || y2_ < 0 || y2_ > 719)
  {
    length_moi = 0;
  }
  else
  {
    // cout <<"log-------695"<<endl;
    auto udist_new_1 = depth_thresh.at<double>(y1_, x1_) + abs(depth_thresh.at<double>(y1_ + moving_y1, x1_ + moving_x1) - depth_thresh.at<double>(y1_, x1_));
    auto udist_new_2 = depth_thresh.at<double>(y2_, x2_) + abs(depth_thresh.at<double>(y2_ + moving_y2, x2_ + moving_x2) - depth_thresh.at<double>(y2_, x2_));
    // cout << "udist_new_1: " << udist_new_1 << " --udist_new_2: " << udist_new_2 <<endl;
    // cout << "xxx:----" << abs(udist_new_1 - udist_new_2) << endl;
    rs2_deproject_pixel_to_point(upoint_cal, &intr, upixel_cal, udist_new_1);
    rs2_deproject_pixel_to_point(vpoint_cal, &intr, vpixel_cal, udist_new_2);
    // cout <<"log-------702"<<endl;
    length_moi = sqrt(pow(upoint_cal[0] - vpoint_cal[0], 2) + pow(upoint_cal[1] - vpoint_cal[1], 2) + pow(upoint_cal[2] - vpoint_cal[2], 2));
  }
  return length_moi;
}


//////////////////////////////////////////////////////////////////////////////////////
bool leftDown=false,leftup=false;
std::vector<std::pair<int, int>> cor1;
std::vector<std::pair<int, int>> cor2;

void mouse_call (int event, int x, int y, int flags, void* param)
{
  cout << "x" <<endl;
	if(event==CV_EVENT_LBUTTONDBLCLK && leftDown==false && leftup==false )
	{
		leftDown=true;
		cor1.push_back(make_pair(x, y));

	}
	else if(event==CV_EVENT_LBUTTONDBLCLK && leftDown==true&& leftup == false)
	{

		leftup=true;
		cor2.push_back(make_pair(x, y));

	}

	if(leftDown==true&&leftup==false) //when the left button is down
	{
		Point pt;
		pt.x=x;
		pt.y=y;
		Mat temp_img=RGB.clone();

		line(temp_img, Point(cor1[0].first, cor1[0].second), Point(pt.x, pt.y), Scalar(255,0,0), 1.5, CV_AA);
		imshow("Color",temp_img);

	}
	if(leftDown==true&&leftup==true) //when the selection is done
	{
          int distance = abs(cor1[0].first - cor2[0].first);
          if (distance > 200)
          {
               line(RGB, Point(cor1[0].first, cor1[0].second), Point(cor2[0].first, cor2[0].second), Scalar(255,0,0), 1.5, CV_AA);
               float Measured= calculate_Width_Auto_vector(cor1[0].first, cor1[0].second, cor2[0].first, cor2[0].second);
               imshow("Color", RGB);
               cout << "Measured Size Click: "<< Measured*100 <<" (cm)" <<endl;
          }
          else
          {
               line(RGB, Point(cor1[0].first, cor1[0].second), Point(cor2[0].first, cor2[0].second), Scalar(255,0,0), 1.5, CV_AA);
               float Measured= calculate_Height_Auto_vector(cor1[0].first, cor1[0].second, cor2[0].first, cor2[0].second);
               imshow("Color", RGB);
               cout << "Measured Size Click: "<< Measured*100 <<" (cm)" <<endl;
          } 
		waitKey(0);
		
		leftDown=false;
		leftup=false;
		cor1.clear();
		cor2.clear();

	}
}

/////////////////////////////////////////////////////////////////////////////////////////


Mat contourss(Mat depth_gray, Mat RGB)
{
    findContours( depth_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<Rect> boundRect(contours.size());
    Mat drawing = Mat::zeros( depth_gray.size(), CV_8UC3 ); 

    for(int i=0;i<contours.size();i++)
    { 
      float size=contourArea(contours[i]);

        if(size>10000)
          {
            bbox.push_back(boundingRect(Mat(contours[i])));
            Scalar color( rand()&255, rand()&255, rand()&255 );
            drawContours(drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
            for ( j=0; j<bbox.size(); j++)
            {
              RGB(Rect(Point(bbox[j].tl().x - 10, bbox[j].tl().y - 10), Point(bbox[j].br().x + 10,bbox[j].br().y + 10))).copyTo(crop_Image);
              canny_output = houghline(crop_Image);
              double max_dist = 0;

            for( size_t i = 0; i < lines.size(); i++ )
            {
                Vec4i l = lines[i];
                double theta1,theta2, hyp, result;

                theta1 = (l[3]-l[1]);
                theta2 = (l[2]-l[0]);

                hyp = hypot(theta1,theta2);

                if (max_dist < hyp) 
                {
                    max_l = l;
                    max_dist = hyp;  
                }
            }
            line( crop_Image, Point(max_l[0], max_l[1]), Point(max_l[2], max_l[3]), Scalar(255,0,0), 3, CV_AA); 

            pdd P = make_pair((int)(max_l[2]+max_l[0])/2, (int)(max_l[1]+max_l[3])/2);
            float a = max_l[2]-max_l[0];
            float b = max_l[3]-max_l[1];
            float c = perpendicular_line(a,b, P.first, P.second);

            float indi = draw_width_line(a,b,c,P.first, P.second, canny_out);
            pdd Q = make_pair((int)(P.first+colected[indi].x)/2, (int)(P.second+colected[indi].y)/2);
            float aa = P.first - colected[indi].x;
            float bb = P.second - colected[indi].y;
            float cc = perpendicular_line(aa, bb, Q.first, Q.second);

            Mat crop_Image = draw_length_line (aa,bb,cc, Q.first, Q.second, canny_out);
            _x1= colected2[indi2].x + bbox[j].tl().x-10;
            _y1= colected2[indi2].y + bbox[j].tl().y-10;
            _x2= colected2[indi3].x + bbox[j].tl().x -10;
            _y2= colected2[indi3].y + bbox[j].tl().y -10;
            _u1 = P.first + bbox[j].tl().x -10;
            _v1 = P.second + bbox[j].tl().y -10;
            _u2 = colected[indi].x +bbox[j].tl().x -10 ;
            _v2 = colected[indi].y + bbox[j].tl().y-10;

            line(RGB, Point(_x1,_y1), Point(_x2,_y2),Scalar(0,255,0), 3, CV_AA);
            line(RGB, Point(_u1,_v1), Point(_u2,_v2), Scalar(0,255,0), 3, CV_AA);

            float w = calculate_Width_Auto_vector(_x1, _y1, _x2, _y2);
            cout << "h = " << w << endl;
            float h = calculate_Height_Auto_vector(_u1, _v1, _u2, _v2);
            cout << "w = " << h << endl;
            float true_Width = 25.168/100;   // real size =  25,14 x 6,1 cm
            float true_Height = 6.16/100;

            float error_Height = (abs(true_Height - h)/true_Height)*100;
            float error_Width = (abs(true_Width - w)/true_Width)*100;
            cout << "error_Height = " << error_Height << "(%)"<< " , error_Width = " << error_Width << " (%)" << endl;
            // cout <<"xx"<<endl;

            imshow("xxx", RGB);
            waitKey(0);
            canny_out.clear();
            colected2.clear();
            colected.clear();
            lines.clear();
            bbox.erase(bbox.begin(), bbox.end());
          }
    }
}

            
}
int main(int argc, char** argv)
{
  glob("/home/thanhbear/Desktop/CV_2020/Object_Measurement/data/Deepthresh/*.png", filenames, true);
  for(int i=0; i<filenames.size();++i)
  {
    img = imread(filenames[i]);
    glob("/home/thanhbear/Desktop/CV_2020/Object_Measurement/data/RGB/*.png", fn, true);
    RGB = imread(fn[i]);
    intr.width = 1280;
    intr.height = 720;
    intr.ppx = 645.22509765625;
    intr.ppy = 346.736877441406;
    intr.fx = 918.7568359375;
    intr.fy = 916.401245117188;
    glob("/home/thanhbear/Desktop/CV_2020/Object_Measurement/data/Depth/*.xml", fn1, true);
    readxml(depth_thresh, i);
    cvtColor(img, depth_gray, CV_RGB2GRAY);                                            
    contourss(depth_gray, RGB);
    cout <<"--------------------------"<<i<<endl;
  }
    return 0;

}
