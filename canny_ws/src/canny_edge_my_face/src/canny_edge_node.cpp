#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>


using namespace std;
using namespace cv_bridge;
using namespace cv;

Mat src,src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold=20;
int ratio = 3;
int kernel_size = 3;

class ImageConverter

{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  public:
  ImageConverter(): it_(nh_)
  {
    image_sub_ = it_.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb,this);
    image_pub_ = it_.advertise("/image_converter/raw_image",1);

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {

      cv_bridge::CvImagePtr cv_ptr;
      namespace enc = sensor_msgs::image_encodings;

      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      src=cv_ptr->image;
      cvtColor(cv_ptr->image,src_gray,COLOR_BGR2GRAY);

      blur( src_gray, detected_edges, Size(3,3) );
      /// Canny detector
      Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

      dst = Scalar::all(0);

      src.copyTo( dst, detected_edges);
      sensor_msgs::ImagePtr msg2= CvImage(std_msgs::Header(),"bgr8",dst).toImageMsg();
      image_pub_.publish(msg2);
      imshow("Original",src);
      imshow("Canny_edged_face",dst);

    }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "canny_edge_node");
  ImageConverter ic;

  while(ros::ok())
  {
    ros::spinOnce();
    ROS_INFO("CODE RUNNING");
    if(waitKey(10)==27)
    {
      ROS_INFO("EXITING");
      break;
    }
  }
  return 0;
}
