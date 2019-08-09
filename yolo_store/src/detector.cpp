#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/Image.msg>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
//#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//PCL
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
// OpenCV
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include <boost/foreach.hpp>

using namespace std;
using namespace std_msgs;
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    // image_transport::ImageTransport it(n_);
    // image_transport::Publisher yolo_pub_ = it.advertise("not_used", 1);
    pub_ = n_.advertise<sensor_msgs::Image>("yolo_store/yolo_image", 1);
    sub_ = n_.subscribe("/head_xtion/depth_registered/points", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg)
  {
    cout<< "in Callback";
    cv_bridge::CvImage im;
    cv::Mat image= im.image;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr processed_cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    // cloud_(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >());
    // processed_cloud_processed_cloud_(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >());
    
     pcl::fromROSMsg(*pcl_msg, *cloud_);
    // copyPointCloud(*cloud_, *processed_cloud_);

    // image = cv::Mat(processed_cloud_->height, processed_cloud_->width, CV_8UC3);
    // for (int y = 0; y < image.rows; y++) {
    //   for (int x = 0; x < image.cols; x++) {
    //     cv::Vec3b color = cv::Vec3b(processed_cloud_->points.at(y*image.cols+x).b,
    //                                 processed_cloud_->points.at(y*image.cols+x).g,
    //                                 processed_cloud_->points.at(y*image.cols+x).r);
    //     image.at<cv::Vec3b>(cv::Point(x,y)) = color;
    //   }
    // }
    // im.image=image;
    // im.encoding = sensor_msgs::image_encodings::BGR8;
    // sensor_msgs::Image yolo_send_image = *im.toImageMsg();
   // pub_.publish(yolo_send_image);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_publish");
  SubscribeAndPublish SAPObject;
  ros::Rate rate(1);
  rate.sleep();
  ros::spin();
  return 0;
}