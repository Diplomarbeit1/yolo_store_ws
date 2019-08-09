// ROS core
#include <ros/ros.h>
//Image message
#include <sensor_msgs/Image.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//stl stuff
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tmc_vision_msgs/Detection.h>
#include <tmc_vision_msgs/DetectionArray.h>
// #include <tmc_vision_msgs/Detection>

using namespace std;
using namespace tmc_vision_msgs;
class PointCloudToImage
{
public:
  void
  cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
     ROS_INFO_STREAM("Listening for incoming data on topic" );
    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg (*cloud, yolo_image); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
    yolo_pub_.publish (yolo_image); //publish our cloud image
    //tmc_vision_msgs::Detection det;
    DetectionArray yolo_sub_detections = *ros::topic::waitForMessage<DetectionArray>(yolo_sub_topic);
    cout<<"publish to mongo";
    mongo_pub_.publish (yolo_sub_detections);
  }
  PointCloudToImage () : 
	//cloud_topic_("/head_xtion/depth_registered/points"),
	cloud_topic_("/hsrb/head_rgbd_sensor/depth_registered/rectified_points"),
	image_topic_("/yolo_store/yolo_input_image"),
	yolo_sub_topic("/yolo2_node/detections")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloudToImage::cloud_cb, this);
    yolo_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    std::string r_yt = nh_.resolveName (yolo_sub_topic);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
    ROS_INFO_STREAM("Recieving from yolo on topic " << r_yt );
  }
private:
  ros::NodeHandle nh_;
  sensor_msgs::Image yolo_image; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  std::string yolo_sub_topic;
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher yolo_pub_; //image message publisher
  ros::Publisher mongo_pub_;
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pointcloud_to_image");
  ros::init (argc, argv, "convert_pointcloud_to_image");
  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  /*
  //Loop
   while (ros::ok()) {
        cout<<"starting transform"<<endl;
        PointCloudToImage pci; //this loads up the node
	//ros::Rate rate(1);
        //rate.sleep();
   }
*/	
  return 0;
}
