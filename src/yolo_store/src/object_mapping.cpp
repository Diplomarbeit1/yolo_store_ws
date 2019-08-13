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
#include <tf2_msgs/TFMessage.h>
#include <tmc_vision_msgs/yolo_store_msg_Array.h>
#include <tmc_vision_msgs/yolo_store_msg.h>

// #include <tmc_vision_msgs/Detection>

using namespace std;
using namespace tmc_vision_msgs;
class PointCloudToImage
{
public:
  void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    ros::WallTime begin_, end_;
    begin_ = ros::WallTime::now();
    cout<<"\n\n Starting timer\n";
    //nh_.subscribe (tf_sub_topic, 1,&PointCloudToImage::tf_cb, this);
    tf2_msgs::TFMessage tf_saved = *ros::topic::waitForMessage<tf2_msgs::TFMessage>(tf_sub_topic);
    end_ =  ros::WallTime::now();
   
    double elapsed_secs = (end_ - begin_).toNSec() * 1e-9;
    cout<<"tf after "<<elapsed_secs<< " seconds\n";
    //cout<<"Listening for incoming pcl \n";
    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg (*cloud, yolo_image); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: \n"
                        << e.what());
    }

    end_ =  ros::WallTime::now();
    elapsed_secs = (end_ - begin_).toNSec() * 1e-9;
    cout<<"pcl after "<<elapsed_secs<< " seconds\n";

    //std::cout<<"\n"<< "publish to yolo \n";
    yolo_pub_.publish (yolo_image); //publish our cloud image
    pcl_pub_.publish(cloud);
    end_ =  ros::WallTime::now();
    elapsed_secs = (end_ - begin_).toNSec() * 1e-9;
    cout<<"pub yolo and pcl after "<<elapsed_secs<< " seconds\n";
    
    DetectionArray yolo_sub_detections = *ros::topic::waitForMessage<DetectionArray>(yolo_sub_topic_);
    end_ =  ros::WallTime::now();
    elapsed_secs = (end_ - begin_).toNSec() * 1e-9;
    if(yolo_sub_detections.detections.size()>0) cout<<"recieve "<<yolo_sub_detections.detections.size()<<" Objects after "<<elapsed_secs<< " seconds\n";
    
    //cout<<"transforming yolo_msg to mongo_msg \n";
    yolo_store_msg_Array mongo_detections;
    mongo_detections.header = yolo_sub_detections.header;
    mongo_detections.tf = tf_saved;
    mongo_detections.msgs.resize(yolo_sub_detections.detections.size());
    end_ =  ros::WallTime::now();
    elapsed_secs = (end_ - begin_).toNSec() * 1e-9;
    if(yolo_sub_detections.detections.size()>0) cout<<"transforming loop starts after "<<elapsed_secs<< " seconds\n";

    for (int i=0;i<yolo_sub_detections.detections.size();i++)
    {
          mongo_detections.msgs[i].detections=yolo_sub_detections.detections[i];
    }

    end_ =  ros::WallTime::now();
    elapsed_secs = (end_ - begin_).toNSec() * 1e-9;
    if(yolo_sub_detections.detections.size()>0) cout<<"transform to mongo_msg for loop after "<<elapsed_secs<< " seconds\n";

    cout<<"publish to mongo \n";
    mongo_pub_.publish (mongo_detections);

    end_ =  ros::WallTime::now();
    elapsed_secs = (end_ - begin_).toNSec() * 1e-9;
    if(yolo_sub_detections.detections.size()>0) cout<<"pub to mongo after "<<elapsed_secs<< " seconds\n";

    cout<<"finsihed and resting now \n";
  }
  PointCloudToImage () : 
	//cloud_topic_("/head_xtion/depth_registered/points"),
	cloud_topic_("/hsrb/head_rgbd_sensor/depth_registered/rectified_points"),
	image_topic_("/yolo_store/yolo_input_image"),
  mongo_topic_("/yolo_store/msg_mongo"),
  pcl_topic_("/yolo_store/pcl_mongo"),
  yolo_sub_topic_("/yolo2_node/detections"),
  tf_sub_topic("tf")

  {
    sub_ = nh_.subscribe (cloud_topic_, 1, &PointCloudToImage::cloud_cb, this);
    yolo_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 1);
    mongo_pub_ = nh_.advertise<yolo_store_msg_Array> (mongo_topic_,1);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl_topic_,1);
    x=5;
    //info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    std::string r_yt = nh_.resolveName (yolo_sub_topic_);
    std::string r_pt = nh_.resolveName (pcl_topic_);
    std::string r_mt = nh_.resolveName (mongo_topic_);
    std::string r_tf = nh_.resolveName (tf_sub_topic);

    ROS_INFO_STREAM("Listening for incoming tf on topic " << r_tf );
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
    ROS_INFO_STREAM("Recieving from yolo on topic " << r_yt );
    ROS_INFO_STREAM("Publish point cloud to " << r_pt );
    ROS_INFO_STREAM("Publish mongodb_message to " << r_mt );
  }
private:
  ros::NodeHandle nh_;
  sensor_msgs::Image yolo_image; //cache the image message
  //geometry_msgs::TransformStamped tf_saved;

  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  std::string mongo_topic_;
  std::string pcl_topic_;
  std::string yolo_sub_topic_;
  std::string tf_sub_topic;
  ros::Subscriber sub_; 
  ros::Publisher yolo_pub_; 
  ros::Publisher mongo_pub_;
  ros::Publisher pcl_pub_;
  int x;
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "object_mapping");
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
