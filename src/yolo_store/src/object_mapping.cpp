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
#include <std_msgs/Int32.h>

// #include <tmc_vision_msgs/Detection>

using namespace std;
using namespace tmc_vision_msgs;
static int current_ind;

class PointCloudToImage
{
public:
  yolo_store_msg get_yolo_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,tmc_vision_msgs::Detection det) //,tf2_msgs::TFMessage tf_to_map)
  {
    yolo_store_msg msg;
    msg.label = det.label;
    int height4 = det.height/4;
    float dividerh = float(height4/det.height);
    int width4 = det.width/4;
    float dividerw = float(width4/det.width);

    int cx = det.x + int(det.height/2);
    int cy = det.y + int(det.height/2);
    std::pair<int,int> adress = get_valid_point(cloud,cx,cy);
    int x = adress.first;
    int y = adress.second;
    msg.x = cloud->at(x,y).x;
    msg.y = cloud->at(x,y).y;
    msg.z = cloud->at(x,y).z;

    int fact = 4;
    std::pair<int,int> adress_right = get_valid_point(cloud, cx + int(det.height/fact), cy);
    std::pair<int,int> adress_left = get_valid_point(cloud, cx - int(det.height/fact), cy);
    std::pair<int,int> adress_top = get_valid_point(cloud, cx, cy + int(det.height/fact));
    std::pair<int,int> adress_bottom = get_valid_point(cloud, cx, cy - int(det.height/fact));
    int xr = adress_right.first , yr = adress_right.second;
    int xl = adress_left.first , yl = adress_left.second;
    int xt = adress_top.first , yt = adress_top.second;
    int xb = adress_bottom.first , yb = adress_bottom.second;

    


  }
  
  
     
  //######## check if valid if not...
  std::pair<int,int> get_valid_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cld, int x, int y) 
  {
    pair<int,int> p=make_pair(x,y);
    if(isnan(cld->at(x,y).x)||isnan(cld->at(x,y).y)||isnan(cld->at(x,y).z))  return valid_point_(cld,"right",--x,++y,2,0); else return p;
  }
  //######## ... get next valid point
  std::pair<int,int> valid_point_(pcl::PointCloud<pcl::PointXYZ>::Ptr cld, string direction, int x, int y, int dist, int cnt)
  {
    int b=0;                                                //minimum distance to the edges of the image
    int d=5;                                                //distance from the center point in pixel
    std::pair<int,int> p ;                                  //vraiable that is returned at the end
    int not_valid;                                          //variable to check if point is valid 

    //######maintenance 
    int m=0;                                                //ignore nans and check the limits of the algorythm
    int print=0;                                            //print x y z of pcl
    if(m) cout<<(x)<<" "<<(y)<<" ";                         //print distance --set x/y start value to 100!
    if(print&&m) cout<<"x:"<<cld->at(x,y).x<<" y:"<<cld->at(x,y).y<<"z:"<<cld->at(x,y).z;
    if(m) cout<<"\n";
    //######maintenance 
    
    if(x<=b||x>=(cld->width-b)||y<=b||y>=(cld->height-b))   //too near to the edge
    {
      cout<<"out of bounds"<<"\n";
      return make_pair(0,0);
    }
    if(dist>(d*2))//times 2 because for every increase of the distance 2 more pixels are added per direction => so actually in 4 pixels in whole
    {
      cout<<"too far away"<<"\n";
      return make_pair(0,0);
    }
    if(isnan(cld->at(x,y).x)||isnan(cld->at(x,y).y)||isnan(cld->at(x,y).z)) not_valid=1; else not_valid=m;
    //not_valid=1;// no limits
    if(direction=="right")
    {
      if(not_valid)
      {
        if(cnt<dist) 
        {
          p = valid_point_(cld,"right",++x,y,dist,++cnt);   //moves 1 to the right and increases cnt
        } 
        else valid_point_(cld,"down",x,--y,dist,1);         //already moves 1 down therefore cnt=1
      }
      else p = make_pair(x, y); 
    }
    else if(direction=="down")
    {
      if(not_valid)
      {
        if(cnt<dist) 
        {
          p = valid_point_(cld,"down",x,--y,dist,++cnt);
        } 
        else valid_point_(cld,"left",--x,y,dist,1);
      }
      else p = make_pair(x, y); 
    }
    else if(direction=="left")
    {
      if(not_valid)
      {
        if(cnt<dist) 
        {
          p = valid_point_(cld,"left",--x,y,dist,++cnt);
        } 
        else valid_point_(cld,"up",x,++y,dist,1);
      }
      else p = make_pair(x, y);
    }
    else if(direction=="up")
    {
      if(not_valid)
      {
        if(cnt<(dist-1))
        {
          p= valid_point_(cld,"up",x,++y,dist,++cnt);
        } 
        else 
        {
          dist=dist+2;
          y++;
          cout<<"\n\nrestart";
          valid_point_(cld,"right",--x,++y,dist,0);
        }
      }
      else p = make_pair(x, y);
    }
    else return p;                                          //this case can/should never occur --spelling error 
    return p;                                               //return the correct value of p
  }

//######transform pcl for yolo and send points to mongodb#######################################################################
  void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    current_ind = (*ros::topic::waitForMessage<std_msgs::Int32>(index_topic_,ros::Duration(2))).data;
    int timer_=0;
    ros::WallTime begin_;
    //###########timer###########
    begin_ = ros::WallTime::now();
    if(timer_){
    cout<<"\n\n Starting timer\n";
    }//###########timer###########

    current_ind = (*ros::topic::waitForMessage<std_msgs::Int32>(index_topic_,ros::Duration(2))).data;
    //###########timer###########
     if(timer_){
    cout<<"tf after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########

    //nh_.subscribe (tf_sub_topic, 1,&PointCloudToImage::tf_cb, this);
    tf2_msgs::TFMessage tf_saved = *ros::topic::waitForMessage<tf2_msgs::TFMessage>(tf_sub_topic,ros::Duration(2));
    //int current_max_id = *ros::topic::waitForMessage<int32>(index_topic_);
    //cout<<current_max_id;
    //###########timer###########
     if(timer_){
    cout<<"tf after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########

    //convert pcl to image
    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg (*cloud, yolo_image); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: \n" << e.what());
    }
    //###########timer###########
     if(timer_){
    cout<<"pcl after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########

    //publish image to yolo
    yolo_pub_.publish (yolo_image);
    pcl_pub_.publish(cloud);

    //###########timer###########
    if(timer_){
    cout<<"pub yolo and pcl after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########

    //recieve bounding box and name from yolo
    DetectionArray yolo_sub_detections = *ros::topic::waitForMessage<DetectionArray>(yolo_sub_topic_,ros::Duration(2));
    
    //###########timer###########
    if(timer_){
    if(yolo_sub_detections.detections.size()>0) cout<<"recieve "<<yolo_sub_detections.detections.size()<<" Objects after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########

    //transforming yolo_msg to mongo_msg;
    yolo_store_msg_Array mongo_detections;
    mongo_detections.header = cloud->header;
    mongo_detections.msgs.resize(yolo_sub_detections.detections.size());

    //convert msg pcl to xyz pcl
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    for (int i=0;i<yolo_sub_detections.detections.size();i++)
    {

      mongo_detections.msgs[i] = get_yolo_detection(temp_cloud,yolo_sub_detections.detections[i]);

    }

    // //###########timer###########
    // if(timer_){
    // end_ =  ros::WallTime::now();
    // elapsed_secs = (end_ - begin_).toNSec() * 1e-9;
    // if(yolo_sub_detections.detections.size()>0) cout<<"transforming loop starts after "<<elapsed_secs<< " seconds\n";
    // }//###########timer###########
    
   

    //###########timer###########
    if(timer_){
    if(yolo_sub_detections.detections.size()>0) cout<<"transform to mongo_msg for loop after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########

    //publish to mongo
    mongo_pub_.publish (mongo_detections);

    //###########timer###########
    if(timer_){
    if(yolo_sub_detections.detections.size()>0) cout<<"pub to mongo after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########

    cout<<"finsihed and resting now \n";

  }
  
  PointCloudToImage () : 
	cloud_topic_("/head_xtion/depth_registered/points"),
	//cloud_topic_("/hsrb/head_rgbd_sensor/depth_registered/rectified_points"),
	image_topic_("/yolo_store/yolo_input_image"),
  mongo_topic_("/yolo_store/msg_mongo"),
  index_topic_("/yolo_store/max_index"),
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
    std::string r_id = nh_.resolveName (index_topic_);
    

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
  std::string index_topic_;
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
  ros::spin (); 

//   //Loop
//    while (ros::ok()) {
//         cout<<"starting transform"<<endl;
//         PointCloudToImage pci; //this loads up the node
// 	//ros::Rate rate(1);
//         //rate.sleep();
//    }
  return 0;
}
