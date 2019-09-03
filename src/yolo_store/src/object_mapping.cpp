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

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

// #include <tmc_vision_msgs/Detection>

using namespace std;
using namespace tmc_vision_msgs;
static int current_ind;






class PointCloudToImage
{
public:
  geometry_msgs::Point yolo_transform(geometry_msgs::Point center_p, geometry_msgs::TransformStamped& tf_cam2world)
  {

        cout<<tf_cam2world.header;
    geometry_msgs::PointStamped point_in, point_out;
    point_in.point = center_p;
    point_in.header = tf_cam2world.header;
    tf2::doTransform(point_in, point_out, tf_cam2world);
    return point_out.point;
  }

  yolo_store_msg get_yolo_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,tmc_vision_msgs::Detection det, geometry_msgs::TransformStamped& tf_cam2world) 
  {
    yolo_store_msg msg;
    msg.label = det.label;
    //msg.header = cloud->header;

    int cx = round(det.x);// + int(det.height/2);
    int cy = round(det.y);// + int(det.height/2);
    std::pair<int,int> adress = get_valid_point(cloud,cx,cy);

    if(!adress.first||!adress.second)
    {

      msg.x = 0; 
      msg.y = 0; 
      msg.z = 0; 
      return msg;
    }
    int x = adress.first;
    int y = adress.second;

    int fact = 4; // division factor for assuring that the points for calculating width/height are in the object 
                  // by shrinking the bounding box by the factor/2 and then expanding it later with the factors below
    int height4 = det.height / fact;
    float divh = float(det.height)/float(height4); //if perfect and no rounding error occurs divh = fact
    int width4 = det.width / fact;
    float divw = float(det.width)/float(width4);

    //
    cout<<"right";
    std::pair<int,int> adress_right = get_valid_point(cloud, cx + int(det.height/fact), cy);
    cout<<"left";
    std::pair<int,int> adress_left = get_valid_point(cloud, cx - int(det.height/fact), cy);
    cout<<"top";
    std::pair<int,int> adress_top = get_valid_point(cloud, cx, cy + int(det.height/fact));
    cout<<"bottom";
    std::pair<int,int> adress_bottom = get_valid_point(cloud, cx, cy - int(det.height/fact));
    cout<<"\n variables gathered \n";
    //give the values to the coordinates
    int xr = adress_right.first , yr = adress_right.second;
    int xl = adress_left.first , yl = adress_left.second;
    int xt = adress_top.first , yt = adress_top.second;
    int xb = adress_bottom.first , yb = adress_bottom.second;

    if(!xr||!yr||!xl||!yl||!xt||!yt||!xb||!yb) 
    {
      cout<<"one variable is 0 in bounding box error detected";
      msg.x = 0; 
      msg.y = 0; 
      msg.z = 0; 
      return msg;
    }

    geometry_msgs::Point center_p;
    geometry_msgs::Point center_3D;

    //calculating real height with rounding error and /2 because height or width /fact was added 2 times
    msg.lz = (cloud->at(xt,yt).z - cloud->at(xb,yb).z)  * divw / 2;
    float width = (cloud->at(xr,yr).x - cloud->at(xl,yl).x) * divw / 2;
    if(width < 0) width = -width;
    msg.ly = width;
    msg.lx = width; 
    center_p.x = cloud->at(cx,cy).x;
    center_p.y = cloud->at(cx,cy).y; 
    center_p.z = cloud->at(cx,cy).z + width;
        
    center_3D = yolo_transform(center_p,tf_cam2world);
    msg.x = center_3D.x; 
    msg.y = center_3D.y;
    msg.z = center_3D.z;
  }
  
  
     
  //######## check if valid if not...
  std::pair<int,int> get_valid_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cld, int x, int y) 
  {
    int b=10;
    int print = 0;
    if(print) cout<<"\n get valid point x: "<<(x)<<" y:"<<(y)<<" \n";                         //print distance --set x/y start value to 100!
    pair<int,int> p=make_pair(x,y);
    pair<int,int> err=make_pair(0,0);
    if(x<b||x>(640-b)||y<b||y>(480-b)) return err; 
    else cld->at(x,y);
    if(isnan(cld->at(x,y).x)||isnan(cld->at(x,y).y)||isnan(cld->at(x,y).z))  return valid_point_(cld,"right",--x,++y,2,0); else return p;
  }
  //######## ... get next valid point
  std::pair<int,int> valid_point_(pcl::PointCloud<pcl::PointXYZ>::Ptr cld, string direction, int x, int y, int dist, int cnt)
  {
    int b=10;                                               //minimum distance to the edges of the image
    int d=3;                                                //distance from the center point in pixel
    std::pair<int,int> p ;                                  //vraiable that is returned at the end
    int not_valid;                                          //variable to check if point is valid 

    //######maintenance 
    int m=0;                                                //ignore nans and check the limits of the algorythm
    int print=1;                                            //print x y z of pcl
    if(print) cout<<"\n valid point x:"<<(x)<<" y:"<<(y)<<" \n";                         //print distance --set x/y start value to 100!
    if(print) cout<<"x:"<<cld->at(x,y).x<<" y:"<<cld->at(x,y).y<<"z:"<<cld->at(x,y).z;
    if(print) cout<<"\n";
    //######maintenance 
    
    if(y<b||y>(480-b)||x<b||x>(640-b))   //too near to the edge
    {
      cout<<"out of bounds"<<"\n";
      return make_pair(0,0);
    }
    else cld->at(x,y);
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
          if(print) cout<<"\n\nrestart";
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
    //define and start timer
    int timer_=1;
    ros::WallTime begin_;
    //###########timer###########
    begin_ = ros::WallTime::now();
    if(timer_){
    cout<<"\n\n Starting timer\n";
    }//###########timer###########

    //recieve max index of mongodb
    current_ind = (*ros::topic::waitForMessage<std_msgs::Int32>(index_topic_,ros::Duration(5))).data;

    //###########timer###########
     if(timer_){
    cout<<"ind after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########

    //get transfromation from camera to world frame
    geometry_msgs::TransformStamped tf_cam2world; 
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener(tfBuffer);
    tf_cam2world = tfBuffer.lookupTransform("head_rgbd_sensor_rgb_frame", "base_link", ros::Time(0), ros::Duration(10.0));//, ros::Duration(1.0) );
    
    //##### 
    //nh_.subscribe (tf_sub_topic, 1,&PointCloudToImage::tf_cb, this);
    // tf2_msgs::TFMessage tf_saved = *ros::topic::waitForMessage<tf2_msgs::TFMessage>(tf_sub_topic,ros::Duration(2));
    //int current_max_id = *ros::topic::waitForMessage<int32>(index_topic_);
    //cout<<current_max_id;
    //###########timer###########
     if(timer_) cout<<"tf after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    //###########timer###########

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
     if(timer_) cout<<"pcl after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    //###########timer###########

    //publish image to yolo
    yolo_pub_.publish (yolo_image);
    pcl_pub_.publish(cloud);

    //###########timer###########
    if(timer_)  cout<<"pub yolo and pcl after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    //###########timer###########

    //recieve bounding box and name from yolo
    DetectionArray yolo_sub_detections = *ros::topic::waitForMessage<DetectionArray>(yolo_sub_topic_);//,ros::Duration(1));
    
    //###########timer###########
    if(timer_){
    if(yolo_sub_detections.detections.size()>0) cout<<"recieve "<<yolo_sub_detections.detections.size()<<" Objects after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########


    //convert msg pcl to xyz pcl
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    //transforming yolo_msg to mongo_msg;
    yolo_store_msg_Array mongo_detections;
    mongo_detections.header = cloud->header;
    mongo_detections.msgs.resize(yolo_sub_detections.detections.size());

    //  transform points and save to mongo_message
    for (int i=0;i<yolo_sub_detections.detections.size();i++)
    {
      cout<<"\n start storing ";
      cout<<yolo_sub_detections.detections[i].label.name;
      cout<<"\n input x:"<<yolo_sub_detections.detections[i].x;
      cout<<"\n input y:"<<yolo_sub_detections.detections[i].y;
      cout<<"\n input w:"<<yolo_sub_detections.detections[i].width;
      cout<<"\n input h:"<<yolo_sub_detections.detections[i].height<<"\n";
      
      
      mongo_detections.msgs[i] = get_yolo_detection(temp_cloud,yolo_sub_detections.detections[i],tf_cam2world);
      if(mongo_detections.msgs[i].x == 0 || mongo_detections.msgs[i].y == 0 ||mongo_detections.msgs[i].z == 0) 
      {
        cout<<"\n error in callback: object: "<<yolo_sub_detections.detections[i].label.name<<" will not be stored \n";
        cout<<"\n  x:"<<yolo_sub_detections.detections[i].x;
        cout<<"\n  y:"<<yolo_sub_detections.detections[i].y<<"\n";
      }
      else cout<<"SUCESS in callback: object: "<<yolo_sub_detections.detections[i].label.name<<" will be stored \n";
    }


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
	//cloud_topic_("/head_xtion/depth_registered/points"),
	cloud_topic_("/hsrb/head_rgbd_sensor/depth_registered/rectified_points"),
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
