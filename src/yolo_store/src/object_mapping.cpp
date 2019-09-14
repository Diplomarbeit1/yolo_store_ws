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
#include <tf/transform_listener.h>

// #include <tmc_vision_msgs/Detection>

using namespace std;
using namespace tmc_vision_msgs;
static int current_ind;
tf2_ros::Buffer *tfBuffer_pntr;

class PointCloudToImage
{
public:
  float sign_(float x)
  {
    if(x < 0) return (x * (-1.0));
    else return x;
  }
  geometry_msgs::Point yolo_transform(geometry_msgs::Point center_p, geometry_msgs::TransformStamped& tf_cam2world)
  {
    int print_tranform = 0; 
    if(print_tranform) cout<<"transform point";
    point_in.point = center_p;
    //point_in.header.frame_id = "head_rgbd_sensor_rgb_frame";
    point_in.header.frame_id = "head_xtion_depth_optical_frame";
    tf2::doTransform(point_in, point_out, tf_cam2world);
    if(print_tranform) cout<<"\ntransformed"<<point_out.point.x;
    return point_out.point;
  }
  yolo_store_msg get_yolo_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,tmc_vision_msgs::Detection det, geometry_msgs::TransformStamped& tf_cam2world) 
  {
    msg.label = det.label;
    print = 1; 
    overwrite = 0;
    cx = round(det.x);
    cy = round(det.y);
    std::pair<int,int> adress = get_valid_point(cloud,cx,cy);


    if(overwrite)
    {
      center_p.x = 1; 
      center_p.y = 1;
      center_p.z = 5; 
      msg.lx = 1; 
      msg.ly = 1; 
      msg.lz = 1;
      center_3D = yolo_transform(center_p,tf_cam2world);
      msg.x = center_3D.x; 
      msg.y = center_3D.y;
      msg.z = center_3D.z;
      return msg;  
    }

    if(!adress.first||!adress.second)
    {

      msg.x = 0; 
      msg.y = 0; 
      msg.z = 0; 
      return msg;
    }
    int x = adress.first;
    int y = adress.second;
    center_p.x = cloud->at(x,y).x;
    center_p.y = cloud->at(x,y).y; 
    center_p.z = cloud->at(x,y).z;
    center_3D = yolo_transform(center_p,tf_cam2world);

    float fact = 4.0;// division factor for assuring that the points for calculating width/height are in the object 
                  // by shrinking the bounding box by the factor/2 and then expanding it later with the factors below
    int height4 = det.height / fact;
    float divh = float(det.height)/float(height4); //if perfect and no rounding error occurs divh = fact
    cout<<"\n divh "<<divh;
    int width4 = det.width / fact;
    float divw = float(det.width)/float(width4);
    cout<<"\n divw "<<divw<<"\n";
    //
    if(print) cout<<"\nright";
    std::pair<int,int> adress_right = get_valid_point(cloud, x + int(det.height/fact), y);
    if(print) cout<<"\nleft";
    std::pair<int,int> adress_left = get_valid_point(cloud, x - int(det.height/fact), y);
    if(print) cout<<"\ntop";
    std::pair<int,int> adress_top = get_valid_point(cloud, x, y - int(det.height/fact));
    if(print) cout<<"\nbottom";
    std::pair<int,int> adress_bottom = get_valid_point(cloud, x, y + int(det.height/fact));
    if(print) cout<<"\n variables gathered \n";
    //give the values to the coordinates
    int xr = adress_right.first , yr = adress_right.second;
    int xl = adress_left.first , yl = adress_left.second;
    int xt = adress_top.first , yt = adress_top.second;
    int xb = adress_bottom.first , yb = adress_bottom.second;
    if(!xr||!yr||!xl||!yl||!xt||!yt||!xb||!yb) 
    {
      cout<<"ERROR detected one variable is 0";
      msg.x = 0; 
      msg.y = 0; 
      msg.z = 0; 
      return msg;
    }
    if(print)
    {
      cout<<"\n xr "<<cloud->at(xr,yr).x;
      cout<<"\n yr "<<cloud->at(xr,yr).y;
      cout<<"\n zr "<<cloud->at(xr,yr).z;
      cout<<"\n xt "<<cloud->at(xt,yt).x;
      cout<<"\n yt "<<cloud->at(xt,yt).y;
      cout<<"\n zt "<<cloud->at(xt,yt).z;
      cout<<"\n xb "<<cloud->at(xb,yb).x;
      cout<<"\n yb "<<cloud->at(xb,yb).y;
      cout<<"\n zb "<<cloud->at(xb,yb).z;
      cout<<"\n xl "<<cloud->at(xl,yl).x;
      cout<<"\n yl "<<cloud->at(xl,yl).y;
      cout<<"\n zl "<<cloud->at(xl,yl).z;
    }

    

    cam_p.x = 0.0; 
    cam_p.y = 0.0; 
    cam_p.z = 0.0; 
    cam_tf = yolo_transform(cam_p,tf_cam2world);

    right_p.x = cloud->at(xr,yr).x;
    right_p.y = cloud->at(xr,yr).y; 
    right_p.z = cloud->at(xr,yr).z;
    right_tf = yolo_transform(right_p,tf_cam2world);

    left_p.x = cloud->at(xl,yl).x;
    left_p.y = cloud->at(xl,yl).y; 
    left_p.z = cloud->at(xl,yl).z;
    left_tf = yolo_transform(left_p,tf_cam2world);

    bottom_p.x = cloud->at(xb,yb).x;
    bottom_p.y = cloud->at(xb,yb).y; 
    bottom_p.z = cloud->at(xb,yb).z;
    bottom_tf = yolo_transform(bottom_p,tf_cam2world);

    top_p.x = cloud->at(xt,yt).x;
    top_p.y = cloud->at(xt,yt).y; 
    top_p.z = cloud->at(xt,yt).z;
    top_tf = yolo_transform(top_p,tf_cam2world);
    
    //distance of left to right 
    float width_standing = (sqrt(pow((right_tf.x - left_tf.x),2.0) + pow((right_tf.y - left_tf.y),2.0)))*divw*float(fact) / 2.0;
    float height_standing = (top_tf.z - bottom_tf.z);

    if(print)
    {
      
      cout<<"\nwidth: "<<width_standing;
      cout<<"\nheight: "<<height_standing;
      cout<<"\n right left x:" << right_tf.x <<" "<< left_tf.x;
      cout<<"\n right left y:" << right_tf.y <<" "<< left_tf.y;
      cout<<"\nrest:" << right_tf.z <<" "<< left_tf.z;
      cout<<"calc" << pow((right_tf.x - left_tf.x),2.0);
      cout<<"root" << sqrt(pow((right_tf.x - left_tf.x),2.0) + pow((right_tf.y - left_tf.y),2.0));
      
    }
    if(height_standing > width_standing)
    {
      if(print) cout<<"\nstanding";
      //object is standing 
      msg.lz = height_standing;
      msg.ly = width_standing;
      msg.lx = width_standing; 
      msg.x = center_3D.x; 
      msg.y = center_3D.y;
      msg.z = center_3D.z;
      msg.center_p.x = center_3D.x;
      msg.center_p.y = center_3D.y;
      msg.center_p.z = center_3D.z;
      
    }
    else 
    {
      if(print) cout<<"\nlying";
      //object is lying 
      msg.lz = 0.05;
      msg.ly = width_standing;
      msg.lx = width_standing;
      msg.x = center_3D.x; 
      msg.y = center_3D.y;
      msg.z = center_3D.z; 
      msg.center_p.x = center_3D.x;
      msg.center_p.y = center_3D.y;
      msg.center_p.z = center_3D.z;
      int print; 
      int overwrite;
      int cx,cy;
    }


    // //calculating real height including rounding error and /2 because ((height or width)/fact) was added 2 times
    // msg.lz = (cloud->at(xt,yt).x - cloud->at(xb,yb).x)  * divw / 2; 
    // if(msg.lz < 0) msg.lz = -msg.lz;
    // float width = (cloud->at(xr,yr).y - cloud->at(xl,yl).y) * divw / 2; 
    // int disp = 1;
    // if(width < 0) width = -width;
    
 


    return msg; 
  }
  
  
     
  //######## check if valid if not...
  std::pair<int,int> get_valid_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cld, int x, int y) 
  {
    int b = 5;
    int print = 0;
    int print2 = 1; 
    if(print) cout<<"\n get valid point x: "<<(x)<<" y:"<<(y)<<" \n";                         //print distance --set x/y start value to 100!
    if(print) cout<<"\n get valid point x: "<<cld->at(x,y).x<<" y:"<<cld->at(x,y).y<<" z:"<<cld->at(x,y).y<<" \n";
    pair<int,int> p = make_pair(x,y);
    pair<int,int> err = make_pair(0,0);

    if(x<b||x>(640-b)||y<b||y>(480-b)) return err; 
    else if(isnan(cld->at(x,y).x)||isnan(cld->at(x,y).y)||isnan(cld->at(x,y).z))  return valid_point_(cld,"right",--x,++y,2,0);
    else return p;
  }
  //######## ... get next valid point
  std::pair<int,int> valid_point_(pcl::PointCloud<pcl::PointXYZ>::Ptr cld, string direction, int x, int y, int dist, int cnt)
  {
    int b = 10;                                               //minimum distance to the edges of the image
    int d = 3;                                                //distance from the center point in pixel
    std::pair<int,int> p ;                                  //vraiable that is returned at the end
    int not_valid;                                          //variable to check if point is valid 

    //######maintenance 
    int m = 0;                                                //ignore nans and check the limits of the algorythm
    int print = 0;                                            //print x y z of pcl
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
    timer_=1;
    ros::WallTime begin_;
    //###########timer###########
    begin_ = ros::WallTime::now();
    if(timer_) cout<<"\n\n Starting timer\n";
    //###########timer###########

    //recieve max index of mongodb
    current_ind = (*ros::topic::waitForMessage<std_msgs::Int32>(index_topic_)).data;//,ros::Duration(5))).data;

    //###########timer###########
     if(timer_) cout<<"ind after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    //###########timer###########

    //get transfromation from camera to world frame
    // tf::TransformListener list_;
    // tf::StampedTransform transform;
    // list_.lookupTransform("map","head_rgbd_sensor_rgb_frame",ros::Time(0), transform);
    // tf2_ros::TransformListener tfListener(tfBuffer);
    
    //try{tf_cam2world = tfBuffer_pntr->lookupTransform("map","head_rgbd_sensor_rgb_frame", cloud->header.stamp);//, ros::Duration(1.0));//, ros::Duration(1.0) );

    try{
        tf_cam2world = tfBuffer_pntr->lookupTransform("map","head_xtion_depth_optical_frame", cloud->header.stamp);//, ros::Duration(1.0));//, ros::Duration(1.0) );
      } catch (tf2::TransformException &ex) { 
      std::cout << ex.what() << std::endl <<"Ignoring this pcl" <<std::endl;
      return;
    }
    // tf::TransformListener listener;
    // tf::StampedTransform tf_cam2world;
    // listener.lookupTransform("/turtle2", "/turtle1",
    //                            ros::Time(0), tf_cam2world);
    // cout<<"\n\n transformation"<<tf_cam2world<<"\n\n";


    //##### 
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
      return;
    }
    //###########timer###########
     if(timer_) cout<<"pcl after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    //###########timer###########

    //publish image to yolo
    yolo_pub_.publish (yolo_image);
    //pcl_pub_.publish (cloud);

    //###########timer###########
    if(timer_)  cout<<"pub yolo and pcl after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    //###########timer###########

    //recieve bounding box and name from yolo
    int yolo_time = 15;
    try{
       yolo_sub_detections = ros::topic::waitForMessage<DetectionArray>(yolo_sub_topic_,ros::Duration(yolo_time));
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Results from yolo not recieved after: " <<yolo_time << " seconds \n ...restarting"<<"\nError:"<< e.what());
      return;
    }
    if(yolo_sub_detections == NULL) 
    {
      ROS_ERROR_STREAM("No results recieved from yolo after: " <<yolo_time << " seconds \n ...restarting");
      return;
    }
    cout<<"still here";
    //###########timer###########
    if(timer_){
    if(yolo_sub_detections->detections.size()>0)if(yolo_sub_detections->detections.size()>0) cout<<"recieve "<<yolo_sub_detections->detections.size()<<" Objects after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    else cout<<"did no recieve any objects";
    }//###########timer###########


    //convert msg pcl to xyz pcl
    pcl_conversions::toPCL(*cloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    //transforming yolo_msg to mongo_msg;
    yolo_store_msg_Array mongo_detections;
    mongo_detections.header = cloud->header;
    mongo_detections.msgs.resize(yolo_sub_detections->detections.size());

    //  transform points and save to mongo_message
    int running_ = 1;
    int i = 0;
    int var_ = 0;
  
    while(running_ == 1)
    {
      cout<<"\n start storing "<<yolo_sub_detections->detections[i].label.name;
      cout<<"\n input x:"<<yolo_sub_detections->detections[i].x;
      cout<<"\n input y:"<<yolo_sub_detections->detections[i].y;
      cout<<"\n input w:"<<yolo_sub_detections->detections[i].width;
      cout<<"\n input h:"<<yolo_sub_detections->detections[i].height<<"\n";

      mongo_detections.msgs[i] = get_yolo_detection(temp_cloud,yolo_sub_detections->detections[i],tf_cam2world);
      if(mongo_detections.msgs[i].x == 0 || mongo_detections.msgs[i].y == 0 || mongo_detections.msgs[i].z == 0) 
      {
        cout<<"\n error in callback: object: "<<yolo_sub_detections->detections[i].label.name<<" will NOT be stored \n";
      }
      else 
      {
        cout<<"SUCESS in callback: object: "<<yolo_sub_detections->detections[i].label.name<<" will be stored \n";
        i++;
      }
      var_++;
      if(var_ == yolo_sub_detections->detections.size()) running_ = 0;
    }
    cout<<"i: "<<i;
    mongo_detections.msgs.resize(i);


    //###########timer###########
    if(timer_){
    if(yolo_sub_detections->detections.size()>0) cout<<"\n transform to mongo_msg for loop after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
    }//###########timer###########

    //publish to mongo
    if(i) mongo_pub_.publish (mongo_detections);

    //###########timer###########
    if(timer_){
    if(yolo_sub_detections->detections.size()>0) cout<<"\n publish "<<i<<" to mongo after "<<(ros::WallTime::now() - begin_).toSec()<< " seconds\n";
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
  geometry_msgs::TransformStamped tf_cam2world;
  //tf2_ros::Buffer tfBuffer;
  //tf2_ros::TransformListener tf2_listener;
  geometry_msgs::Point right_p, left_p, bottom_p, top_p;
  geometry_msgs::Point right_tf, left_tf, bottom_tf, top_tf;
  geometry_msgs::Point cam_p, cam_tf;
  geometry_msgs::PointStamped point_in, point_out;
  geometry_msgs::Point center_p;
  geometry_msgs::Point center_3D;
  //DetectionArray yolo_sub_detections;
  boost::shared_ptr <tmc_vision_msgs::DetectionArray const> yolo_sub_detections;
  yolo_store_msg msg;
  int print, overwrite, timer_;
  float cx,cy,cz;
  int x;
  pcl::PCLPointCloud2 pcl_pc2;
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "object_mapping");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tfBuffer_pntr= &tfBuffer;

  PointCloudToImage pci;

  
  ros::spin();

  return 0;
}

  // while (ros::ok()){
  // try{PointCloudToImage pci;} //this loads up the node
  //   catch (tf::TransformException &ex) {
  //     ROS_ERROR("%s",ex.what());
  //     continue;
  //   }
//   //Loop
//    while (ros::ok()) {
//         cout<<"starting transform"<<endl;
//         PointCloudToImage pci; //this loads up the node
// 	//ros::Rate rate(1);
//         //rate.sleep();
//    }