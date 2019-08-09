/*
 * cloud_publisher.cpp
 *
 *  Created on: April 3, 2019
 *      Author: Tim Patten
 *      Email: patten@acin.tuwien.ac.at
 *   Institute: ACIN, Technical University of Vienna
 */

//#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"
#include <boost/foreach.hpp>

//#include "object_mapping/client.h"
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <utility>
#include <sstream>
#include <cassert>

using std::string;
using namespace std;
using namespace geometry_msgs;
//using namespace mongodb_store;
using namespace std;

//ros::ServiceClient *_mongo_client;



static bool info = true;

class yolo_store_client{
      // getRGBImage(cv::Mat &image, ros::Time &time_stamp) {
      // time_stamp = time_;
      // copyPointCloud(*cloud_, *processed_cloud_);
      // image = cv::Mat(processed_cloud_->height, processed_cloud_->width, CV_8UC3);
      // for (int y = 0; y < image.rows; y++) {
      //     for (int x = 0; x < image.cols; x++) {
      //       cv::Vec3b color = cv::Vec3b(processed_cloud_->points.at(y*image.cols+x).b,
      //                                   processed_cloud_->points.at(y*image.cols+x).g,
      //                                   processed_cloud_->points.at(y*image.cols+x).r);
      //       image.at<cv::Vec3b>(cv::Point(x,y)) = color;
      //     }
      //   }
      // }
public:
  init()
  {
    sub = n.subscribe("/", 1, chatterCallback);
    pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
  }
  
//mongodb set up:
 /*private:
  MessageStoreProxy *messageStore;
 public:


 ObjectMappingClient(ros::NodeHandle &nh)
  : messageStore(new MessageStoreProxy(nh))
  {}
*/


private:
  ros::NodeHandle n; 
  ros::Publisher pub;
  ros::Subscriber sub;
};

int main(int argc, char **argv){
  //Initalize
  ros::init(argc, argv, "yolo_store");
  //ros::ServiceClient client = nh.serviceClient<object_mapping::detect_objects_3d>("detect_objects_3d/detect_objects_3d");
  
  ros::Rate rate(1);
  

  //Loop
 /*  while (ros::ok()) {
        cout<<"starting client"<<endl;
        if (sub.call(srv))
        {
          //cl_.client_call(srv.response.detections);//, messageStore); 
          ObjectMappingClient::client_call(srv.response.detections);//, messageStore); 
        }
        else
        {
          cout<<"No objects detected"<<endl;
          //ROS_INFO("CLIENT=FALSE"); //No objects detected or client failure
          
        }
        rate.sleep();
   }
   */
  return 0;
}
