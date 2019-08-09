/*!
 * ObjectDetector3D.cpp
 *
 *  Created on: April 4, 2019
 *      Author: Tim Patten
 *      Email: patten@acin.tuwien.ac.at
 *   Institute: ACIN, Technical University of Vienna
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "geometry_msgs/Pose.h"
#include <boost/foreach.hpp>

#include "object_mapping/client.h"
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <utility>
#include <sstream>
#include <cassert>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

namespace {
const std::string YoloActionServerName = "yolo_store/3D_Object_Detector";
const std::string kMapFrameID = "map";
//uint32_t kMarkerShape = visualization_msgs::Marker::CUBE;
//uint32_t kMarkerText = visualization_msgs::Marker::TEXT_VIEW_FACING;
}


ObjectDetector3D::ObjectDetector3D(ros::NodeHandle n)
  : node_handle_(n),
    cloud_(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>()),
    processed_cloud_(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>()),
    darknet_action_server_(kDarknetActionServerName, true) {
  ROS_INFO("[ObjectDetector3D] Node started");
  ROS_INFO("[ObjectDetector3D] Waiting for darknet action server to start");
  darknet_action_server_.waitForServer();
  ROS_INFO("[ObjectDetector3D] Darknet action server started");
  init();
}

ObjectDetector3D::~ObjectDetector3D() {}

void ObjectDetector3D::init() {
  // Get the parameters
  std::string cloud_topic_name;
  std::string detector_service_name;
  std::string marker_publisher_name;
  std::string objects_publisher_name;
  std::string bounding_boxes_publisher_name;
  float m_duration;
  node_handle_.param("/object_detector_3d/cloud_topic", cloud_topic_name, std::string("/camera/depth_registered/points"));
  if (cloud_topic_name.empty())
    ROS_WARN("[ObjectDetector3D] Camera topic name not given, using default");
  node_handle_.param("/object_detector_3d/detection_service", detector_service_name, std::string("detection_service"));
  node_handle_.param("/object_detector_3d/marker_topic", marker_publisher_name, std::string("detect_objects_3d/object_marker"));
  node_handle_.param("/object_detector_3d/detections_topic", objects_publisher_name, std::string("detect_objects_3d/object_detections"));
  node_handle_.param("/object_detector_3d/bounding_boxes_topic", bounding_boxes_publisher_name, std::string("detect_objects_3d/bounding_boxes"));
  node_handle_.param("/object_detector_3d/marker_duration", m_duration, 20.0f);
  marker_duration_ = ros::Duration(m_duration, 0.0f);
  node_handle_.param("/object_detector_3d/probability_threshold", probability_threshold_, 0.1f);
  node_handle_.param("/object_detector_3d/print_results", print_results_, false);

  // Initialize the service
  detector_service_ = node_handle_.advertiseService(detector_service_name, &ObjectDetector3D::objectDetector3DCallback, this);

  // Initialize the point cloud subscriber
  cloud_subscriber_ = node_handle_.subscribe(cloud_topic_name, 1, &ObjectDetector3D::cloudCallback, this);

  // Initialize the publishers
  marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(marker_publisher_name, 1);
  objects_publisher_ = node_handle_.advertise<object_mapping::DetectedObjects3D>(objects_publisher_name, 1);
  bounding_boxes_publisher_ = node_handle_.advertise<darknet_ros_msgs::BoundingBoxes>(bounding_boxes_publisher_name, 1);

  ROS_INFO("[ObjectDetector3D] Waiting for service calls...");
}

void ObjectDetector3D::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl::fromROSMsg(*msg, *cloud_);
  time_ = msg->header.stamp;
  camera_frame_ = msg->header.frame_id;
}

bool ObjectDetector3D::objectDetector3DCallback(object_mapping::detect_objects_3d::Request &req,
                                                object_mapping::detect_objects_3d::Response &res) {
  // Get the image from the point cloud
  cv_bridge::CvImage image;
  ros::Time time_stamp;
  getRGBImage(image.image, time_stamp);
  image.header.stamp = time_stamp;
  image.encoding = sensor_msgs::image_encodings::BGR8;

  // Send goal to darknet action server
  darknet_ros_msgs::CheckForObjectsGoal goal;
  goal.image = *image.toImageMsg();
  darknet_action_server_.sendGoal(goal);
  // Wait for the action to return
  bool finished_before_timeout = darknet_action_server_.waitForResult(ros::Duration(10.0));

  // Get the results
  darknet_ros_msgs::CheckForObjectsResultConstPtr result;
  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = darknet_action_server_.getState();
    ROS_INFO("[ObjectDetector3D] Darknet action finished: %s", state.toString().c_str());
    result = darknet_action_server_.getResult();
  } else {
    ROS_WARN("[ObjectDetector3D] Darknet action did not finish before the time out");
    return false;
  }

  // Get the bounding boxes
  darknet_ros_msgs::BoundingBoxes bounding_boxes = result->bounding_boxes;
  if (bounding_boxes.bounding_boxes.size() == 0) {
    ROS_WARN("[ObjectDetector3D] No objects detected");
    return false;
  }

  ROS_INFO("[ObjectDetector3D] Received %lu bounding boxes", bounding_boxes.bounding_boxes.size());
  bounding_boxes_publisher_.publish(bounding_boxes);

  // Publish object coordinates
  if (bounding_boxes.bounding_boxes.size()) {
    // Set up
    object_mapping::DetectedObjects3D detected_objects;
    detected_objects.header.stamp = ros::Time::now();
    // Calculate center point of the objects
    for (size_t i = 0; i < bounding_boxes.bounding_boxes.size(); ++i) {
      std::cout << bounding_boxes.bounding_boxes.at(i).Class << " " << bounding_boxes.bounding_boxes.at(i).probability << std::endl;
      if (bounding_boxes.bounding_boxes.at(i).probability > probability_threshold_) {
        object_mapping::DetectedObject3D detected_object;
        geometry_msgs::Point coordinates;
        coordinates = getCoordinatesByMean(bounding_boxes.bounding_boxes.at(i));
        geometry_msgs::PointStamped coordinates_map = transformToMap(coordinates);
        publishMarkers(coordinates_map, bounding_boxes.bounding_boxes.at(i).Class, i);
        detected_objects.child_frame_id = coordinates_map.header.frame_id;
        detected_objects.header.frame_id = coordinates_map.header.frame_id;
        detected_object.pose = coordinates_map.point;
        detected_object.Class = bounding_boxes.bounding_boxes.at(i).Class;
        detected_objects.objects.push_back(detected_object);
      }
    }
    res.detections = detected_objects;
    objects_publisher_.publish(detected_objects);
    //res.detections=detected_objects;
  }

  // Clear the processed point cloud
  processed_cloud_->clear();

  // Return success
  return true;
}

void ObjectDetector3D::getRGBImage(cv::Mat &image, ros::Time &time_stamp) {
  time_stamp = time_;
  copyPointCloud(*cloud_, *processed_cloud_);
  image = cv::Mat(processed_cloud_->height, processed_cloud_->width, CV_8UC3);
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      cv::Vec3b color = cv::Vec3b(processed_cloud_->points.at(y*image.cols+x).b,
                                  processed_cloud_->points.at(y*image.cols+x).g,
                                  processed_cloud_->points.at(y*image.cols+x).r);
      image.at<cv::Vec3b>(cv::Point(x,y)) = color;
    }
  }
}

int ObjectDetector3D::getNextValidPoint(int &ind, int &shift_amount, const std::string direction) const {
  int i = 1;
  if (processed_cloud_->points[ind].x != processed_cloud_->points[ind].x && direction == "right") {
    while (i <= shift_amount) {
      ind++;
      if (shift_amount == i) {
        direction == "down";
        ind = getNextValidPoint(ind, shift_amount, direction);
        i++;
      }
    }
  } else if (processed_cloud_->points[ind].x != processed_cloud_->points[ind].x && direction == "down") {
    while (i <= shift_amount){
      ind += processed_cloud_->width;
      if (shift_amount == i) {
        shift_amount++;
        direction == "left";
      }
      ind = getNextValidPoint(ind, shift_amount, direction);
      i++;
    }
  } else if (processed_cloud_->points[ind].x != processed_cloud_->points[ind].x && direction == "left") {
    while (i <= shift_amount) {
      ind--;
      if (shift_amount == i) {
        direction == "up";
        ind = getNextValidPoint(ind, shift_amount, direction);
        i++;
      }
    }
  } else if (processed_cloud_->points[ind].x != processed_cloud_->points[ind].x && direction == "up") {
    while (i <= shift_amount) {
      ind -= processed_cloud_->width;
      if (shift_amount == i) {
        direction == "right";
        shift_amount++;
      }
      ind = getNextValidPoint(ind, shift_amount, direction);
      i++;
    }
  }

  return ind;
}

void ObjectDetector3D::publishMarkers(const geometry_msgs::PointStamped &centroid, const std::string class_name,
    const int id) const {

  // Create shape marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = centroid.header.frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = class_name;
  marker.id = id;
  marker.type = kMarkerShape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = marker_duration_;
  marker.pose.position = centroid.point;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  // Create text marker
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = centroid.header.frame_id;
  text_marker.header.stamp = ros::Time();
  text_marker.type = kMarkerText;
  text_marker.ns = "text";
  text_marker.id = id;
  text_marker.text = class_name;
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0;
  text_marker.scale.z = 0.2;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.position = centroid.point;
  text_marker.pose.position.y -= 0.15;
  text_marker.lifetime = marker_duration_;

  // Publish shape marker
  marker_publisher_.publish(marker);
  // Publish text marker
  marker_publisher_.publish(text_marker);
}

geometry_msgs::Point ObjectDetector3D::getCoordinatesByMean(const darknet_ros_msgs::BoundingBox &bounding_box) const {
  // Get centroid and bounds
  int xc = round(bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin) * 0.5);
  int yc = round(bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin) * 0.5);
  int x_max_mean = bounding_box.xmax - (bounding_box.xmax - bounding_box.xmin) / 4;
  int x_min_mean = bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin) / 4;
  int y_max_mean = bounding_box.ymax - (bounding_box.ymax - bounding_box.ymin) / 4;
  int y_min_mean = bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin) / 4;

  // Get valid point
  int ind = (xc) + (yc)*processed_cloud_->width;
  int shift = 1;
  if (processed_cloud_->points[ind].x != processed_cloud_->points[ind].x)
    ind = getNextValidPoint(ind, shift, "right");

  // Get the mean z height
  std::vector<double> v;
  float z;
  for (int i = x_min_mean; i <= x_max_mean; i++) {
    for (int j = y_min_mean; j <= y_max_mean; j++) {
      ind = (i) + (j) * processed_cloud_->width;
      z = (float)processed_cloud_->points[ind].z;
      if (z == z && z != 0) {
        v.push_back(z);
      }
    }
  }
  float mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();

  // Set the coordinates
  geometry_msgs::Point coordinates;
  coordinates.x = processed_cloud_->points[ind].x;
  coordinates.y = processed_cloud_->points[ind].y;
  if (mean != 0 && mean == mean) {
    coordinates.z = mean;
  } else {
    coordinates.z = processed_cloud_->points[ind].z;
  }

  if (print_results_) {
    ROS_INFO("%s found with coordinates:\n x = %d y = %d ind = %d ", bounding_box.Class.c_str(), xc, yc, ind);
    ROS_INFO("real coordinates : \nx = %f y = %f z = %f ", coordinates.x, coordinates.y, mean);
    ROS_INFO("cloud point : xc = %f yc = %f zc = %f\nwidth = %d ",
             processed_cloud_->points[ind].x, processed_cloud_->points[ind].y, processed_cloud_->points[ind].z,
             processed_cloud_->width);
  }

  return coordinates;
}

geometry_msgs::PointStamped ObjectDetector3D::transformToMap(const geometry_msgs::Point &coordinates) const {
  // Transform using the listener
  ros::Time common_time;
  std::string* error;
  geometry_msgs::PointStamped coordinates_stamped;
  coordinates_stamped.point = coordinates;
  coordinates_stamped.header.frame_id = camera_frame_;
  geometry_msgs::PointStamped coordinates_transformed;
  coordinates_transformed.point = coordinates;
  coordinates_transformed.header.frame_id = kMapFrameID;
  try {
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform(camera_frame_.c_str(), kMapFrameID.c_str(), ros::Time(0), ros::Duration(3.0));
    tf::StampedTransform transform;
	tf_listener.lookupTransform(camera_frame_.c_str(), kMapFrameID.c_str(), ros::Time(0), transform);
  tf::Stamped<tf::Point> pin, pout;
	tf::pointStampedMsgToTF(coordinates_stamped, pin);
	pout.setData(transform * pin);
	pout.stamp_ = transform.stamp_;
	pout.frame_id_ = kMapFrameID;
	tf::pointStampedTFToMsg(pout, coordinates_transformed);
  } catch (tf::TransformException ex) {
    // Error occurred!
    ROS_ERROR("Tf listener exception thrown with message '%s'", ex.what() );
    return coordinates_stamped;
  }
  // Return transformed coordinates
  return coordinates_transformed;
}
