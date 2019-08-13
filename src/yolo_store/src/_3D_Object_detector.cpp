/*!
 * 3D_Object_detector.cpp
 *
 *  Created on: April 4, 2019
 *      Author: Tim Patten
 *      Email: patten@acin.tuwien.ac.at
 *   Institute: ACIN, Technical University of Vienna
 */

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
// 
// #include <unordered_map>
// #include <boost/functional/hash.hpp>
// #include <utility>
// #include <sstream>
// #include <cassert>
// 
class _3D_Object_detector {
 public:
  /**
   * Constructor
   */
  _3D_Object_detector(ros::NodeHandle n);

  /**
   * Destructor
   */
  ~_3D_Object_detector();

 private:
  /**
   * Initializes the service, publishers and subscribers.
   */
  void init();

  /**
   * Callback function for the point cloud.
   * @param msg A point cloud message.
   */
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * Callback function for service offered by the class.
   * @param req The service request.
   * @param res The service response.
   * @return True of the service returned with success, otherwise false.
   */
  //bool 3D_Object_detectorCallback(object_mapping::detect_objects_3d::Request &req,
  //                              object_mapping::detect_objects_3d::Response &res);

  /**
   * Gets the RGB image from the current point cloud.
   * @param image The OpenCV matrix of the image (returned).
   * @param time_stampe The time stamp of the point cloud when converted to an image.
   */
   void getRGBImage(cv::Mat &image, ros::Time &time_stamp);

  /**
   * Gets the next valid point in the point cloud from the given index.
   * @param ind The index into the point cloud.
   * @param shift_amount The amount of shift.
   * @param direction The direction to shift.
   * @return The index in the point cloud that is valid.
   */
  int getNextValidPoint(int &ind, int &shift_amount, const std::string direction) const;

  /**
   * Publishes visualization markers (shape and text) for an object.
   * @param centroid The (x,y,z) center of the detected object.
   * @param class_name The class type of the object.
   * @param id The id of the marker.
   */
  void publishMarkers(const geometry_msgs::PointStamped &centroid, const std::string class_name, const int id) const;

  /**
   * Gets the coordinate of an object from the bounding box detection.
   * @param bounding_box The bounding box in the image.
   * @return The coordinate of the object in the point cloud.
   */
  //geometry_msgs::Point getCoordinatesByMean(const darknet_ros_msgs::BoundingBox &bounding_box) const;

   /**
   * Transforms the coordinates to the map frame.
   * @param coordinates The coordinates to transform.
   * @return The coordinates that have been transformed to the map frame.
   */
  // geometry_msgs::PointStamped transformToMap(const geometry_msgs::Point &coordinates) const;

  ros::NodeHandle node_handle_;                           // ROS node handle
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;         // Pointer to cloud
  ros::Time time_;                                        // The time of the current cloud message
  std::string camera_frame_;                              // The frame id of the camera
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr processed_cloud_;  // Pointer to the currently processed cloud
  ros::ServiceServer detector_service_;                   // Service for detecting objects
  ros::Subscriber cloud_subscriber_;                      // Point cloud subscriber
  ros::Publisher marker_publisher_;                       // Marker publisher
  ros::Publisher objects_publisher_;                      // Objects publisher
  ros::Publisher bounding_boxes_publisher_;               // Bounding boxes publisher
  ros::Duration marker_duration_;                         // Duration that markers stay in rviz
  float probability_threshold_;                           // Lower limit on acceptable detection confidence
  bool print_results_;                                    // Print detection results to the terminal
//  actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> darknet_action_server_; // Darknet action server
};



namespace {
const std::string YoloActionServerName = "yolo_store/_3D_Object_Detector";
const std::string kMapFrameID = "map";
//uint32_t kMarkerShape = visualization_msgs::Marker::CUBE;
//uint32_t kMarkerText = visualization_msgs::Marker::TEXT_VIEW_FACING;
}


_3D_Object_detector::_3D_Object_detector(ros::NodeHandle n)
  : node_handle_(n),
    cloud_(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >()),
    processed_cloud_(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >()),

_3D_Object_detector::~_3D_Object_detector() {}

void _3D_Object_detector::init() {
  // Get the parameters
  /*std::string cloud_topic_name;
  std::string detector_service_name;
  std::string marker_publisher_name;
  std::string objects_publisher_name;
  std::string bounding_boxes_publisher_name;
  float m_duration;
  node_handle_.param("/object_detector_3d/cloud_topic", cloud_topic_name, std::string("/camera/depth_registered/points"));
  if (cloud_topic_name.empty())
    ROS_WARN("[3D_Object_detector] Camera topic name not given, using default");
  node_handle_.param("/object_detector_3d/detection_service", detector_service_name, std::string("detection_service"));
  node_handle_.param("/object_detector_3d/marker_topic", marker_publisher_name, std::string("detect_objects_3d/object_marker"));
  node_handle_.param("/object_detector_3d/detections_topic", objects_publisher_name, std::string("detect_objects_3d/object_detections"));
  node_handle_.param("/object_detector_3d/bounding_boxes_topic", bounding_boxes_publisher_name, std::string("detect_objects_3d/bounding_boxes"));
  node_handle_.param("/object_detector_3d/marker_duration", m_duration, 20.0f);
  marker_duration_ = ros::Duration(m_duration, 0.0f);
  node_handle_.param("/object_detector_3d/probability_threshold", probability_threshold_, 0.1f);
  node_handle_.param("/object_detector_3d/print_results", print_results_, false);
  */
  // Initialize the service
  detector_service_ = node_handle_.advertiseService("_3D_Object_detector", &_3D_Object_detector::_3D_Object_detectorCallback, this);
  
  // Initialize the point cloud subscriber
  cloud_subscriber_ = node_handle_.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &_3D_Object_detector::cloudCallback, this);
// /hsrb/head_rgbd_sensor/depth_registered/rectified_points

  // Initialize the publishers
  /*marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(marker_publisher_name, 1);
  objects_publisher_ = node_handle_.advertise<yolo_store::DetectedObjects3D>(objects_publisher_name, 1);
  bounding_boxes_publisher_ = node_handle_.advertise<darknet_ros_msgs::BoundingBoxes>(bounding_boxes_publisher_name, 1);
  */
  image_transport::ImageTransport it(n);
  image_transport::Publisher yolo_publisher_ = it.advertise("yolo_store/yolo_image", 1);
  ROS_INFO("[_3D_Object_detector] Waiting for service calls...");
}

void _3D_Object_detector::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl::fromROSMsg(*msg, *cloud_);
}

bool _3D_Object_detector::_3D_Object_detectorCallback(yolo_store::detect_objects_3d::Request &req,
                                                yolo_store::detect_objects_3d::Response &res) {
  // Get the image from the point cloud
  cv_bridge::CvImage image;
  getRGBImage(image.image);
  image.encoding = sensor_msgs::image_encodings::BGR8;

  // Send goal to darknet action server
  sensor_msgs::Image yolo_send_image = *image.toImageMsg();
  //darknet_action_server_.sendGoal(goal);
  yolo_publisher_.publish(yolo_send_image);
  // Wait for the action to return
      //bool finished_before_timeout = darknet_action_server_.waitForResult(ros::Duration(10.0));

  // Get the results
  std_msgs::StringConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("/yolo_store/yolo_recieve_topic");
  if (msg) std::cout<<"Msg received!"<<std::endl;
  else std::cout<<"No message!"<<std::endl;



  // Publish object coordinates
  // if (bounding_boxes.bounding_boxes.size()) {
  //   // Set up
  //   yolo_store::DetectedObjects3D detected_objects;
  //   detected_objects.header.stamp = ros::Time::now();
  //   // Calculate center point of the objects
  //   for (size_t i = 0; i < bounding_boxes.bounding_boxes.size(); ++i) {
  //     std::cout << bounding_boxes.bounding_boxes.at(i).Class << " " << bounding_boxes.bounding_boxes.at(i).probability << std::endl;
  //     if (bounding_boxes.bounding_boxes.at(i).probability > probability_threshold_) {
  //       yolo_store::DetectedObject3D detected_object;
  //       geometry_msgs::Point coordinates;
  //       coordinates = getCoordinatesByMean(bounding_boxes.bounding_boxes.at(i));
  //       geometry_msgs::PointStamped coordinates_map = transformToMap(coordinates);
  //       publishMarkers(coordinates_map, bounding_boxes.bounding_boxes.at(i).Class, i);
  //       detected_objects.child_frame_id = coordinates_map.header.frame_id;
  //       detected_objects.header.frame_id = coordinates_map.header.frame_id;
  //       detected_object.pose = coordinates_map.point;
  //       detected_object.Class = bounding_boxes.bounding_boxes.at(i).Class;
  //       detected_objects.objects.push_back(detected_object);
  //     }
  //   }
  //   res.detections = detected_objects;
  //   objects_publisher_.publish(detected_objects);
  //   //res.detections=detected_objects;
  // }

  // Clear the processed point cloud
  processed_cloud_->clear();

  // Return success
  return true;
}

void _3D_Object_detector::getRGBImage(cv::Mat &image) {
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
