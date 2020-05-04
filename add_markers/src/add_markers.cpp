#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/AddMarker.h"

ros::Publisher markerPub;

visualization_msgs::Marker getDefaultMarker(uint8_t id) {
  // Set our initial shape type to be a cube
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID Any marker sent with the same namespace and id will overwrite the old
  // one
  marker.ns = "basic_shapes";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
  // (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(5);
  return marker;
}

void runTestAddMarkers() {
  visualization_msgs::Marker marker = getDefaultMarker(0);

  marker.pose.position.x = 3;
  markerPub.publish(marker);

  ros::Duration(10).sleep();

  marker.pose.position.x = -3;
  markerPub.publish(marker);
}

bool handle_command(add_markers::AddMarker::Request& req,
                    add_markers::AddMarker::Response& res) {
  ROS_INFO("x: %1.2f, y: %1.2f", (float)req.x, (float)req.y);

  visualization_msgs::Marker marker = getDefaultMarker(0);
  marker.pose.position.x = (float)req.x;
  marker.pose.position.y = (float)req.y;
  markerPub.publish(marker);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n("~");
  markerPub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (markerPub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  std::string env;
  n.getParam("env", env);

  std::cout << env << std::endl;
  ROS_INFO("%s", env.c_str());
  if (env.compare("test") == 0) {
    ROS_INFO("Test Add Marker");
    runTestAddMarkers();
  } else {
    ROS_INFO("Spin Add Marker Service");
    ros::ServiceServer service = n.advertiseService("/add_markers/add_marker", handle_command);
  }

  ros::spin();
  return 0;
}

