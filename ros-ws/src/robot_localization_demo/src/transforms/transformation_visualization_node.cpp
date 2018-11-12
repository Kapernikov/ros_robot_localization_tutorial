#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/TeleportAbsolute.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <turtlesim/Spawn.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "transformation_visualization_node");

  ros::NodeHandle node_handle;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Rate rate(10.0);
  while(node_handle.ok()) {
    // Get base_link to map transformation.
    geometry_msgs::TransformStamped base_link_to_map_transform;
    try {
      base_link_to_map_transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_INFO("tf2_ros::Buffer::lookupTransform failed: %s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // Move visualization turtle to the estimated position.
    geometry_msgs::PoseStamped pose_base_link, pose_map;
    pose_base_link.header.stamp = ros::Time::now();
    pose_base_link.header.frame_id = "base_link";
    pose_base_link.pose.position.x = 0.;
    pose_base_link.pose.position.y = 0.;
    pose_base_link.pose.position.z = 0.;
    pose_base_link.pose.orientation.x = 0.;
    pose_base_link.pose.orientation.y = 0.;
    pose_base_link.pose.orientation.z = 0.;
    pose_base_link.pose.orientation.w = 1.;
    tf2::doTransform(pose_base_link, pose_map, base_link_to_map_transform);
    turtlesim::TeleportAbsolute visualize_current_pose;
    visualize_current_pose.request.x = pose_map.pose.position.x;
    visualize_current_pose.request.y = pose_map.pose.position.y;
    tf2::Quaternion quaternion(pose_map.pose.orientation.x, pose_map.pose.orientation.y, pose_map.pose.orientation.z, pose_map.pose.orientation.w);
    visualize_current_pose.request.theta = quaternion.getAngle();
    auto client = node_handle.serviceClient<decltype(visualize_current_pose)>("/visualization/turtle1/teleport_absolute");
    client.call(visualize_current_pose);

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
