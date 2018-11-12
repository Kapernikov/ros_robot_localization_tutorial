#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/TeleportRelative.h>
#include <robot_localization_demo/odometry.hpp>


namespace robot_localization_demo {

  TurtleOdometry::TurtleOdometry(ros::NodeHandle node_handle, double frequency,
      double error_vx_systematic, double error_vx_random, double error_wz_systematic, double error_wz_random, bool visualize):
    node_handle_{node_handle},
    turtle_pose_subscriber_{node_handle_.subscribe("/world/turtle1/pose", 16, &TurtleOdometry::turtlePoseCallback, this)},
    turtle_twist_publisher_{node_handle_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/world/turtle1/sensors/twist", 16)},
    frequency_{frequency},
    random_generator_{},
    random_distribution_vx_{error_vx_systematic, error_vx_random},
    random_distribution_wz_{error_wz_systematic, error_wz_random},
    frame_sequence_{0},
    visualize_{visualize},
    visualization_turtle_name_{""}
  {
    ;
  }


  TurtleOdometry::~TurtleOdometry() {
    ;
  }


  void TurtleOdometry::spin() {
    ros::Rate rate(frequency_);
    while(node_handle_.ok()) {
      ros::spinOnce();
      // Distort real twist to get a 'measurement'.
      auto measurement = cached_pose_;
      measurement.linear_velocity *= (1. + random_distribution_vx_(random_generator_));
      measurement.angular_velocity += measurement.linear_velocity * random_distribution_wz_(random_generator_);
      // Publish measurement.
      geometry_msgs::TwistWithCovarianceStamped current_twist;
      current_twist.header.seq = ++ frame_sequence_;
      current_twist.header.stamp = ros::Time::now();
      current_twist.header.frame_id = "base_link";
      current_twist.twist.twist.linear.x = measurement.linear_velocity;
      current_twist.twist.twist.linear.y = 0.;
      current_twist.twist.twist.linear.z = 0.;
      current_twist.twist.twist.angular.x = 0.;
      current_twist.twist.twist.angular.y = 0.;
      current_twist.twist.twist.angular.z = measurement.angular_velocity;
      current_twist.twist.covariance = boost::array<double, 36>({
          std::sqrt(random_distribution_vx_.stddev()), 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., std::sqrt(measurement.linear_velocity * random_distribution_wz_.stddev())});
      turtle_twist_publisher_.publish(current_twist);
      if(visualization_turtle_name_ != "") {
        // Move visualization turtle to the 'measured' position.
        turtlesim::TeleportRelative visualize_current_twist;
        visualize_current_twist.request.linear = measurement.linear_velocity / frequency_;
        visualize_current_twist.request.angular = measurement.angular_velocity / frequency_;
        auto client = node_handle_.serviceClient<decltype(visualize_current_twist)>(
            "/visualization/" + visualization_turtle_name_ + "/teleport_relative");
        client.call(visualize_current_twist);
      }
      // Sleep until we need to publish a new measurement.
      rate.sleep();
    }
  }


  void TurtleOdometry::turtlePoseCallback(const turtlesim::PoseConstPtr & message) {
    cached_pose_timestamp_ = ros::Time::now();
    cached_pose_ = *message;
    // If this is the first message, initialize the visualization turtle.
    if(visualize_ && visualization_turtle_name_ == "") {
      ros::service::waitForService("/visualization/spawn");
      turtlesim::Spawn spawn_visualization_turtle;
      spawn_visualization_turtle.request.x = message->x;
      spawn_visualization_turtle.request.y = message->y;
      spawn_visualization_turtle.request.theta = message->theta;
      auto client = node_handle_.serviceClient<decltype(spawn_visualization_turtle)>("/visualization/spawn");
      client.call(spawn_visualization_turtle);
      visualization_turtle_name_ = spawn_visualization_turtle.response.name;
    }
  }

}