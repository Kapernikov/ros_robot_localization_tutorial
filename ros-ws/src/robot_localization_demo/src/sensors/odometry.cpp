#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportRelative.h>
#include <robot_localization_demo/odometry.hpp>


namespace robot_localization_demo {

  TurtleOdometry::TurtleOdometry(ros::NodeHandle node_handle, double frequency,
      double error_vx_systematic, double error_vx_random, double error_wz_systematic, double error_wz_random, bool visualize):
    node_handle_{node_handle},
    turtle_pose_subscriber_{node_handle_.subscribe("turtle1/pose", 16, &TurtleOdometry::turtlePoseCallback, this)},
    turtle_twist_publisher_{node_handle_.advertise<geometry_msgs::TwistWithCovarianceStamped>("turtle1/sensors/twist", 16)},
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
          std::pow(random_distribution_vx_.mean() + random_distribution_vx_.stddev(), 2), 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., std::pow(measurement.linear_velocity * (random_distribution_wz_.mean() + random_distribution_wz_.stddev()), 2)});
      turtle_twist_publisher_.publish(current_twist);
      if(isVisualizationRequested() && isVisualizationTurtleAvailable()) {
        moveVisualizationTurtle(measurement);
      }
      // Sleep until we need to publish a new measurement.
      rate.sleep();
    }
  }


  void TurtleOdometry::turtlePoseCallback(const turtlesim::PoseConstPtr & message) {
    cached_pose_timestamp_ = ros::Time::now();
    cached_pose_ = *message;
    // If this is the first message, initialize the visualization turtle.
    if(isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
      spawnAndConfigureVisualizationTurtle(*message);
    }
  }


  void TurtleOdometry::spawnAndConfigureVisualizationTurtle(const turtlesim::Pose & initial_pose) {
    if(isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
      // Spawn a new turtle and store its name.
      ros::service::waitForService("spawn");
      turtlesim::Spawn spawn_visualization_turtle;
      spawn_visualization_turtle.request.x = initial_pose.x;
      spawn_visualization_turtle.request.y = initial_pose.y;
      spawn_visualization_turtle.request.theta = initial_pose.theta;
      auto client = node_handle_.serviceClient<decltype(spawn_visualization_turtle)>("spawn");
      client.call(spawn_visualization_turtle);
      visualization_turtle_name_ = spawn_visualization_turtle.response.name;
      // Set pen color to blue.
      turtlesim::SetPen configure_visualization_turtle;
      configure_visualization_turtle.request.r = 255;
      configure_visualization_turtle.request.g = 0;
      configure_visualization_turtle.request.b = 0;
      configure_visualization_turtle.request.width = 1;
      configure_visualization_turtle.request.off = 0;
      auto client_configure = node_handle_.serviceClient<decltype(configure_visualization_turtle)>(
          visualization_turtle_name_ + "/set_pen");
      client_configure.call(configure_visualization_turtle);
      // Log message.
      ROS_INFO("Relative position measurement (odometry) visualized by '%s' with a red pen.", visualization_turtle_name_.c_str());
    }
  }


  void TurtleOdometry::moveVisualizationTurtle(const turtlesim::Pose & measurement) {
    if(isVisualizationRequested() && isVisualizationTurtleAvailable()) {
      // Move visualization turtle to the 'measured' position.
      turtlesim::TeleportRelative visualize_current_twist;
      visualize_current_twist.request.linear = measurement.linear_velocity / frequency_;
      visualize_current_twist.request.angular = measurement.angular_velocity / frequency_;
      auto client = node_handle_.serviceClient<decltype(visualize_current_twist)>(
          visualization_turtle_name_ + "/teleport_relative");
      client.call(visualize_current_twist);
    }
  }

}