#ifndef __robot_localization_demo__positioning_system__
#define __robot_localization_demo__positioning_system__

#include <random>
#include <ros/ros.h>
#include <turtlesim/Pose.h>

namespace robot_localization_demo {

  class TurtlePositioningSystem {
    public:
      TurtlePositioningSystem(ros::NodeHandle node_handle, double frequency, double error_x_systematic,
          double error_x_random, double error_y_systematic, double error_y_random, double error_yaw_systematic,
          double error_yaw_random, bool visualize=false);
      ~TurtlePositioningSystem();
      void spin();
    private:
      ros::NodeHandle node_handle_;
      ros::Subscriber turtle_pose_subscriber_;
      ros::Publisher turtle_pose_publisher_;
      double frequency_;
      std::default_random_engine random_generator_;
      std::normal_distribution<double> random_distribution_x_;
      std::normal_distribution<double> random_distribution_y_;
      std::normal_distribution<double> random_distribution_yaw_;
      bool visualize_;
      std::string visualization_turtle_name_;
      unsigned frame_sequence_;
      ros::Time cached_pose_timestamp_;
      turtlesim::Pose cached_pose_;
      void turtlePoseCallback(const turtlesim::PoseConstPtr & message);
      inline bool isVisualizationRequested() { return visualize_; };
      inline bool isVisualizationTurtleAvailable() { return visualization_turtle_name_ != ""; };
      void spawnAndConfigureVisualizationTurtle(const turtlesim::Pose & initial_pose);
      void moveVisualizationTurtle(const turtlesim::Pose & measurement);
  };

}

#endif