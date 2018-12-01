#ifndef __robot_localization_demo__odometry__
#define __robot_localization_demo__odometry__

#include <random>
#include <ros/ros.h>
#include <turtlesim/Pose.h>

namespace robot_localization_demo {

  class TurtleOdometry {
    public:
      TurtleOdometry(ros::NodeHandle node_handle, double frequency, double error_vx_systematic, double error_vx_random,
          double error_wz_systematic, double error_wz_random, bool visualize=false);
      ~TurtleOdometry();
      void spin();
    private:
      ros::NodeHandle node_handle_;
      ros::Subscriber turtle_pose_subscriber_;
      ros::Publisher turtle_twist_publisher_;
      double frequency_;
      std::default_random_engine random_generator_;
      std::normal_distribution<double> random_distribution_vx_;
      std::normal_distribution<double> random_distribution_wz_;
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