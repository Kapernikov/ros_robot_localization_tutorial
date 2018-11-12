#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <robot_localization_demo/positioning_system.hpp>


int main(int argc, char ** argv) {

  double frequency;
  double error_x_systematic;
  double error_x_random;
  double error_y_systematic;
  double error_y_random;
  double error_yaw_systematic;
  double error_yaw_random;

  namespace po = boost::program_options;
  po::options_description description("Recognised options");
  description.add_options()
      ("help,h", "print help message")
      ("frequency,f", po::value<double>(&frequency)->default_value(1.), "set measurement frequency (Hz)")
      ("error-x-systematic,X", po::value<double>(&error_x_systematic)->default_value(0.), "set systematic error on X")
      ("error-x-random,x", po::value<double>(&error_x_random)->default_value(0.), "set random error on X")
      ("error-y-systematic,Y", po::value<double>(&error_y_systematic)->default_value(0.), "set systematic error on Y")
      ("error-y-random,y", po::value<double>(&error_y_random)->default_value(0.), "set random error on Y")
      ("error-yaw-systematic,T", po::value<double>(&error_yaw_systematic)->default_value(0.), "set systematic error on yaw")
      ("error-yaw-random,t", po::value<double>(&error_yaw_random)->default_value(0.), "set random error on yaw")
      ("visualize,v", "visualize positioning system measurement");
  po::variables_map variables_map;
  po::store(po::parse_command_line(argc, argv, description), variables_map);
  po::notify(variables_map);

  if (variables_map.count("help")) {
    std::cout << description << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "turtle_positioning_system");
  ros::NodeHandle node_handle;

  robot_localization_demo::TurtlePositioningSystem turtle_positioning_system{node_handle, frequency,
    error_x_systematic, error_x_random, error_y_systematic, error_y_random,
    error_yaw_systematic, error_yaw_random, (variables_map.count("visualize")? true: false)};
  turtle_positioning_system.spin();

  return EXIT_SUCCESS;
}