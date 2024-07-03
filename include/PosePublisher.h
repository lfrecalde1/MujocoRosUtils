#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <nav_msgs/msg/odometry.hpp>

#include <string>

namespace MujocoRosUtils
{

/** \brief Plugin to publish odometry of the body. */
class PosePublisher : public rclcpp::Node
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static PosePublisher * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  PosePublisher(PosePublisher &&) = default;

  /** \brief Reset.
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  /** \brief Constructor.
      \param m model
      \param d data
      \param sensor_id sensor ID
      \param node_name node name
      \param topic_name topic name
      \param publish_rate publish rate
  */
  PosePublisher(const mjModel * m,
                mjData * d,
                int sensor_id,
                const std::string & node_name,
                const std::string & topic_name,
                mjtNum publish_rate);

protected:
  //! Sensor ID
  int sensor_id_ = -1;

  //! Body ID
  int body_id_ = -1;

  //! ROS publisher for odometry
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  //! Topic name
  std::string topic_name_;

  //! Iteration interval to skip ROS publish
  int publish_skip_ = 0;

  //! Iteration count of simulation
  int sim_cnt_ = 0;
};

} // namespace MujocoRosUtils