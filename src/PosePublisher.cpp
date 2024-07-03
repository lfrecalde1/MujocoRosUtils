#include "PosePublisher.h"

#include <mujoco/mujoco.h>

#include <iostream>

namespace MujocoRosUtils
{

void PosePublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::PosePublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char * attributes[] = {"node_name", "topic_name", "publish_rate"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel *, // m
                           int, // plugin_id
                           int // sensor_id
                        ) { return 0; };

  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto * plugin_instance = PosePublisher::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<PosePublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class PosePublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    )
  {
    auto * plugin_instance = reinterpret_cast<class PosePublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

PosePublisher * PosePublisher::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // node_name
  const char * node_name_char = mj_getPluginConfig(m, plugin_id, "node_name");
  std::string node_name = "";
  if(node_name_char && strlen(node_name_char) > 0)
  {
    node_name = std::string(node_name_char);
  }

  // topic_name
  const char * topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  std::string topic_name = "";
  if(topic_name_char && strlen(topic_name_char) > 0)
  {
    topic_name = std::string(topic_name_char);
  }

  // publish_rate
  const char * publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
  mjtNum publish_rate = 30.0;
  if(publish_rate_char && strlen(publish_rate_char) > 0)
  {
    publish_rate = strtod(publish_rate_char, nullptr);
  }
  if(publish_rate <= 0)
  {
    mju_error("[PosePublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // Set sensor_id
  int sensor_id = 0;
  for(; sensor_id < m->nsensor; sensor_id++)
  {
    if(m->sensor_type[sensor_id] == mjSENS_PLUGIN && m->sensor_plugin[sensor_id] == plugin_id)
    {
      break;
    }
  }
  if(sensor_id == m->nsensor)
  {
    mju_error("[PosePublisher] Plugin not found in sensors.");
    return nullptr;
  }
  if(m->sensor_objtype[sensor_id] != mjOBJ_XBODY)
  {
    mju_error("[PosePublisher] Plugin must be attached to a xbody.");
    return nullptr;
  }

  std::cout << "[PosePublisher] Create." << std::endl;

  // Ensure ROS 2 is initialized
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  if (node_name.empty()) {
    node_name = "pose_publisher";
  }

  return new PosePublisher(m, d, sensor_id, node_name, topic_name, publish_rate);
}

PosePublisher::PosePublisher(const mjModel * m,
                             mjData *, // d
                             int sensor_id,
                             const std::string & node_name,
                             const std::string & topic_name,
                             mjtNum publish_rate)
: Node(node_name), sensor_id_(sensor_id), body_id_(m->sensor_objid[sensor_id]), topic_name_(topic_name),
  publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
{
  if(topic_name_.empty())
  {
    topic_name_ = "mujoco/odometry";
  }

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_name_, 100);
}

void PosePublisher::reset(const mjModel *, // m
                          int // plugin_id
)
{
}

void PosePublisher::compute(const mjModel * m, mjData * d, int // plugin_id
)
{
  sim_cnt_++;
  if(sim_cnt_ % publish_skip_ != 0)
  {
    return;
  }

  auto stamp_now = this->get_clock()->now();

  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = stamp_now;
  odom_msg.header.frame_id = "map";

  // Position
  odom_msg.pose.pose.position.x = d->xpos[3 * body_id_ + 0];
  odom_msg.pose.pose.position.y = d->xpos[3 * body_id_ + 1];
  odom_msg.pose.pose.position.z = d->xpos[3 * body_id_ + 2];

  // Orientation (quaternion)
  odom_msg.pose.pose.orientation.w = d->xquat[4 * body_id_ + 0];
  odom_msg.pose.pose.orientation.x = d->xquat[4 * body_id_ + 1];
  odom_msg.pose.pose.orientation.y = d->xquat[4 * body_id_ + 2];
  odom_msg.pose.pose.orientation.z = d->xquat[4 * body_id_ + 3];

  // Linear and Angular Velocity
  mjtNum vel[6];
  mj_objectVelocity(m, d, mjOBJ_XBODY, body_id_, vel, 0);
  odom_msg.twist.twist.linear.x = vel[3];
  odom_msg.twist.twist.linear.y = vel[4];
  odom_msg.twist.twist.linear.z = vel[5];
  odom_msg.twist.twist.angular.x = vel[0];
  odom_msg.twist.twist.angular.y = vel[1];
  odom_msg.twist.twist.angular.z = vel[2];

  odom_pub_->publish(odom_msg);
}

} // namespace MujocoRosUtils