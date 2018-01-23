#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/server.h>
#include <zero_gravity_control/zero_gravity_controlConfig.h>

#include "boost/bind.hpp"

class ZeroGravityControl
{
public:
  ZeroGravityControl() : initialized_(false), force_updated_(false)
  {
    std::string wrench_topic;
    std::string cart_controller_topic;
    ros::NodeHandle pn("~");
    pn.param("end_effector_frame", end_effector_frame_, std::string("s_model_tool0"));
    pn.param("wrench_topic", wrench_topic, std::string("wrench_topic"));
    pn.param("cart_controller_topic", cart_controller_topic, std::string("moveit_cartesian_traj_controller/cartesian_command"));

    // dynamic reconfigure setup
    f_ = boost::bind(&ZeroGravityControl::dynamic_reconfigure_callback, this, _1, _2);
    server_.setCallback(f_);

    wrench_sub_ = nh_.subscribe(wrench_topic, 1, &ZeroGravityControl::wrench_callback, this);
    cart_command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cart_controller_topic, 1);

    // Initialize desired_pose_
    desired_pose_.header.frame_id = end_effector_frame_;
    desired_pose_.pose.orientation.w = 1.0;
  }

  void wrench_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    geometry_msgs::Vector3Stamped pos;
    pos.header = msg->header;
    pos.vector = msg->wrench.force;

    try
    {
      tf_listener_.waitForTransform(end_effector_frame_, pos.header.frame_id, pos.header.stamp, ros::Duration(.1));
      tf_listener_.transformVector(end_effector_frame_, pos, pos);
    }
    catch(std::runtime_error& e)
    { 
      ROS_WARN_STREAM("Could not transform wrench: " << e.what());
      return;
    }
     
    {
      std::lock_guard<std::mutex> lock(force_mutex_);
      force_ = pos.vector;
      force_updated_ = true;
    }
    if(!initialized_)
      initial_forces_.push_back(force_);
  }

  void dynamic_reconfigure_callback(zero_gravity_control::zero_gravity_controlConfig &config, uint32_t level)
  {
    ROS_INFO("Reconfigure Request: %f %f", config.decay, config.force_factor);

    decay_ = config.decay;
    force_factor_ = config.force_factor;
  }

  void spin()
  {
    geometry_msgs::Vector3 current_force;
    double position_magnitude;

    ros::Rate r(125);
    while(ros::ok())
    {
      if(force_updated_ && initialized_)
      {
        {
          std::lock_guard<std::mutex> lock(force_mutex_);
          force_updated_ = false;
          current_force = force_;
        }

        current_force.x = current_force.x - force_offset_.x;
        current_force.y = current_force.y - force_offset_.y;
        current_force.z = current_force.z - force_offset_.z;

        desired_pose_.pose.position.x = desired_pose_.pose.position.x * decay_;
        desired_pose_.pose.position.y = desired_pose_.pose.position.y * decay_;
        desired_pose_.pose.position.z = desired_pose_.pose.position.z * decay_;

        desired_pose_.pose.position.x = std::max(-0.1, std::min(desired_pose_.pose.position.x + (current_force.x * force_factor_), 0.1));
        desired_pose_.pose.position.y = std::max(-0.1, std::min(desired_pose_.pose.position.y + (current_force.y * force_factor_), 0.1));
        desired_pose_.pose.position.z = std::max(-0.1, std::min(desired_pose_.pose.position.z + (current_force.z * force_factor_), 0.1));

        position_magnitude = sqrt(std::pow(desired_pose_.pose.position.x,2.0) + std::pow(desired_pose_.pose.position.y,2.0) + std::pow(desired_pose_.pose.position.z,2.0));

        if(position_magnitude > 0.005)
        {
          cart_command_pub_.publish(desired_pose_);
        }
      }
      else if(!initialized_ && initial_forces_.size() > 200)
      {
        int iterations = 0;
        geometry_msgs::Vector3 sum_force;

        for(auto force : initial_forces_)
        {
          sum_force.x += force.x;
          sum_force.y += force.y;
          sum_force.z += force.z;
          iterations++;
        }

        force_offset_.x = sum_force.x/double(iterations);
        force_offset_.y = sum_force.y/double(iterations);
        force_offset_.z = sum_force.z/double(iterations);

        initialized_ = true;
        ROS_INFO("Initialization successful.");
      }
    }
  }
private:
  ros::NodeHandle nh_;

  ros::Subscriber wrench_sub_;
  ros::Publisher cart_command_pub_;
  dynamic_reconfigure::Server<zero_gravity_control::zero_gravity_controlConfig> server_;
  dynamic_reconfigure::Server<zero_gravity_control::zero_gravity_controlConfig>::CallbackType f_;

  geometry_msgs::Vector3 force_;
  geometry_msgs::Vector3 force_offset_;
  std::string end_effector_frame_;
  std::mutex force_mutex_;
  bool force_updated_;
  bool initialized_;
  std::vector<geometry_msgs::Vector3> initial_forces_;
  double decay_;
  double force_factor_;
  geometry_msgs::PoseStamped desired_pose_;

  tf::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zero_gravity_control_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Duration(2.0).sleep();

  ZeroGravityControl controller;

  controller.spin();

  return 0;
}
