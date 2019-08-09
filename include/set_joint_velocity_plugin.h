#ifndef FOLLOW_VELS_PLUGIN_HH
#define FOLLOW_VELS_PLUGIN_HH

#include "set_joint_position_plugin.h"

namespace gazebo{
	class set_joint_velocity_plugin : public set_joint_position_plugin{
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		//public: void OnUpdate();
		
		private:
			void CB_joint_msg_vel(const sensor_msgs::msg::JointState::SharedPtr msg);
			rclcpp::Time last_time;
		//	std::shared_ptr<rclcpp::Node> nh;
		//	std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState> > sub;
		//	physics::ModelPtr mod;
		//	//TODO Replace with hashmap for speed.
		//	std::vector<std::string> j_names;
		//	std::vector<double> j_poses;
		//	event::ConnectionPtr updateConnection;
		//	rclcpp::Time last_time;
	};
}
#endif
