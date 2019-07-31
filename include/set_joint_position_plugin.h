#ifndef FOLLOW_JNTS_PLUGIN_HH
#define FOLLOW_JNTS_PLUGIN_HH

/*#include <string>


// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Events.hh>
// #include <RobotLib/RobotLib.h>

#include "gazebo/transport/transport.hh"*/

//#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/time.hpp>

namespace gazebo{
	class set_joint_position_plugin : public ModelPlugin{
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate();
		
		private:
			void CB_joint_msg(const sensor_msgs::msg::JointState::SharedPtr msg);
			std::shared_ptr<rclcpp::Node> nh;
			std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState> > sub;
			physics::ModelPtr mod;
			//TODO Replace with hashmap for speed.
			std::vector<std::string> j_names;
			std::vector<double> j_poses;
			event::ConnectionPtr updateConnection;
			rclcpp::Time last_time;
	};
	
	GZ_REGISTER_MODEL_PLUGIN(set_joint_position_plugin)
}
#endif
