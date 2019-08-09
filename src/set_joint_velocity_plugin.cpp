#include <set_joint_velocity_plugin.h>
namespace gazebo{
	void set_joint_velocity_plugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf){
		set_joint_position_plugin::Load(parent, sdf);
		
		
		//Start subscribing to joint messages.
		sub = nh->create_subscription<sensor_msgs::msg::JointState>(
			sdf->GetElement("topicName")->Get<std::string>(),
			1, 
			std::bind(& set_joint_velocity_plugin::CB_joint_msg_vel, this, std::placeholders::_1)
		);
		
		RCLCPP_INFO(this->nh->get_logger(),
			"%s will take control of the following joints:\n"
		);
		for(unsigned int i = 0; i < this->j_names.size(); i++){
			RCLCPP_INFO(this->nh->get_logger(),
				"\t%s", this->j_names[i].c_str()
			);
		}
		
		this->last_time = this->nh->now();
		
		this->updateConnection = event::Events::ConnectWorldUpdateEnd(boost::bind(&set_joint_position_plugin::OnUpdate, this));
	}

	void set_joint_velocity_plugin::CB_joint_msg_vel(const sensor_msgs::msg::JointState::SharedPtr msg){
		RCLCPP_WARN(this->nh->get_logger(), "GOT CB");
		rclcpp::Duration delta = this->last_time - this->nh->now();
		this->last_time = this->nh->now();
		
		for(unsigned int i = 0; i < msg->velocity.size(); i++){
			for(unsigned int j = 0; j < this->j_names.size(); j++){
				if(this->j_names[j].compare(msg->name[i]) == 0){
					this->j_poses[j] += (delta.seconds() + 0.000001 * delta.nanoseconds()) * msg->velocity[i];
					break;
				}
				RCLCPP_WARN(this->nh->get_logger(),
					"Asked to directly control a non--directly-controllable joint %s",
					msg->name[i].c_str()
				);
			}
		}
	}


	GZ_REGISTER_MODEL_PLUGIN(set_joint_velocity_plugin)
}
