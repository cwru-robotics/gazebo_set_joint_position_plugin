
//#include <algorithm>
//#include <assert.h>

#include <set_joint_position_plugin.h>

//#include <iostream>
//#include <sstream>
//#include <string>

namespace gazebo {
	//TODO Add HWI functionality for ros_controller interface.

	void set_joint_position_plugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf){
		//ROS init.
		this->nh = rclcpp::Node::make_shared(this->GetHandle() + "_sjp_node");
		
		//Get the joints which we are supposed to be operating on.
		std::string all_jnts = sdf->GetElement("joints")->Get<std::string>();
		
		this->mod = parent;
		
		std::stringstream ss(all_jnts);
		std::string item;
		while(std::getline(ss, item, ' ')){
			physics::JointPtr tmp_jnt = parent->GetJoint(item);
			if(tmp_jnt == NULL){
				RCLCPP_WARN(this->nh->get_logger(),
					"Joint control plugin %s refers to a non-existant joint %s.",
					this->GetHandle().c_str(),
					item.c_str()
				);
			} else{
				this->j_names.push_back(item);
				this->j_poses.push_back(0.0);
			}
		}
		
		//Inform the user that everything is OK.
		RCLCPP_WARN(this->nh->get_logger(),
			"%s is listening for joint states on %s",
			this->GetHandle().c_str(),
			sdf->GetElement("topicName")->Get<std::string>().c_str()
		);
		
		
		//Start subscribing to joint messages.
		sub = nh->create_subscription<sensor_msgs::msg::JointState>(
			sdf->GetElement("topicName")->Get<std::string>(),
			1, 
			std::bind(& set_joint_position_plugin::CB_joint_msg, this, std::placeholders::_1)
		);
		
		RCLCPP_INFO(this->nh->get_logger(),
			"%s will take control of the following joints:\n"
		);
		for(unsigned int i = 0; i < this->j_names.size(); i++){
			RCLCPP_INFO(this->nh->get_logger(),
				"\t%s", this->j_names[i].c_str()
			);
		}
		
		this->updateConnection = event::Events::ConnectWorldUpdateEnd(boost::bind(&set_joint_position_plugin::OnUpdate, this));
	}
	
	void set_joint_position_plugin::OnUpdate(){
		rclcpp::spin_some(this->nh);
		for(unsigned int i = 0; i < this->j_names.size(); i++){
			this->mod->SetJointPosition(this->mod->GetName() + "::" + this->j_names[i], this->j_poses[i]);
		}
	}
	
	void set_joint_position_plugin::CB_joint_msg(const sensor_msgs::msg::JointState::SharedPtr msg){
		for(unsigned int i = 0; i < msg->velocity.size(); i++){
			for(unsigned int j = 0; j < this->j_names.size(); j++){
				if(this->j_names[j].compare(msg->name[i]) == 0){
					this->j_poses[j] = msg->position[i];
					break;
				}
				RCLCPP_WARN(this->nh->get_logger(),
					"Asked to directly control a non--directly-controllable joint %s",
					msg->name[i].c_str()
				);
			}
		}
	}
	
	GZ_REGISTER_MODEL_PLUGIN(set_joint_position_plugin)
}
