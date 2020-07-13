
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
		int a = 0;//No, it will NOT just accept an argument size of 0 without shenanigans. Annoying.
		ros::init(a, (char **) NULL, this->GetHandle() + "_sjp_node");
		this->nh = & (this->nh_inner);//TODO This is weird.
		//this->nh = rclcpp::Node::make_shared(this->GetHandle() + "_sjp_node");
		
		//Get the joints which we are supposed to be operating on.
		std::string all_jnts = sdf->GetElement("joints")->Get<std::string>();
		
		this->mod = parent;
		
		std::stringstream ss(all_jnts);
		std::string item;
		while(std::getline(ss, item, ' ')){
			physics::JointPtr tmp_jnt = parent->GetJoint(item);
			if(tmp_jnt == NULL){
				ROS_WARN("Joint control plugin %s refers to a non-existant joint %s.",
					this->GetHandle().c_str(),
					item.c_str()
				);
			} else{
				this->j_names.push_back(item);
				this->j_poses.push_back(0.0);
			}
		}
		
		//Inform the user that everything is OK.
		ROS_WARN("%s is listening for joint states on %s",
			this->GetHandle().c_str(),
			sdf->GetElement("topicName")->Get<std::string>().c_str()
		);
		
		
		//Start subscribing to joint messages.
		sub = nh->subscribe<sensor_msgs::JointState>(
			sdf->GetElement("topicName")->Get<std::string>(),
			1, 
			std::bind(& set_joint_position_plugin::CB_joint_msg, this, std::placeholders::_1)
		);
		
		ROS_INFO("%s will take control of the following joints:\n",
			this->GetHandle().c_str()
		);
		for(unsigned int i = 0; i < this->j_names.size(); i++){
			ROS_INFO("\t%s", this->j_names[i].c_str());
		}
		
		/*for(unsigned int i = 0; i < this->j_names.size(); i++){
			mod->GetJoint(this->mod->GetName() + "::" + this->j_names[i])->GetChild()->*/
		
		this->updateConnection = event::Events::ConnectWorldUpdateEnd(boost::bind(&set_joint_position_plugin::OnUpdate, this));
	}
	
	void set_joint_position_plugin::OnUpdate(){
		ros::spinOnce();
		for(unsigned int i = 0; i < this->j_names.size(); i++){
			this->mod->GetJoint(this->mod->GetName() + "::" + this->j_names[i])->SetPosition(0, this->j_poses[i]);
			this->mod->GetJoint(this->mod->GetName() + "::" + this->j_names[i])->SetVelocity(0, 0.0);
			//this->mod->GetJoint(this->mod->GetName() + "::" + this->j_names[i])->SetProvideFeedback(false);
		}
	}
	
	void set_joint_position_plugin::CB_joint_msg(const sensor_msgs::JointStateConstPtr & msg){
		for(unsigned int i = 0; i < msg->position.size(); i++){
			bool found = false;
			for(unsigned int j = 0; j < this->j_names.size(); j++){
				//ROS_INFO("Comparing \"%s\" to \"%s\"", this->j_names[j].c_str(), msg->name[i].c_str()
				if(this->j_names[j].compare(msg->name[i]) == 0){
					this->j_poses[j] = msg->position[i];
					found = true;
					break;
				}
			}
			if(!found){
				ROS_WARN("Asked to directly control a non--directly-controllable joint %s",
					msg->name[i].c_str()
				);
			}
		}
	}
	
	GZ_REGISTER_MODEL_PLUGIN(set_joint_position_plugin)
}
