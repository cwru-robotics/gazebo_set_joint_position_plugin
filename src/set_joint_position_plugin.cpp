
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
		RCLCPP_INFO(this->nh->get_logger(),
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
				"%s", this->j_names[i].c_str()
			);
		}
		
		this->last_time = this->nh->now();
		
		this->updateConnection = event::Events::ConnectWorldUpdateEnd(boost::bind(&set_joint_position_plugin::OnUpdate, this));
	}
	
	void set_joint_position_plugin::OnUpdate(){
		for(unsigned int i = 0; i < this->j_names.size(); i++){
			this->mod->SetJointPosition(this->mod->GetName() + "::" + this->j_names[i], this->j_poses[i]);
		}
	}
	
	void set_joint_position_plugin::CB_joint_msg(const sensor_msgs::msg::JointState::SharedPtr msg){
		rclcpp::Duration delta = this->last_time - this->nh->now();
		this->last_time = this->nh->now();
		
		for(unsigned int i = 0; i < msg->velocity.size(); i++){
			for(unsigned int j = 0; j < this->j_names.size(); j++){
				if(this->j_names[j].compare(msg->name[i]) == 0){
					this->j_poses[j] += (delta.seconds() + 0.000001 * delta.nanoseconds()) * msg->velocity[i];
					break;
				}
			}
			RCLCPP_WARN(this->nh->get_logger(),
				"Asked to directly control a non--directly-controllable joint %s",
				msg->name[i].c_str()
			);
		}
	}
//GZ_REGISTER_MODEL_PLUGIN(set_joint_position_plugin);

/*////////////////////////////////////////////////////////////////////////////////
// Constructor
set_joint_position_plugin::set_joint_position_plugin()
{

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
set_joint_position_plugin::~set_joint_position_plugin()
{
    event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

    // Custom Callback Queue
    this->queue_.clear();
    this->queue_.disable();
    this->rosnode_->shutdown();
    this->callback_queue_thread_.join();
    this->joints_list.clear();
    this->sub_.shutdown();


    delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void set_joint_position_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{


    ROS_INFO("####################################");
    ROS_INFO("INITIALIZING SET_JOINT_POSITION PLUGIN");


    // load parameters
    this->robot_namespace_ = "";

    if ( _sdf->HasElement("robotNamespace") ) this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";


    if ( !_sdf->HasElement("topicName") )
    {
        ROS_FATAL("force plugin missing <topicName>, cannot proceed");
        return;
    }
    else this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();


    // Make sure the ROS node for Gazebo has already been initialized
    if ( !ros::isInitialized() )
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    ROS_INFO_STREAM("topicName:" << this->topic_name_);

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);


    this->sub_ = this->rosnode_->subscribe( this->topic_name_, 1, &set_joint_position_plugin::CallBackMethod, this, ros::TransportHints().tcpNoDelay() );


    // Custom Callback Queue
    this->callback_queue_thread_ = boost::thread( boost::bind(&set_joint_position_plugin::QueueThread, this) );

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&set_joint_position_plugin::UpdateChild, this) );*/


    /* Checking the list of joints */
    /*this->joints_list = _model->GetJoints();

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void set_joint_position_plugin::CallBackMethod(const sensor_msgs::JointState _msg)
{
    this->lock_.lock();
    //  ROS_INFO_STREAM( "Getting a new position" );
    set_joint_state_ =  _msg;
    ROS_INFO_STREAM( "Getting a new position of size: "<< set_joint_state_.position.size() );


    for (int i = 0; i < set_joint_state_.position.size();i++){
        int j = i+1; // use next robot joint
#if GAZEBO_MAJOR_VERSION == 2
        this->joints_list[j]->SetAngle(0, set_joint_state_.position[i]);
#else // if GAZEBO_MAJOR_VERSION != 2
        this->joints_list[j]->SetPosition(0, set_joint_state_.position[i]);
        ROS_INFO_STREAM( "Joint: "<< j << ", name: " << this->joints_list[j]->GetName());
#endif // if GAZEBO_MAJOR_VERSION == 2
    }
    this->lock_.unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void set_joint_position_plugin::UpdateChild()
{
    this->lock_.lock();

//    for (int i = 0; i < set_joint_state_.position.size();i++){
//        int j = i+1; // use next robot joint
//#if GAZEBO_MAJOR_VERSION == 2
//        this->joints_list[j]->SetAngle(0, set_joint_state_.position[i]);
//#else // if GAZEBO_MAJOR_VERSION != 2
//        this->joints_list[j]->SetPosition(0, set_joint_state_.position[i]);
//        ROS_INFO_STREAM( "Joint: "<< j << ", name: " << this->joints_list[j]->GetName());
//#endif // if GAZEBO_MAJOR_VERSION == 2
//    }

    this->lock_.unlock();

}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void set_joint_position_plugin::QueueThread()
{
    static const double timeout = 0.01;

    while ( this->rosnode_->ok() )
    {
        this->queue_.callAvailable( ros::WallDuration(timeout) );
    }
}*/
}
