#include "LoadSupportController.h"




LoadSupportController::LoadSupportController(ros::NodeHandle &n,
        double frequency,
        double M_object,
        std::string wrench_external_topic_name,
        std::string wrench_control_topic_name)
	: nh_(n),
	  loop_rate_(frequency),
	  M_object_(M_object),
	  wrench_external_topic_name_(wrench_external_topic_name),
	  wrench_control_topic_name_(wrench_control_topic_name),
	  dt_(1 / frequency) {

	ROS_INFO_STREAM("Load-Support-Controoler is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");

	// Subscribers:
	sub_platform_state_ = nh_.subscribe(wrench_external_topic_name, 5,
	                                    &LoadSupportController::UpdateExternalForce, this,
	                                    ros::TransportHints().reliable().tcpNoDelay());
	loadShare_ = 0;
	M_estiamted_ = 0;

	while (nh_.ok() ) {
		ROS_WARN_THROTTLE(5, "Running the nod");

		ComputeLoadShare();

		ros::spinOnce();
		loop_rate_.sleep();
	}

}



void LoadSupportController::UpdateExternalForce(const geometry_msgs::WrenchStamped::ConstPtr& msg) {

	wrench_external_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
	                 msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

	// ROS_INFO_STREAM_THROTTLE(1,
	//                          " M.x: "  << wrench_external_(0) / gravity_acc <<
	//                          " M.y: "  << wrench_external_(1) / gravity_acc <<
	//                          " M.z: "  << wrench_external_(2) / gravity_acc );
}


void LoadSupportController::ComputeLoadShare() {

	M_estiamted_ = - wrench_external_(2) / gravity_acc;
	if (M_estiamted_ < 0) {
		M_estiamted_ = 0;
	}

	loadShare_ = M_estiamted_ / M_object_;

	if (loadShare_ > 1) {
		ROS_WARN_STREAM_THROTTLE(1, "Overload, load-share: " <<  loadShare_);
		loadShare_ = 1;
	}

	ROS_INFO_STREAM_THROTTLE(1, "load-share " <<  loadShare_);



	// wrench_external_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
	//                  msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

	// std::cout << wrench_external_(2) << std::endl;

}

// // Filter and update
// wrench_external_ <<  (1 - wrench_filter_factor_) * wrench_external_ +
//                  wrench_filter_factor_ * rotation_ft_base * wrench_ft_frame;