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
	sub_external_wrench_ = nh_.subscribe(wrench_external_topic_name, 5,
	                                     &LoadSupportController::UpdateExternalForce, this,
	                                     ros::TransportHints().reliable().tcpNoDelay());

	// Publishers:
	pub_control_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>(wrench_control_topic_name, 5);



	loadShare_ = 0;
	M_estiamted_ = 0;
	ForceZ_ = 0;

	while (nh_.ok() ) {

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


	double target_force = - wrench_external_(2);

	double Fz_error =   target_force - ForceZ_;

	if (Fz_error > 0) {
		ForceZ_ += 0.1 * Fz_error;
	}
	else {
		ForceZ_ += 0.2 * Fz_error;
	}

	if (ForceZ_ < 0) {
		ForceZ_ = 0;
	}

	if (ForceZ_ > 2*M_object_*gravity_acc) {
		ForceZ_ = 2*M_object_*gravity_acc;
	}




	M_estiamted_ = ForceZ_ / gravity_acc;
	if (M_estiamted_ < 0) {
		M_estiamted_ = 0;
	}

	loadShare_ = M_estiamted_ / M_object_;

	if (loadShare_ > 1) {
		ROS_WARN_STREAM_THROTTLE(1, "Overload, load-share: " <<  loadShare_);
		loadShare_ = 1;
	}

	ROS_INFO_STREAM_THROTTLE(1, "load-share " <<  loadShare_);








	geometry_msgs::WrenchStamped msg_wrench;

	msg_wrench.header.stamp    = ros::Time::now();
	msg_wrench.header.frame_id = "ur5_arm_base_link";
	msg_wrench.wrench.force.x  = 0;
	msg_wrench.wrench.force.y  = 0;
	msg_wrench.wrench.force.z  = ForceZ_;
	msg_wrench.wrench.torque.x = 0;
	msg_wrench.wrench.torque.y = 0;
	msg_wrench.wrench.torque.z = 0;
	pub_control_wrench_.publish(msg_wrench);






	// wrench_external_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
	//                  msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

	// std::cout << wrench_external_(2) << std::endl;

}

// // Filter and update
// wrench_external_ <<  (1 - wrench_filter_factor_) * wrench_external_ +
//                  wrench_filter_factor_ * rotation_ft_base * wrench_ft_frame;