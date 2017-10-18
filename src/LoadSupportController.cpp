#include "LoadSupportController.h"




LoadSupportController::LoadSupportController(ros::NodeHandle &n,
        double frequency,
        double M_object,
        double Z_ceiling,
        double Z_level,
        std::string topic_wrench_external,
        std::string topic_wrench_control,
        std::string topic_desired_equilibrium)
	: nh_(n),
	  loop_rate_(frequency),
	  M_object_(M_object),
	  Z_ceiling_(Z_ceiling),
	  Z_level_(Z_level),
	  dt_(1 / frequency) {

	ROS_INFO_STREAM("Load-Support-Controoler is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");

	// Subscribers:
	sub_external_wrench_ = nh_.subscribe(topic_wrench_external, 5,
	                                     &LoadSupportController::UpdateExternalForce, this,
	                                     ros::TransportHints().reliable().tcpNoDelay());

	// Publishers:
	pub_control_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>(topic_wrench_control, 5);

	pub_desired_equilibrium_ = nh_.advertise<geometry_msgs::Point>(topic_desired_equilibrium, 5);



	loadShare_ = 0;
	M_estiamted_ = 0;
	ForceZ_ = 0;

	attractor_ = Z_ceiling_;

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


	double side_force_x = -wrench_external_(0);
	double side_force_y = -wrench_external_(1);
	double target_force = -wrench_external_(2);




	double Fz_error =   target_force - ForceZ_;

	ForceZ_ += 0.1 * Fz_error;



	if (ForceZ_ < 0) {
		ForceZ_ = 0;
	}

	if (ForceZ_ > 2 * M_object_ * gravity_acc) {
		ForceZ_ = 2 * M_object_ * gravity_acc;
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



	double missing_force = (M_object_ - M_estiamted_) * gravity_acc;
	if(missing_force<0){
		missing_force=0;
	}



	double ForceX = 0;
	double ForceY = 0;

	if (abs(side_force_x) < missing_force) {
		ForceX = side_force_x;
	}
	else {
		ForceX = - sign(side_force_x) * missing_force;
	}

	if (abs(side_force_y) < missing_force) {
		ForceY = side_force_y;
	}
	else {
		ForceY = - sign(side_force_y) * missing_force;
	}






	geometry_msgs::WrenchStamped msg_wrench;

	msg_wrench.header.stamp    = ros::Time::now();
	msg_wrench.header.frame_id = "ur5_arm_base_link";
	msg_wrench.wrench.force.x  = ForceX;
	msg_wrench.wrench.force.y  = ForceY;
	msg_wrench.wrench.force.z  = ForceZ_;
	msg_wrench.wrench.torque.x = 0;
	msg_wrench.wrench.torque.y = 0;
	msg_wrench.wrench.torque.z = 0;
	pub_control_wrench_.publish(msg_wrench);

	double att_target = 0;

	if (loadShare_ > 0.8) {
		double beta = (loadShare_ - 0.8) / 0.1;

		att_target = (1 - beta) * Z_ceiling_ + beta * Z_level_;

		ROS_INFO_STREAM_THROTTLE(1, "Lowering the weight " <<  attractor_);
	}
	else {
		att_target = Z_ceiling_;
	}

	if (attractor_ < Z_level_) {
		attractor_ = Z_level_;
	}


	double att_err = att_target - attractor_;

	if (att_err > 0) {
		attractor_  += 0.001 * att_err;

	}
	else {
		attractor_  += 0.0005 * att_err;

	}


	geometry_msgs::Point msg_point;
	msg_point.x = 0.111;
	msg_point.y = 0.494;
	msg_point.z = attractor_;
	pub_desired_equilibrium_.publish(msg_point);





	// wrench_external_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
	//                  msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

	// std::cout << wrench_external_(2) << std::endl;

}

// // Filter and update
// wrench_external_ <<  (1 - wrench_filter_factor_) * wrench_external_ +
//                  wrench_filter_factor_ * rotation_ft_base * wrench_ft_frame;