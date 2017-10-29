#include "LoadSupportController.h"


#include <string>
#include <iostream>


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


	ros::Duration(1).sleep(); // wait for soundplay to start running

	soundclinet_.say("I am alive!");
	ros::Duration(2).sleep(); // wait for soundplay to start running



	loadShare_ = 0;
	M_estiamted_ = 0;
	ForceZ_ = 0;

	X_attractor_ = 0.111;
	Y_attractor_ = 0.494;

	Z_attractor_ = Z_ceiling_;

	state_marker_old_ = "close";
	state_marker_new_ = "lost";

	ros::Time::now();
	talking_rate_ = 2;
	repeating_rate_ = 10;
	time_talk_ = ros::Time::now() + ros::Duration(talking_rate_);
	time_repeat_ = ros::Time::now() + ros::Duration(repeating_rate_);



	while (nh_.ok() ) {

		FindObject();

		ComputeLoadShare();

		ros::spinOnce();
		loop_rate_.sleep();
	}

}

void LoadSupportController::FindObject() {

	tf::StampedTransform transform;
	tf::Vector3 ee_to_obejct;
	ee_to_obejct.setZero();

	if (true) {
		try {
			// listener_object_.lookupTransform("robotiq_force_torque_frame_id", "object",
			listener_object_.lookupTransform("mocap_palm", "mocap_object",
			                                 ros::Time(0), transform);

			ee_to_obejct =  transform.getOrigin();

			if (ee_to_obejct.length() > 1) {
				state_marker_new_ = "far";
			}
			else if (ee_to_obejct.length() < .1) {
				state_marker_new_ = "close";
			}
			else {
				state_marker_new_ = "tracking";
			}


		}
		catch (tf::TransformException ex) {
			ROS_WARN_THROTTLE(1, "Couldn't find transform between ee and the object");
			state_marker_new_ = "lost";

		}

		ROS_INFO_STREAM_THROTTLE(1, "distance to ee: " <<  ee_to_obejct.length());
		ROS_INFO_STREAM_THROTTLE(1, "state flag: " <<  state_marker_new_ << " the old one: " << state_marker_old_);


	}

	// ROS_INFO_STREAM((ros::Time::now() - time_talk_));

	if (state_marker_new_.compare(state_marker_old_) && (ros::Time::now() - time_talk_).toSec() > 0  ) {

		state_marker_old_ = state_marker_new_;
		time_talk_ = ros::Time::now() + ros::Duration(talking_rate_);



		if (!state_marker_new_.compare("lost")) {
			soundclinet_.say("I cannot see the marker!");
			ROS_WARN_THROTTLE(1, "............ In the lost parT!");
		}
		if (!state_marker_new_.compare("far")) {
			soundclinet_.say("The marker is too far!");
			ROS_INFO_STREAM_THROTTLE(1, "............  The marker is too far away!");
		}
		if (!state_marker_new_.compare("close")) {
			soundclinet_.say("I reach the marker!");
			ROS_INFO_STREAM_THROTTLE(1, " ............  I reached the marker");
		}
		if (!state_marker_new_.compare("tracking")) {
			soundclinet_.say("I am tracking the marker!");
			ROS_INFO_STREAM_THROTTLE(1, " ............  I am tracking the marker");
		}


	}

	if (!state_marker_new_.compare("tracking") && (ros::Time::now() - time_repeat_).toSec() > 0  ) {
	time_repeat_ = ros::Time::now() + ros::Duration(repeating_rate_);

		soundclinet_.say("I am still tracking the marker!");
		ROS_INFO_STREAM_THROTTLE(1, " ............  I am tracking the marker");
	}










	// if (true) {
	// 	try {
	// 		listener_object_.lookupTransform("ur5_arm_base_link", "object",
	// 		                                 ros::Time(0), transform);

	// 		target_x_ = transform.getOrigin().x();
	// 		target_y_ = transform.getOrigin().y();

	// 	}
	// 	catch (tf::TransformException ex) {
	// 		ROS_WARN_THROTTLE(1, "Couldn't find transform between arm and the object");
	// 		target_x_ = 0.111;
	// 		target_y_ = 0.494;
	// 	}
	// }



	// // if (abs(target_x_ - X_attractor_) < 0.15 && abs(target_y_ - Y_attractor_) < 0.15) {
	// // 	X_attractor_ += 0.1 * (0.111 - X_attractor_);
	// // 	Y_attractor_ += 0.1 * (0.494 - Y_attractor_);
	// // }

	// double dist_to_cent = (target_x_ - 0.111 ) * (target_x_ - 0.111 ) +
	//                       (target_y_ - 0.494) * (target_y_ - 0.494);
	// dist_to_cent = sqrt(dist_to_cent);


	// if (dist_to_cent > 0.3) {
	// 	X_attractor_ += 0.005 * (0.111 - X_attractor_);
	// 	Y_attractor_ += 0.005 * (0.494 - Y_attractor_);
	// 	ROS_INFO_STREAM_THROTTLE(1, "Target is far!");

	// }
	// else if (dist_to_cent > 0.05) {

	// 	X_attractor_ += 0.005 * (target_x_ - X_attractor_);
	// 	Y_attractor_ += 0.005 * (target_y_ - Y_attractor_);
	// 	ROS_INFO_STREAM_THROTTLE(1 , "Target is close!");

	// }






	// if (abs(target_x_ - 0.111)  < 0.3 && abs(target_y_ - 0.494)  < 0.3) {


	// 	ROS_INFO_STREAM_THROTTLE(1, "Target is close!");

	// }
	// else {
	// 	X_attractor_ += 0.1 * (0.111 - X_attractor_);
	// 	Y_attractor_ += 0.1 * (0.494 - Y_attractor_);
	// 	ROS_INFO_STREAM_THROTTLE(1, "Target is far!");

	// }


	// if (abs(error_x) < 0.5  && abs(error_y) < 0.5) {
	// 	if (abs(error_x) > 0.1  && abs(error_y) > 0.1) {


	// 	}
	// }



	ROS_INFO_STREAM_THROTTLE(1, "X_att:" <<  X_attractor_ << " Y_att:"  << Y_attractor_ );

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
	if (missing_force < 0) {
		missing_force = 0;
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

		ROS_INFO_STREAM_THROTTLE(1, "Lowering the weight " <<  Z_attractor_);
	}
	else {
		att_target = Z_ceiling_;
	}

	if (Z_attractor_ < Z_level_) {
		Z_attractor_ = Z_level_;
	}


	double att_err = att_target - Z_attractor_;

	if (att_err > 0) {
		Z_attractor_  += 0.001 * att_err;

	}
	else {
		Z_attractor_  += 0.0005 * att_err;

	}


	geometry_msgs::Point msg_point;
	msg_point.x = X_attractor_;
	msg_point.y = Y_attractor_;
	msg_point.z = Z_attractor_;
	pub_desired_equilibrium_.publish(msg_point);





	// wrench_external_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
	//                  msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

	// std::cout << wrench_external_(2) << std::endl;

}

// // Filter and update
// wrench_external_ <<  (1 - wrench_filter_factor_) * wrench_external_ +
//                  wrench_filter_factor_ * rotation_ft_base * wrench_ft_frame;