#include "LoadSupportController.h"


#include <string>
#include <iostream>


LoadSupportController::LoadSupportController(ros::NodeHandle &n,
        double frequency,
        double M_object,
        double Z_ceiling,
        double Z_level,
        double Max_Weitht_compensation,
        std::vector<double> ee_rest_position,
        std::string topic_wrench_external,
        std::string topic_wrench_control,
        std::string topic_desired_equilibrium)
	: nh_(n),
	  loop_rate_(frequency),
	  M_object_(M_object),
	  MAX_weight_(Max_Weitht_compensation),
	  ee_rest_position_(ee_rest_position.data()),
	  Z_ceiling_(Z_ceiling),
	  Z_level_(Z_level),
	  dt_(1 / frequency) {

	ROS_INFO_STREAM("Load-Support-Controoler is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");

	// a subscriber to get read the external forces
	sub_external_wrench_ = nh_.subscribe(topic_wrench_external, 5,
	                                     &LoadSupportController::UpdateExternalForce, this,
	                                     ros::TransportHints().reliable().tcpNoDelay());

	// a publisher to send the control wrench:
	pub_control_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>(topic_wrench_control, 5);

	// a publisher to send the desired equilibrium point:
	pub_desired_equilibrium_ = nh_.advertise<geometry_msgs::Point>(topic_desired_equilibrium, 5);


	// initializing the sensed weight as zero
	Weight_ = 0;

	// initializing the load-share as zero
	loadShare_ = 0;

	// initializing the control force as zero
	Force_control_.setZero();

	// initializing the attractor at the rest position
	attractor_ = ee_rest_position_;

	// initializing the position of the object at the rest position
	object_position_ = ee_rest_position_;

	//  assuming the object is far away
	dist_object_ee_ground_ = 1e2;

	// a refractory period (e.g 2seconds) between each talking commands
	talking_rate_ = 2;
	// a time-duration that robot should not stay silent more than
	talking_forced_rate_ = 10;

	// intializing some sentence for the robot
	speech_statement_ = "I have nothing to say!";
	speech_statement_last_ = speech_statement_;
}



void LoadSupportController::Run() {

	// wait for soundplay to start running
	ros::Duration(1).sleep();
	// Say something on the startup
	soundclinet_.say("I am alive! How can I serve you?");
	// wait for the sound to be played
	ros::Duration(2).sleep();

	// reseting ROS-Time (not really necessary I guess).
	ros::Time::now();

	// next time that the robot is allowed to speak
	time_to_be_silient_ = ros::Time::now() + ros::Duration(talking_rate_);
	// next maximum time that the robot say something
	time_to_say_something_ = ros::Time::now() + ros::Duration(talking_forced_rate_);


	while (nh_.ok() ) {

		FindObject();

		ComputeLoadShare();

		WeightCompensation();

		ComputeEquilibriumPoint();

		// the robot only talks if a refractory period has passed (i.e., talking_rate_)
		if ( (ros::Time::now() - time_to_be_silient_).toSec() > 0  ) {
			if (speech_statement_.compare(speech_statement_last_) || (ros::Time::now() - time_to_say_something_).toSec() > 0) {
				SayWhatIsHappining();
				time_to_be_silient_ = ros::Time::now() + ros::Duration(talking_rate_);
				time_to_say_something_ = ros::Time::now() + ros::Duration(talking_forced_rate_);
			}
		}

		ros::spinOnce();
		loop_rate_.sleep();
	}

}

// The robot says what is stored in "speech_statement_"
void LoadSupportController::SayWhatIsHappining() {

	// only talk if the robot has something new to say
	// if (speech_statement_.compare(speech_statement_last_) ) {
	speech_statement_last_ = speech_statement_;
	soundclinet_.say(speech_statement_);
	ROS_INFO_STREAM_THROTTLE(1, "Talking: " << speech_statement_);
	// }
}

void LoadSupportController::FindObject() {

	tf::StampedTransform transform;
	Vector3d object_to_ee;

	object_to_ee.setZero();

	// first we read and check the distance of the object with respect to the end-effector of the robot
	try {
		listener_object_.lookupTransform("robotiq_force_torque_frame_id", "object",
		                                 // listener_object_.lookupTransform("mocap_palm", "mocap_object",
		                                 ros::Time(0), transform);

		object_to_ee << transform.getOrigin().x(),
		             transform.getOrigin().y(),
		             transform.getOrigin().z();

		dist_object_ee_ground_ = object_to_ee.segment(0, 2).norm();

		if (dist_object_ee_ground_ > 1) {
			speech_statement_ = "The object is too far.";
			object_position_ = ee_rest_position_;

		}
		else if (dist_object_ee_ground_ < .1) {
			speech_statement_ = "I reached the object.";

			if (loadShare_ > 0.8) {

				object_position_ = ee_rest_position_;

			}

		}
		else {
			speech_statement_ = "I am tracking the marker.";

			try {
				listener_object_.lookupTransform("ur5_arm_base_link", "object",
				                                 ros::Time(0), transform);

				object_position_ << transform.getOrigin().x(),
				                 transform.getOrigin().y(),
				                 transform.getOrigin().z();

			}
			catch (tf::TransformException ex) {
				ROS_WARN_THROTTLE(1, "Couldn't find transform between arm and the object");
			}
		}
	}
	catch (tf::TransformException ex) {
		ROS_WARN_THROTTLE(1, "Couldn't find transform between ee and the object");
		speech_statement_ = "I can not find the marker!";
		dist_object_ee_ground_ = 1e2;
	}

	ROS_INFO_STREAM_THROTTLE(1, "distance to ee: " <<  dist_object_ee_ground_);



	// double dt = loop_rate_.expectedCycleTime().toSec();

	// attractor_ += pow(0.95,1/dt)  * (object_position_ - attractor_);
	// ROS_INFO_STREAM_THROTTLE(1, "dt :" <<  loop_rate_.expectedCycleTime().toSec());

	// update only the x and y of the attractor based on the location of the object
	attractor_.segment(0, 2)  += 0.05  * (object_position_.segment(0, 2) - attractor_.segment(0, 2) );



	// ROS_INFO_STREAM_THROTTLE(1, "X_att:" <<  attractor_ );

}



void LoadSupportController::UpdateExternalForce(const geometry_msgs::WrenchStamped::ConstPtr& msg) {

	wrench_external_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
	                 msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

}


void LoadSupportController::ComputeLoadShare() {

	// filtering the z-axis forces to meausre the weight supported by the robot
	Weight_ += 0.1 * ( -wrench_external_(2) - Weight_);

	ROS_INFO_STREAM_THROTTLE(1, "weight:  " <<  Weight_);



	// constraining the sensed weight between 0 and allowed-maximum of what is expected
	Weight_ = (Weight_ < 0) ? 0 : Weight_;
	Weight_ = (Weight_ > MAX_weight_) ? MAX_weight_ : Weight_;

	// computing the load-share
	loadShare_ = Weight_ / M_object_ / gravity_acc;

	// display load-share on the screen every second
	ROS_INFO_STREAM_THROTTLE(1, "load-share " <<  loadShare_);

	// saturating the load-share at 1
	loadShare_ = (loadShare_ > 1) ? 1 : loadShare_;
}



void LoadSupportController::WeightCompensation() {

	// computing the remainder of the weight for the expected object
	double Weight_missing = (M_object_ * gravity_acc) - Weight_;
	Weight_missing = (Weight_missing < 0) ? 0 : Weight_missing;

	// forces applied side-way to the end-effector
	double Weight_sidway = wrench_external_.segment(0, 2).norm();

	// by default we cancel all external forces
	Force_control_ = -wrench_external_.segment(0, 3);

	// if the side-way forces are more than missing weight, we just add a deadzone
	if ( Weight_sidway > Weight_missing) {
		Force_control_ = (Weight_missing / Weight_sidway) * Force_control_;
	}

	// simply compensating the weight by applying the exact opposite force
	Force_control_(2) = Weight_;

	// sending the control Force over the corresponding ROS-topic
	geometry_msgs::WrenchStamped msg_wrench;

	msg_wrench.header.stamp    = ros::Time::now();
	msg_wrench.header.frame_id = "ur5_arm_base_link";
	msg_wrench.wrench.force.x  = Force_control_(0);
	msg_wrench.wrench.force.y  = Force_control_(1);
	msg_wrench.wrench.force.z  = Force_control_(2);
	msg_wrench.wrench.torque.x = 0;
	msg_wrench.wrench.torque.y = 0;
	msg_wrench.wrench.torque.z = 0;
	pub_control_wrench_.publish(msg_wrench);

}





void LoadSupportController::ComputeEquilibriumPoint() {



	// starting by the rest hieght of the arm
	double att_target = (dist_object_ee_ground_ > 0.2) ? ee_rest_position_(2) : Z_ceiling_;

	if (loadShare_ > 0.8) {
		double beta = (loadShare_ - 0.8) / 0.1;
		beta = (beta > 1) ? 1 : beta;
		att_target = (1 - beta) * Z_ceiling_ + beta * Z_level_;
		ROS_INFO_STREAM_THROTTLE(1, "Lowering the weight " <<  att_target);
	}


	// filtering the Location of the attractor
	double att_err = att_target - attractor_(2);

	// attractor_(2) += ((att_err > 0) ? 0.001 : 0.0005 ) * att_err;
	attractor_(2) += 0.01  * att_err;

	// saturating the Z_attractor
	attractor_(2) = (attractor_(2) < Z_level_  ) ? Z_level_   : attractor_(2);
	attractor_(2) = (attractor_(2) > Z_ceiling_) ? Z_ceiling_ : attractor_(2);

	// sending the desired attractor over the corresponding ROS-topic
	geometry_msgs::Point msg_point;
	msg_point.x = attractor_(0);
	msg_point.y = attractor_(1);
	msg_point.z = attractor_(2);
	pub_desired_equilibrium_.publish(msg_point);

}
