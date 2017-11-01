#include "LoadSupportController.h"

LoadSupportController::LoadSupportController(
    ros::NodeHandle &n,
    double frequency,
    double M_object,
    double Z_ceiling,
    double Z_level,
    double Max_Weitht_compensation,
    double talking_rate,
    double talking_forced_rate,
    double loadShare_trigger,
    double loadShare_final,
    double loadShare_contact,
    std::vector<double> ee_rest_position,
    std::string topic_wrench_external,
    std::string topic_wrench_control,
    std::string topic_desired_equilibrium,
    std::string frame_arm_base,
    std::string frame_arm_endeffector,
    std::string frame_object) :
	nh_(n),
	loop_rate_(frequency),
	M_object_(M_object),
	MAX_weight_(Max_Weitht_compensation),
	ee_rest_position_(ee_rest_position.data()),
	Z_ceiling_(Z_ceiling),
	Z_level_(Z_level),
	talking_rate_(talking_rate),
	talking_forced_rate_(talking_forced_rate),
	frame_arm_base_(frame_arm_base),
	loadShare_trigger_(loadShare_trigger),
	loadShare_final_(loadShare_final),
	loadShare_contact_(loadShare_contact),
	frame_arm_endeffector_(frame_arm_endeffector),
	frame_object_(frame_object) {

	ROS_INFO_STREAM("Load-Support-Controller is created at: "
	                << nh_.getNamespace() << " with freq: " << frequency << "Hz");

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

	// wait for soundplay to start running
	ros::Duration(1).sleep();

	// Say something on the startup
	soundclinet_.say("I am alive! How can I serve you?");

	// wait for the sound to be played
	ros::Duration(2).sleep();

	// next time that the robot is allowed to speak
	time_to_be_silient_ = ros::Time::now() + ros::Duration(talking_rate_);

	// next maximum time that the robot say something
	time_to_say_something_ = ros::Time::now() + ros::Duration(talking_forced_rate_);

	// intializing the sentence for the robot
	speech_statement_ = "I have nothing to say!";

	// intializing the last sentence for the robot
	speech_statement_last_ = speech_statement_;

}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////// The main Control loop  ////////////////////////////// ////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void LoadSupportController::Run() {

	while (nh_.ok() ) {

		// looking for the marker/object, and setting x,y component of the equilibrium
		FindObject();

		// computing how much of the expected Mass is transferred
		ComputeLoadShare();

		// compensating for the transferred mass
		WeightCompensation();

		// setting z-component of the equilibrium based on the load-share
		ComputeEquilibriumPoint();

		// setting z-component of the equilibrium based on the load-share
		SayWhatIsHappining();

		// spinning once (updating the subscriber and publishers)
		ros::spinOnce();

		// sleeping to get the right frequency
		loop_rate_.sleep();
	}

}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Here we comput how of the object is held by the robot ////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void LoadSupportController::ComputeLoadShare() {

	// filtering the z-axis forces to meausre the weight supported by the robot
	Weight_ += 0.1 * ( -wrench_external_(2) - Weight_);

	// Streaming the measured weight
	ROS_INFO_STREAM_THROTTLE(1, "weight:  " <<  Weight_);

	// weights lower than zero are set to 0
	Weight_ = (Weight_ < 0) ? 0 : Weight_;

	// weight higher than Maximum weight are trimmed
	Weight_ = (Weight_ > MAX_weight_) ? MAX_weight_ : Weight_;

	// computing the load-share
	loadShare_ = Weight_ / M_object_ / GR_ACC_;

	// display load-share on the screen every second
	ROS_INFO_STREAM_THROTTLE(1, "load-share " <<  loadShare_);

	// saturating the load-share at 1
	loadShare_ = (loadShare_ > 1) ? 1 : loadShare_;

}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Here, we compensate for the weight of the object and  ////////////////
//////////////////// add a deadzone for the sideway forces in order to not ////////////////
//////////////////// to react to forces created by a tilted object         ////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void LoadSupportController::WeightCompensation() {

	// computing the remainder of the weight for the expected object
	double Weight_missing = (M_object_ * GR_ACC_) - Weight_;

	// missing weight couldn't be negative
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

	// Publishing the control force for the admittance controller
	msg_wrench.header.stamp    = ros::Time::now();
	msg_wrench.header.frame_id = frame_arm_base_;
	msg_wrench.wrench.force.x  = Force_control_(0);
	msg_wrench.wrench.force.y  = Force_control_(1);
	msg_wrench.wrench.force.z  = Force_control_(2);
	msg_wrench.wrench.torque.x = 0;
	msg_wrench.wrench.torque.y = 0;
	msg_wrench.wrench.torque.z = 0;
	pub_control_wrench_.publish(msg_wrench);

}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////// based on the load that the robot is holding, it       ////////////////
//////////////////// computes a new equilibrium point for the admittance   ////////////////
//////////////////// control in order to bring dwon the object.             ////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void LoadSupportController::ComputeEquilibriumPoint() {

	// starting by the rest hieght of the arm
	double att_target = (dist_object_ee_ground_ > 0.2) ? ee_rest_position_(2) : Z_ceiling_;

	// if the load-share is high enought, the robot starts to lower down
	if (loadShare_ > loadShare_trigger_) {

		// computing ratio between trigger and final loadshare
		double beta = (loadShare_ - loadShare_trigger_) / (loadShare_final_ - loadShare_trigger_);

		// the ration shouldn't be more than 1
		beta = (beta > 1) ? 1 : beta;

		// computing the height of the attractor using the ratio
		att_target = (1 - beta) * Z_ceiling_ + beta * Z_level_;

		// informing the user by streaming
		ROS_INFO_STREAM_THROTTLE(1, "Lowering the weight " <<  att_target);
	}

	// filtering the Location of the attractor
	attractor_(2) += 0.005  * (att_target - attractor_(2));

	// the Z-component of the attractor cannot be less than Z_level_
	attractor_(2) = (attractor_(2) < Z_level_  ) ? Z_level_   : attractor_(2);

	// the Z-component of the attractor cannot be more than Z_ceiling_
	attractor_(2) = (attractor_(2) > Z_ceiling_) ? Z_ceiling_ : attractor_(2);

	// Publishing the desired equilibrium for the admittance controller
	geometry_msgs::Point msg_point;
	msg_point.x = attractor_(0);
	msg_point.y = attractor_(1);
	msg_point.z = attractor_(2);
	pub_desired_equilibrium_.publish(msg_point);

}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Here, the robot say the 'speach_statement_' //////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void LoadSupportController::SayWhatIsHappining() {

	// the robot only talks if a refractory period has passed (i.e., talking_rate_)
	if ( (ros::Time::now() - time_to_be_silient_).toSec() > 0  ) {
		if (speech_statement_.compare(speech_statement_last_) ||
		        (ros::Time::now() - time_to_say_something_).toSec() > 0) {

			speech_statement_last_ = speech_statement_;
			soundclinet_.say(speech_statement_);
			ROS_INFO_STREAM_THROTTLE(1, "Talking: " << speech_statement_);

			time_to_be_silient_ = ros::Time::now() + ros::Duration(talking_rate_);
			time_to_say_something_ = ros::Time::now() + ros::Duration(talking_forced_rate_);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////// In this function, the robot look for the marker //////////////////////
//////////////////// and based on the distance of the object to its  //////////////////////
//////////////////// end-effector and the value of the load-share    //////////////////////
//////////////////// it decides to either follow the marker or stay  //////////////////////
//////////////////// put.                                            //////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void LoadSupportController::FindObject() {

	tf::StampedTransform transform;
	Vector3d object_to_ee;

	object_to_ee.setZero();

	// first we read and check the distance of the object with respect to the end-effector of the robot
	try {
		listener_object_.lookupTransform(frame_arm_endeffector_, frame_object_,
		                                 ros::Time(0), transform);

		object_to_ee << transform.getOrigin().x(),
		             transform.getOrigin().y(),
		             transform.getOrigin().z();

		dist_object_ee_ground_ = object_to_ee.segment(0, 2).norm();

		// if the object is too far away, the robot just rests
		if (dist_object_ee_ground_ > 1) {
			speech_statement_ = "The object is too far.";

			// going back to the rest position
			attractor_.segment(0, 2)  += 0.05  * (ee_rest_position_.segment(0, 2) - attractor_.segment(0, 2) );

			if (loadShare_ > loadShare_contact_) {
				speech_statement_ = "The marker is too far.";
			}
		}

		// if the object is in a certain vicinity, the robot tracks it
		else {
			speech_statement_ = "Coming to help.";

			try {
				listener_object_.lookupTransform(frame_arm_base_, frame_object_,
				                                 ros::Time(0), transform);

				object_position_ << transform.getOrigin().x(),
				                 transform.getOrigin().y(),
				                 transform.getOrigin().z();
			}
			catch (tf::TransformException ex) {
				ROS_WARN_STREAM_THROTTLE(1, "Couldn't find transform between"
				                         << frame_arm_base_ << " and " <<  frame_object_);
			}

			// if the robot is already holding the object, it will say this
			if (loadShare_ > loadShare_contact_) {
				speech_statement_ = "Carrying the object";
			}

			// if the robot is very close to the marker, it notifies the user
			if (dist_object_ee_ground_ < .12) {
				speech_statement_ = "Under the marker.";

				// if the robot is very close to the marker AND holding the object, it notifies the user
				if (loadShare_ > loadShare_contact_) {
					speech_statement_ = "Holding the object.";

					// trying to slowly bring the object to the rest x,y position
					attractor_.segment(0, 2)  += 0.01  * (ee_rest_position_.segment(0, 2) - attractor_.segment(0, 2) );
				}
			}
			else {
				// update only the x and y of the attractor based on the location of the object
				attractor_.segment(0, 2)  += 0.05  * (object_position_.segment(0, 2) - attractor_.segment(0, 2) );
			}
		}
	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Couldn't find transform between"
		                         << frame_arm_endeffector_ << " and " <<  frame_object_);
		speech_statement_ = "No marker!";
		dist_object_ee_ground_ = 1e2;
	}

}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////// A callback to receive the external forces   //////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void LoadSupportController::UpdateExternalForce(const geometry_msgs::WrenchStamped::ConstPtr& msg) {

	wrench_external_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
	                 msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

}

// END OF LOADSUPPORTCONTROLLER.CPP