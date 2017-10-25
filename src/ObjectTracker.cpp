#include "ObjectTracker.h"


ObjectTracker::ObjectTracker(
    ros::NodeHandle &n,
    double frequency,
    std::string frame_robot_static,
    std::string frame_robot_calibration,
    std::string frame_robot_object,
    std::string frame_robot_base,
    std::string frame_mocap_static,
    std::string frame_mocap_object)
	: nh_(n),
	  loop_rate_(frequency),
	  frame_robot_static_(frame_robot_static),
	  frame_robot_calibration_(frame_robot_calibration),
	  frame_robot_object_(frame_robot_object),
	  frame_robot_base_(frame_robot_base),
	  frame_mocap_static_(frame_mocap_static),
	  frame_mocap_object_(frame_mocap_object) {

	ROS_INFO_STREAM("ObjectTracker is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");

	tf::StampedTransform transform_base_link;
	tf::StampedTransform transform_static_calibration;
	tf::StampedTransform transform_static_object;

	bool ready_calibration = false;
	bool ready_mocap = false;
	bool ready_base = false;
	bool ready_static = false;

	quat_offset_ = tf::createQuaternionFromRPY(0, 0, 0);


	ROS_INFO_STREAM("Calibration: match the object-marker with " << frame_robot_calibration_ << " in rviz and press Enter");
	std::cin.get();


	while (nh_.ok() && (!ready_calibration || !ready_mocap || !ready_base || !ready_static) ) {

		try {
			listener_.lookupTransform(frame_robot_base_, frame_robot_base_,
			                          ros::Time(0), transform_base_link);

			ready_base = true;

			time_diff_ = transform_base_link.stamp_ - ros::Time::now();
			ROS_WARN_STREAM_THROTTLE(10,"Time difference:" << time_diff_ );

		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for base_link frame." );
		}

		try {
			listener_.lookupTransform(frame_robot_static_, frame_robot_base_,
			                          ros::Time(0), transform_static_base_);

			ready_static = true;

		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << frame_robot_static_ << " to: " << "base" );
		}


		try {
			listener_.lookupTransform(frame_robot_static_, frame_robot_calibration_,
			                          ros::Time(0), transform_static_calibration);
			ready_calibration = true;
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " <<
			                         frame_robot_static_ << " to: "
			                         << frame_robot_calibration_ );
		}

		try {
			listener_.lookupTransform(frame_mocap_static_, frame_mocap_object_,
			                          ros::Time(0), transform_static_object);
			ready_mocap = true;
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: "
			                         << frame_mocap_static_ << " to: "
			                         << frame_mocap_object_ );
		}

		ros::spinOnce();
		loop_rate_.sleep();

	}


	tf::Vector3 origin2 = transform_static_calibration.getOrigin();
	tf::Vector3 origin1 = transform_static_object.getOrigin();

	tf::Vector3 xyz = origin1.cross(origin2);
	xyz.normalized();
	double w = 1.0 * acos(origin1.dot(origin2) / (origin1.length() * origin2.length()));

	tf::Quaternion q(xyz, w);
	q.normalize();
	quat_offset_ = q;



	ROS_INFO_STREAM("Calibraton is finished. Quaternion offset" << quat_offset_);

	// tf::StampedTransform transform;


	// while (nh_.ok()) {

	// 	try {
	// 		listener_.lookupTransform(frame_mocap_static_, frame_mocap_object_,
	// 		                          ros::Time(0), transform);

	// 		tf::Vector3 correct_position = tf::quatRotate(quat_offset_, transform.getOrigin());

	// 		transform.setOrigin(correct_position);

	// 		// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_robot_static_, "object"));
	// 		transform.setData(transform_static_base_.inverse() * transform );

	// 		transform.stamp_ = ros::Time::now() + time_diff_;
	// 		transform.frame_id_ = frame_robot_base_;
	// 		transform.child_frame_id_ = frame_robot_object_;
	// 		brodcaster_.sendTransform(transform);

	// 	}
	// 	catch (tf::TransformException ex) {
	// 		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: "
	// 		                         << frame_mocap_static_ << " to: "
	// 		                         << frame_mocap_object_ );
	// 	}

	// 	ros::spinOnce();
	// 	loop_rate_.sleep();

	// }

}


void ObjectTracker::Run() {

	tf::StampedTransform transform;



	while (nh_.ok()) {

		try {
			listener_.lookupTransform(frame_mocap_static_, frame_mocap_object_,
			                          ros::Time(0), transform);

			tf::Vector3 correct_position = tf::quatRotate(quat_offset_, transform.getOrigin());

			transform.setOrigin(correct_position);

			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_robot_static_, "object"));
			transform.setData(transform_static_base_.inverse() * transform );

			transform.stamp_ = ros::Time::now() + time_diff_;
			transform.frame_id_ = frame_robot_base_;
			transform.child_frame_id_ = frame_robot_object_;
			brodcaster_.sendTransform(transform);

		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: "
			                         << frame_mocap_static_ << " to: "
			                         << frame_mocap_object_ );
		}

		ros::spinOnce();
		loop_rate_.sleep();

	}
}

