#include "ObjectTracker.h"


ObjectTracker::ObjectTracker(
    ros::NodeHandle &n,
    double frequency)
	: nh_(n),
	  loop_rate_(frequency) {


	ROS_INFO_STREAM("ObjectTracker is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");


	static tf::TransformBroadcaster br;
	tf::StampedTransform transform;

	tf::StampedTransform transform_corner_calibration;
	tf::StampedTransform transform_corner_object;
	tf::StampedTransform transform_base_link;
	tf::StampedTransform transform_corner_base;


	ROS_INFO("Calibration: put the object-marker on top of the imu-sensor and press Enter");
	std::cin.get();

	ros::Duration time_diff;


	bool robots_tf_ready = false;
	bool mocap_tf_ready  = false;
	bool time_robot      = false;
bool corner_base_ready = false;

	while (nh_.ok() && (!robots_tf_ready || !mocap_tf_ready || !time_robot || !corner_base_ready) ) {


		try {
			listener_.lookupTransform("top_right_corner", "top_left_corner",
			                          ros::Time(0), transform_corner_calibration);
			robots_tf_ready = true;
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << "top_right_corner" << " to: " << "top_left_corner" );
		}

		try {
			listener_.lookupTransform("mocap_top_right_corner", "mocap_object",
			                          ros::Time(0), transform_corner_object);
			mocap_tf_ready = true;
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << "mocap_top_right_corner" << " to: " << "mocap_object" );
		}


		try {
			listener_.lookupTransform("base_link", "base_link",
			                          ros::Time(0), transform_base_link);

			time_robot = true;

			time_diff = transform_base_link.stamp_ - ros::Time::now();
			ROS_INFO_STREAM("Time difference:" << time_diff );

		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << "top_right_corner" << " to: " << "top_left_corner" );
		}


		try {
			listener_.lookupTransform("top_right_corner", "base_link",
			                          ros::Time(0), transform_corner_base);

			corner_base_ready = true;

		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << "top_right_corner" << " to: " << "base" );
		}



		ros::spinOnce();
		loop_rate_.sleep();

	}



	tf::Vector3 origin2 = transform_corner_calibration.getOrigin();
	tf::Vector3 origin1 = transform_corner_object.getOrigin();

	tf::Vector3 xyz = origin1.cross(origin2);
	xyz.normalized();
	double w = 1.0 * acos(origin1.dot(origin2) / (origin1.length() * origin2.length()));


	tf::Quaternion quat_offset(xyz, w);
	quat_offset.normalize();





	ROS_INFO("Calibraton finished");


	std::string from_frame = "mocap_top_right_corner";
	std::string to_frame = "mocap_object";

	while (nh_.ok()) {


		try {
			listener_.lookupTransform(from_frame, to_frame,
			                          ros::Time(0), transform);


			tf::Vector3 correct_position = tf::quatRotate(quat_offset, transform.getOrigin());


			transform.setOrigin(correct_position);



			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "top_right_corner", "object"));
			transform.setData(transform_corner_base.inverse() * transform );

			transform.stamp_ = ros::Time::now() + time_diff;
			transform.frame_id_ = "base_link";
			transform.child_frame_id_ = "object";
			br.sendTransform(transform);

		}
		catch (tf::TransformException ex) {
			// rotation_matrix.setZero();
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
			// return false;
		}




		ros::spinOnce();
		loop_rate_.sleep();

	}





}



