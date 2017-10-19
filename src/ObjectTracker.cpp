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



	ROS_INFO("Calibration: put the object-marker on top of the imu-sensor and press Enter");
	std::cin.get();


	bool robots_tf_ready = false;
	bool mocap_tf_ready  = false;

	while (nh_.ok() && (!robots_tf_ready || !mocap_tf_ready) ) {


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

		ros::spinOnce();
		loop_rate_.sleep();

	}



	tf::Vector3 origin2 = transform_corner_calibration.getOrigin();
	tf::Vector3 origin1 = transform_corner_object.getOrigin();

	tf::Vector3 xyz = origin1.cross(origin2);
	xyz.normalized();
	double w = 1.0 * acos(origin1.dot(origin2)/ (origin1.length() * origin2.length()));


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


			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "top_right_corner", "object"));


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



