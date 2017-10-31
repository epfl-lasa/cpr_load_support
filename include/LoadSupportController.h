#ifndef __LOADSUPPORTCONTROLLER__
#define __LOADSUPPORTCONTROLLER__

#include "ros/ros.h"
// #include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/TwistStamped.h"
// #include "geometry_msgs/PointStamped.h"
// #include "nav_msgs/Path.h"


#include <vector>

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Point.h"

#include <tf/transform_listener.h>


#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"


#include <sound_play/sound_play.h>


// #include <mutex>

using namespace Eigen;

typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;


const double gravity_acc = 9.80665;

template <typename T> int sign(T val) {
	return (T(0) < val) - (val < T(0));
}

class LoadSupportController {


private:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	double dt_;

	sound_play::SoundClient soundclinet_;

	ros::Subscriber sub_external_wrench_;

	ros::Publisher pub_control_wrench_;
	ros::Publisher pub_desired_equilibrium_;

	std::string speech_statement_;
	std::string speech_statement_last_;

	ros::Time time_to_be_silient_;
	ros::Time time_to_say_something_;

	double talking_rate_;
	double talking_forced_rate_;


	//Controller variables
	double M_object_;
	double loadShare_;


	double Weight_;

	// upper limit for the maximum computed weight
	double MAX_weight_;

	// a height where the object is expected to be
	double Z_ceiling_;
	// a height where the object is supposed to be brought
	double Z_level_;

	// the attractor that this control computes and send as the quilibrium for the admittance control
	Vector3d attractor_;

	// a configuration for arm to go to in case of losing marker
	Vector3d ee_rest_position_;

	// the position of the detected object
	Vector3d object_position_;


	Vector3d Force_control_;


	// INPUT SIGNAL
	// external wrench (force/torque sensor) in "????" frame
	Vector6d wrench_external_;

	// OUTPUT SIGNAL
	// control wrench (from any controller) expected to be in "????" frame
	Vector6d wrench_control_;

	// distance of the object to the end-effector only considering x and y
	double dist_object_ee_ground_;



	tf::TransformListener listener_object_;



public:
	LoadSupportController(ros::NodeHandle &n,
	                      double frequency,
	                      double M_object,
	                      double Z_ceiling,
	                      double Z_level,
	                      double Max_Weitht_compensation,
	                      std::vector<double> ee_rest_position,
	                      std::string topic_wrench_external,
	                      std::string topic_wrench_control,
	                      std::string topic_desired_equilibrium);


	void Run();

private:

	// bool InitializeDS();

	// bool InitializeROS();

	void UpdateExternalForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);

	void ComputeLoadShare();

	void FindObject();

	void SayWhatIsHappining();

	void WeightCompensation();

	void ComputeEquilibriumPoint();

	// void ComputeDesiredVelocity();

	// void PublishDesiredVelocity();

	// void PublishFuturePath();

};


#endif //__LOADSUPPORTCONTROLLER__