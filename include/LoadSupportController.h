#ifndef __LOADSUPPORTCONTROLLER__
#define __LOADSUPPORTCONTROLLER__

#include "ros/ros.h"
// #include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/TwistStamped.h"
// #include "geometry_msgs/PointStamped.h"
// #include "nav_msgs/Path.h"


// #include <vector>

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Point.h"


#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"


// #include <mutex>

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;


const double gravity_acc = 9.80665;


class LoadSupportController {


private:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	double dt_;

	ros::Subscriber sub_external_wrench_;

	ros::Publisher pub_control_wrench_;
	ros::Publisher pub_desired_equilibrium_;


	//Controller variables
	double M_object_;
	double M_estiamted_;
	double loadShare_;

	double ForceZ_;

	// a height where the object is expected to be
	double Z_ceiling_;
	// a height where the object is supposed to be brought
	double Z_level_;
	// current attractor of the controller, a mixture of the two above
	double attractor_;


	// INPUT SIGNAL
	// external wrench (force/torque sensor) in "????" frame
	Vector6d wrench_external_;

	// OUTPUT SIGNAL
	// control wrench (from any controller) expected to be in "????" frame
	Vector6d wrench_control_;


public:
	LoadSupportController(ros::NodeHandle &n,
	                      double frequency,
	                      double M_object,
	                      double Z_ceiling,
	                      double Z_level,
	                      std::string topic_wrench_external,
	                      std::string topic_wrench_control,
	                      std::string topic_desired_equilibrium);


	// void Run();

private:

	// bool InitializeDS();

	// bool InitializeROS();

	void UpdateExternalForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);

	void ComputeLoadShare();

	// void ComputeDesiredVelocity();

	// void PublishDesiredVelocity();

	// void PublishFuturePath();

};


#endif //__LOADSUPPORTCONTROLLER__