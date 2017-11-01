#ifndef __LOADSUPPORTCONTROLLER__
#define __LOADSUPPORTCONTROLLER__

#include <string>
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include "sound_play/sound_play.h"

#include "eigen3/Eigen/Core"
using namespace Eigen;
typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, 6, 1> Vector6d;

// gravity acceleration
const double GR_ACC_ = 9.80665;


class LoadSupportController {

private:

	////////////////////////////////////////////////////////////////
	////////////// ROS variable and communications /////////////////
	////////////////////////////////////////////////////////////////

	// handle of the ROS node
	ros::NodeHandle nh_;

	// the rate of the node
	ros::Rate loop_rate_;

	// a subscriber to receive the external forces
	ros::Subscriber sub_external_wrench_;

	// a publisher to send control forces to admittance controller
	ros::Publisher pub_control_wrench_;

	// a publisher to send the desired equilibrium point to the admittance controller
	ros::Publisher pub_desired_equilibrium_;

	////////////////////////////////////////////////////////////////
	////////////// Load support variables          /////////////////
	////////////////////////////////////////////////////////////////

	// Expected Mass of the object (give variable)
	double M_object_;

	// the external wrench (applied to the end-effector)
	Vector6d wrench_external_;

	// applied weight to the end-effector (z-component of the external force * g)
	double Weight_;

	// Expected Mass of the object (give variable)
	double loadShare_;

	// a value of the load-share that triggers lowering down
	double loadShare_trigger_;

	// a value of the lod-share that the obect is expected to be lowered-down
	double loadShare_final_;

	// a value for the load-share that the robot assume it has contact with the object
	double loadShare_contact_;

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

	// frame_id for the arm tf
	std::string frame_arm_base_;

	////////////////////////////////////////////////////////////////
	////////////// Object tracking variables       /////////////////
	////////////////////////////////////////////////////////////////

	// frame id for the object
	std::string frame_object_;

	// frame id for the end-effector
	std::string frame_arm_endeffector_;

	// a transform listener to loockup for the object in different frames
	tf::TransformListener listener_object_;

	// distance of the object to the end-effector only considering x and y
	double dist_object_ee_ground_;

	// the position of the detected object
	Vector3d object_position_;

	// control from send to the admittance controller
	Vector3d Force_control_;

	////////////////////////////////////////////////////////////////
	////////////// Speech variables         ////////////////////////
	////////////////////////////////////////////////////////////////

	// a sound client for robot to speak
	sound_play::SoundClient soundclinet_;

	// a refractory period for the robot to stay silent after saying something
	double talking_rate_;

	// a period that robot should at least say something
	double talking_forced_rate_;

	// the next time step that robot is allowed to speak
	ros::Time time_to_be_silient_;

	// the next time step that robot has to say something
	ros::Time time_to_say_something_;

	// the speach statement that the robot needs to say
	std::string speech_statement_;

	// the last statement spoken by the robot
	std::string speech_statement_last_;


public:

	// constructor
	LoadSupportController(
	    ros::NodeHandle &n,
	    double frequency,
	    double M_object,
	    double Z_ceiling,
	    double Z_level,
	    double Max_Weitht_compensation,
	    double taking_rate,
	    double talking_force_rate,
	    double loadShare_trigger,
	    double loadShare_final,
	    double loadShare_contact,
	    std::vector<double> ee_rest_position,
	    std::string topic_wrench_external,
	    std::string topic_wrench_control,
	    std::string topic_desired_equilibrium,
	    std::string frame_arm_base,
	    std::string frame_arm_endeffector,
	    std::string frame_object);

	// running the load-support controller
	void Run();

private:

	// receiving the external forces applied to the end-effector
	void UpdateExternalForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);

	// computing the load-share based on forces and expected mass
	void ComputeLoadShare();

	// locating marker/object
	void FindObject();

	// announcing the state of the controller
	void SayWhatIsHappining();

	// compensating for the weight of the object
	void WeightCompensation();

	// computing a desired quilibrium for the admittance controller based on the load-share
	void ComputeEquilibriumPoint();

};

#endif //__LOADSUPPORTCONTROLLER__