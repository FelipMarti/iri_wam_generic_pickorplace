// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _wam_generic_pickorplace_alg_node_h_
#define _wam_generic_pickorplace_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "wam_generic_pickorplace_alg.h"

// [publisher subscriber headers]

// [service client headers]
#include <iri_common_drivers_msgs/QueryJointsMovement.h>
#include <iri_wam_common_msgs/wamInverseKinematics.h>

// [action server client headers]
#include <iri_action_server/iri_action_server.h>
#include <iri_wam_generic_pickorplace/PickOrPlaceAction.h>
#include <iri_common_drivers_msgs/tool_openAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iri_common_drivers_msgs/tool_closeAction.h>

#include <tf/transform_listener.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class WamGenericPickorplaceAlgNode:public algorithm_base::IriBaseAlgorithm <
    WamGenericPickorplaceAlgorithm > {
 private:
	// [publisher attributes]

	// [subscriber attributes]

	// [service attributes]

	// [client attributes]
	ros::ServiceClient wam_joints_pose_client_;
	iri_common_drivers_msgs::QueryJointsMovement wam_joints_pose_srv_;
	ros::ServiceClient get_robot_ik_client_;
	iri_wam_common_msgs::wamInverseKinematics get_robot_ik_srv_;

	// [action server attributes]
	IriActionServer < iri_wam_generic_pickorplace::PickOrPlaceAction >
	    pick_or_place_aserver_;
	void pick_or_placeStartCallback(const
					iri_wam_generic_pickorplace::
					PickOrPlaceGoalConstPtr & goal);
	void pick_or_placeStopCallback(void);
	bool pick_or_placeIsFinishedCallback(void);
	bool pick_or_placeHasSucceedCallback(void);
	void pick_or_placeGetResultCallback(iri_wam_generic_pickorplace::
					    PickOrPlaceResultPtr & result);
	void pick_or_placeGetFeedbackCallback(iri_wam_generic_pickorplace::
					      PickOrPlaceFeedbackPtr &
					      feedback);

	// [action client attributes]
	actionlib::SimpleActionClient <
	    iri_common_drivers_msgs::tool_openAction > open_gripper_client_;
	iri_common_drivers_msgs::tool_openGoal open_gripper_goal_;
	void open_gripperMakeActionRequest();
	void open_gripperDone(const actionlib::SimpleClientGoalState & state, const
			      iri_common_drivers_msgs::tool_openResultConstPtr &
			      result);
	void open_gripperActive();
	void open_gripperFeedback(const
				  iri_common_drivers_msgs::
				  tool_openFeedbackConstPtr & feedback);

	actionlib::SimpleActionClient <
	    iri_common_drivers_msgs::tool_closeAction > close_gripper_client_;
	iri_common_drivers_msgs::tool_closeGoal close_gripper_goal_;
	void close_gripperMakeActionRequest();
	void close_gripperDone(const actionlib::SimpleClientGoalState & state, const
			       iri_common_drivers_msgs::tool_closeResultConstPtr
			       & result);
	void close_gripperActive();
	void close_gripperFeedback(const
				   iri_common_drivers_msgs::
				   tool_closeFeedbackConstPtr & feedback);

	// MY CLASS VARS
	int Pick_or_Place_State;
	int Pick_or_Place_Result;
	bool IsEFOpen;

	// Pick or Place Action variables 
	// 3D position   
	struct point_XYZ {	//or point_RPY
		float X;	//or R
		float Y;	//or P
		float Z;	//or Y
	};
	point_XYZ ini_point;
	point_XYZ grasp_point;	//or ungrasp!
	point_XYZ end_point;
	point_XYZ ini_EF_rpy;
	point_XYZ end_EF_rpy;

	// Is a Pick action, or a Place one?
	bool IsPick;

	//List Pose to Move
	geometry_msgs::PoseStamped list_pose_to_move[3];

 public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
	WamGenericPickorplaceAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
	~WamGenericPickorplaceAlgNode(void);

 protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
	void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
	void node_config_update(Config & config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
	void addNodeDiagnostics(void);

	// [diagnostic functions]

	// [test functions]

	// TODO: Move to wam_generic_pickoeplace_alg
	/* Calc all IK that are needed */
	bool Calc_All_IKs();

	/* My Calc IK FUNCTION */
	bool My_Calc_IK_Function(const geometry_msgs::PoseStamped pose,
				 sensor_msgs::JointState & joints);
	/* My Move WAM FUNCTION */
	void My_Move_WAM_Function(const sensor_msgs::JointState & joints);

};

#endif
