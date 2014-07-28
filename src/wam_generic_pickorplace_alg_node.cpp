#include "wam_generic_pickorplace_alg_node.h"

WamGenericPickorplaceAlgNode::WamGenericPickorplaceAlgNode
    (void):algorithm_base::IriBaseAlgorithm <
    WamGenericPickorplaceAlgorithm > (),
pick_or_place_aserver_(public_node_handle_, "pick_or_place"),
open_gripper_client_("open_gripper", true),
close_gripper_client_("close_gripper", true)
{
	//init class attributes if necessary
	Pick_or_Place_State = 0;
	Pick_or_Place_Result = 0;
	IsEFOpen = false;

	//this->loop_rate_ = 2;//in [Hz]

	// [init publishers]

	// [init subscribers]

	// [init services]

	// [init clients]
	wam_joints_pose_client_ =
	    this->public_node_handle_.serviceClient <
	    iri_common_drivers_msgs::QueryJointsMovement > ("wam_joints_pose");
	get_robot_ik_client_ =
	    this->public_node_handle_.serviceClient <
	    iri_wam_common_msgs::wamInverseKinematics > ("get_robot_ik");

	// [init action servers]
	pick_or_place_aserver_.registerStartCallback(boost::bind
						     (&WamGenericPickorplaceAlgNode::
						      pick_or_placeStartCallback,
						      this, _1));
	pick_or_place_aserver_.
	    registerStopCallback(boost::
				 bind(&WamGenericPickorplaceAlgNode::
				      pick_or_placeStopCallback, this));
	pick_or_place_aserver_.
	    registerIsFinishedCallback(boost::
				       bind(&WamGenericPickorplaceAlgNode::
					    pick_or_placeIsFinishedCallback,
					    this));
	pick_or_place_aserver_.
	    registerHasSucceedCallback(boost::
				       bind(&WamGenericPickorplaceAlgNode::
					    pick_or_placeHasSucceedCallback,
					    this));
	pick_or_place_aserver_.
	    registerGetResultCallback(boost::
				      bind(&WamGenericPickorplaceAlgNode::
					   pick_or_placeGetResultCallback, this,
					   _1));
	pick_or_place_aserver_.
	    registerGetFeedbackCallback(boost::
					bind(&WamGenericPickorplaceAlgNode::
					     pick_or_placeGetFeedbackCallback,
					     this, _1));
	pick_or_place_aserver_.start();

	// [init action clients]
}

WamGenericPickorplaceAlgNode::~WamGenericPickorplaceAlgNode(void)
{
	// [free dynamic memory]
}

void WamGenericPickorplaceAlgNode::mainNodeThread(void)
{
	// [fill msg structures]

	// [fill srv structure and make request to the server]

	// [fill action structure and make request to the action server]

	// [publish messages]

	// STATE MACHINE
	// Pick_or_Place_State=0 => Robot ready for a new action 
	// Pick_or_Place_State=1 => Robot goes to the PRE grasp point, or PRE ungrasp
	// Pick_or_Place_State=2 => Robot open or not EF (depending on pick boolean)
	// Pick_or_Place_State=3 => Robot goes to the Grasp point, or Ungrasp point
	// Pick_or_Place_State=4 => Robot open or close EF (depending on pick boolean) 
	// Pick_or_Place_State=5 => Robot goes to the POST grasp point, or POST ungrasp 
	// Pick_or_Place_State=6 => Robot close or not EF (depending on pick boolean) 
	// Pick_or_Place_State=7 => STOP, close EF and stops action
	// Pick_or_Place_State=8 => END OK, robot did Pick and Place correctly! 
	// Pick_or_Place_State=9 => END NOK, robot didn't do Pick and Place correctly 

	if (this->Pick_or_Place_State == 1) {

		sensor_msgs::JointState joints_robot;
		My_Calc_IK_Function(list_pose_to_move[0], joints_robot);
		My_Move_WAM_Function(joints_robot);

		if (Pick_or_Place_State == 1)
			Pick_or_Place_State = 2;

	}
	else if (this->Pick_or_Place_State == 2) {

		if (IsPick)
			open_gripperMakeActionRequest();

		if (Pick_or_Place_State == 2)
			Pick_or_Place_State = 3;

	}
	else if (this->Pick_or_Place_State == 3) {

		sensor_msgs::JointState joints_robot;
		My_Calc_IK_Function(list_pose_to_move[1], joints_robot);
		My_Move_WAM_Function(joints_robot);

		if (Pick_or_Place_State == 3)
			Pick_or_Place_State = 4;

	}
	else if (this->Pick_or_Place_State == 4) {

		if (IsPick)
			close_gripperMakeActionRequest();
		else
			open_gripperMakeActionRequest();

		if (Pick_or_Place_State == 4)
			Pick_or_Place_State = 5;

	}
	else if (this->Pick_or_Place_State == 5) {

		sensor_msgs::JointState joints_robot;
		My_Calc_IK_Function(list_pose_to_move[2], joints_robot);
		My_Move_WAM_Function(joints_robot);

		if (Pick_or_Place_State == 5)
			Pick_or_Place_State = 6;

	}
	else if (this->Pick_or_Place_State == 6) {

		if (!IsPick)
			close_gripperMakeActionRequest();

		Pick_or_Place_Result = 1;
		if (Pick_or_Place_State == 6)
			Pick_or_Place_State = 8;

	}
	else if (this->Pick_or_Place_State == 7) {	//STOP

		if (IsEFOpen)
			close_gripperMakeActionRequest();

		Pick_or_Place_State = 9;

	}

}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */
void WamGenericPickorplaceAlgNode::
pick_or_placeStartCallback(const
			   iri_wam_generic_pickorplace::PickOrPlaceGoalConstPtr
			   & goal)
{
	alg_.lock();
	//check goal 
	//execute goal 

	if (Pick_or_Place_State == 0 || Pick_or_Place_State == 9) {
		//Init var result
		Pick_or_Place_Result = 0;

		//Get all action vars
		ini_point.X = goal->ini_point[0];
		ini_point.Y = goal->ini_point[1];
		ini_point.Z = goal->ini_point[2];

		grasp_point.X = goal->grasp_point[0];	//or ungrasp!
		grasp_point.Y = goal->grasp_point[1];
		grasp_point.Z = goal->grasp_point[2];

		end_point.X = goal->end_point[0];
		end_point.Y = goal->end_point[1];
		end_point.Z = goal->end_point[2];

		ini_EF_rpy.X = goal->ini_EF_rpy[0];
		ini_EF_rpy.Y = goal->ini_EF_rpy[1];
		ini_EF_rpy.Z = goal->ini_EF_rpy[2];

		end_EF_rpy.X = goal->end_EF_rpy[0];
		end_EF_rpy.Y = goal->end_EF_rpy[1];
		end_EF_rpy.Z = goal->end_EF_rpy[2];

		IsPick = goal->pick;

		//All the IK's transformation are calculated to know if the movement is possible
		if (Calc_All_IKs())
			if (goal->execute)
				Pick_or_Place_State = 1;
			else {
				Pick_or_Place_Result = 1;
				Pick_or_Place_State = 8;
			}

		else
			Pick_or_Place_State = 9;

	}

	alg_.unlock();
}

void WamGenericPickorplaceAlgNode::pick_or_placeStopCallback(void)
{
	alg_.lock();
	//stop action
	if (Pick_or_Place_State > 0 && Pick_or_Place_State < 7)
		Pick_or_Place_State = 7;

	alg_.unlock();
}

bool WamGenericPickorplaceAlgNode::pick_or_placeIsFinishedCallback(void)
{
	bool ret = false;

	alg_.lock();
	//if action has finish for any reason 
	if (Pick_or_Place_State == 8 || Pick_or_Place_State == 9) {
		Pick_or_Place_State = 0;
		ret = true;
	}
	alg_.unlock();

	return ret;
}

bool WamGenericPickorplaceAlgNode::pick_or_placeHasSucceedCallback(void)
{
	bool ret = false;

	alg_.lock();
	//if goal was accomplished 
	ret = Pick_or_Place_Result;
	alg_.unlock();

	return ret;
}

void WamGenericPickorplaceAlgNode::
pick_or_placeGetResultCallback(iri_wam_generic_pickorplace::
			       PickOrPlaceResultPtr & result)
{
	alg_.lock();
	//update result data to be sent to client 
	result->successful = Pick_or_Place_Result;
	alg_.unlock();
}

void WamGenericPickorplaceAlgNode::
pick_or_placeGetFeedbackCallback(iri_wam_generic_pickorplace::
				 PickOrPlaceFeedbackPtr & feedback)
{
	alg_.lock();
	//keep track of feedback 
	feedback->PorP_state = Pick_or_Place_State;
	//ROS_INFO("feedback: %s", feedback->data.c_str()); 
	alg_.unlock();
}

/* OPEN GRIPPER ACTION CALLBACKS] */
void WamGenericPickorplaceAlgNode::
open_gripperDone(const actionlib::SimpleClientGoalState & state,
		 const iri_common_drivers_msgs::tool_openResultConstPtr &
		 result)
{
	alg_.lock();
	if (state.toString().compare("SUCCEEDED") == 0) {
		ROS_INFO
		    ("WamGenericPickorplaceAlgNode::open_gripperDone: Goal Achieved!");
		IsEFOpen = true;
	}
	else
		ROS_INFO("WamGenericPickorplaceAlgNode::open_gripperDone: %s",
			 state.toString().c_str());

	//copy & work with requested result 
	alg_.unlock();
}

void WamGenericPickorplaceAlgNode::open_gripperActive()
{
	alg_.lock();
	//ROS_INFO("WamGenericPickorplaceAlgNode::open_gripperActive: Goal just went active!"); 
	alg_.unlock();
}

void WamGenericPickorplaceAlgNode::
open_gripperFeedback(const
		     iri_common_drivers_msgs::tool_openFeedbackConstPtr &
		     feedback)
{
	alg_.lock();
	//ROS_INFO("WamGenericPickorplaceAlgNode::open_gripperFeedback: Got Feedback!"); 

	bool feedback_is_ok = true;

	//analyze feedback 
	//my_var = feedback->var; 

	//if feedback is not what expected, cancel requested goal 
	if (!feedback_is_ok) {
		open_gripper_client_.cancelGoal();
		//ROS_INFO("WamGenericPickorplaceAlgNode::open_gripperFeedback: Cancelling Action!"); 
	}
	alg_.unlock();
}

/* CLOSE GRIPPER ACTION CALLBACKS] */
void WamGenericPickorplaceAlgNode::
close_gripperDone(const actionlib::SimpleClientGoalState & state,
		  const iri_common_drivers_msgs::tool_closeResultConstPtr &
		  result)
{
	alg_.lock();
	if (state.toString().compare("SUCCEEDED") == 0) {
		ROS_INFO
		    ("WamGenericPickorplaceAlgNode::close_gripperDone: Goal Achieved!");
		IsEFOpen = false;
	}
	else
		ROS_INFO("WamGenericPickorplaceAlgNode::close_gripperDone: %s",
			 state.toString().c_str());

	//copy & work with requested result 
	alg_.unlock();
}

void WamGenericPickorplaceAlgNode::close_gripperActive()
{
	alg_.lock();
	//ROS_INFO("WamGenericPickorplaceAlgNode::close_gripperActive: Goal just went active!"); 
	alg_.unlock();
}

void WamGenericPickorplaceAlgNode::
close_gripperFeedback(const
		      iri_common_drivers_msgs::tool_closeFeedbackConstPtr &
		      feedback)
{
	alg_.lock();
	//ROS_INFO("WamGenericPickorplaceAlgNode::close_gripperFeedback: Got Feedback!"); 

	bool feedback_is_ok = true;

	//analyze feedback 
	//my_var = feedback->var; 

	//if feedback is not what expected, cancel requested goal 
	if (!feedback_is_ok) {
		close_gripper_client_.cancelGoal();
		//ROS_INFO("WamGenericPickorplaceAlgNode::close_gripperFeedback: Cancelling Action!"); 
	}
	alg_.unlock();
}

/*  [action requests] */
void WamGenericPickorplaceAlgNode::open_gripperMakeActionRequest()
{
	ROS_INFO
	    ("WamGenericPickorplaceAlgNode::open_gripperMakeActionRequest: Starting New Request!");

	//wait for the action server to start 
	//will wait for infinite time 
	open_gripper_client_.waitForServer();
	ROS_INFO
	    ("WamGenericPickorplaceAlgNode::open_gripperMakeActionRequest: Server is Available!");

	//send a goal to the action 
	//open_gripper_goal_.data = my_desired_goal; 
	open_gripper_client_.sendGoal(open_gripper_goal_,
				      boost::bind
				      (&WamGenericPickorplaceAlgNode::
				       open_gripperDone, this, _1, _2),
				      boost::bind
				      (&WamGenericPickorplaceAlgNode::
				       open_gripperActive, this),
				      boost::bind
				      (&WamGenericPickorplaceAlgNode::
				       open_gripperFeedback, this, _1));
	ROS_INFO
	    ("WamGenericPickorplaceAlgNode::open_gripperMakeActionRequest: Goal Sent. Wait for Result!");
}

void WamGenericPickorplaceAlgNode::close_gripperMakeActionRequest()
{
	ROS_INFO
	    ("WamGenericPickorplaceAlgNode::close_gripperMakeActionRequest: Starting New Request!");

	//wait for the action server to start 
	//will wait for infinite time 
	close_gripper_client_.waitForServer();
	ROS_INFO
	    ("WamGenericPickorplaceAlgNode::close_gripperMakeActionRequest: Server is Available!");

	//send a goal to the action 
	//close_gripper_goal_.data = my_desired_goal; 
	close_gripper_client_.sendGoal(close_gripper_goal_,
				       boost::bind
				       (&WamGenericPickorplaceAlgNode::
					close_gripperDone, this, _1, _2),
				       boost::bind
				       (&WamGenericPickorplaceAlgNode::
					close_gripperActive, this),
				       boost::bind
				       (&WamGenericPickorplaceAlgNode::
					close_gripperFeedback, this, _1));
	ROS_INFO
	    ("WamGenericPickorplaceAlgNode::close_gripperMakeActionRequest: Goal Sent. Wait for Result!");
}

void WamGenericPickorplaceAlgNode::node_config_update(Config & config,
						      uint32_t level)
{
	this->alg_.lock();

	this->alg_.unlock();
}

void WamGenericPickorplaceAlgNode::addNodeDiagnostics(void)
{
}

/*
	MY FUNCTIONS
*/

/* My Calc IK FUNCTION */
bool WamGenericPickorplaceAlgNode::My_Calc_IK_Function(const
						       geometry_msgs::
						       PoseStamped pose,
						       sensor_msgs::
						       JointState & joints)
{

	get_robot_ik_srv_.request.pose = pose;

	//ROS_INFO("TestGraspDeformableAlgNode:: Sending New Request!");
	if (!get_robot_ik_client_.call(get_robot_ik_srv_)) {
		ROS_INFO
		    ("WamGenericPickorplaceAlgNode:: Fail to Call Server topic get_robot_ik");
		return false;
	}

	if (!get_robot_ik_srv_.response.success) {
		ROS_INFO("WamGenericPickorplaceAlgNode:: IK did NOT Succeeded");
		return false;
	}

	joints = get_robot_ik_srv_.response.joints;
	return true;
}

/* My Move WAM FUNCTION */
void WamGenericPickorplaceAlgNode::
My_Move_WAM_Function(const sensor_msgs::JointState & joints)
{

	wam_joints_pose_srv_.request.positions = joints.position;
	wam_joints_pose_srv_.request.velocity = 0.5;
	wam_joints_pose_srv_.request.acceleration = 0.5;

	//ROS_INFO("WamGenericPickorplaceAlgNode:: Sending New POSE to the Robot!");
	if (wam_joints_pose_client_.call(wam_joints_pose_srv_)) {
		//ROS_INFO("WamGenericPickorplaceAlgNode:: Response: %d",
		//                wam_joints_pose_srv_.response.success);
	}
	else {
		ROS_INFO
		    ("WamGenericPickorplaceAlgNode:: Failed to Call Server on topic wam_joints_pose ");
	}

}

/* Calc all IK that are needed to check if it's possible to performance the action */
bool WamGenericPickorplaceAlgNode::Calc_All_IKs()
{

	/* INIT MY VARS */
	//PRE grasp or ungrasp position
	list_pose_to_move[0].pose.position.x = ini_point.X;
	list_pose_to_move[0].pose.position.y = ini_point.Y;
	list_pose_to_move[0].pose.position.z = ini_point.Z;

	//PRE GRASP or PRE ungrasp EF orientation 
	tf::Quaternion q_ini_tf;
	geometry_msgs::Quaternion q_ini;
	q_ini_tf.setRPY(ini_EF_rpy.X, ini_EF_rpy.Y, ini_EF_rpy.Z);
	tf::quaternionTFToMsg(q_ini_tf, q_ini);
	list_pose_to_move[0].pose.orientation = q_ini;

	//GRASP or UNGRASP position
	list_pose_to_move[1].pose.position.x = grasp_point.X;
	list_pose_to_move[1].pose.position.y = grasp_point.Y;
	list_pose_to_move[1].pose.position.z = grasp_point.Z;

	//GRASP or ungrasp EF orientation 
	list_pose_to_move[1].pose.orientation = q_ini;

	//POST grasp or ungrasp position
	list_pose_to_move[2].pose.position.x = end_point.X;
	list_pose_to_move[2].pose.position.y = end_point.Y;
	list_pose_to_move[2].pose.position.z = end_point.Z;

	//POST GRASP or POST ungrasp EF orientation 
	tf::Quaternion q_end_tf;
	geometry_msgs::Quaternion q_end;
	q_end_tf.setRPY(end_EF_rpy.X, end_EF_rpy.Y, end_EF_rpy.Z);
	tf::quaternionTFToMsg(q_end_tf, q_end);
	list_pose_to_move[2].pose.orientation = q_end;

	/// Common parameters
	for (int i = 0; i < 3; i++)
		list_pose_to_move[i].header.frame_id = "estirabot_link_base";

	/* CALC IK's to check if the movement is possible */
	bool IK_OK = true;
	int i = 0;
	sensor_msgs::JointState list_joints_robot[3];

	while (i < 3 && IK_OK) {
		IK_OK =
		    My_Calc_IK_Function(list_pose_to_move[i],
					list_joints_robot[i]);
		i++;
	}
	return IK_OK;

}

/* main function */
int main(int argc, char *argv[])
{
	return algorithm_base::main < WamGenericPickorplaceAlgNode > (argc,
								      argv,
								      "wam_generic_pickorplace_alg_node");
}
