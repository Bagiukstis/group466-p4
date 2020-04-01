#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>



class Movements{

  public:

    void moveHome(){
      static const std::string PLANNING_GROUP = "arm";

      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      moveit::planning_interface::MoveGroupInterface::Plan planHome;
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      //Joint angles for the three main finger joints
      joint_group_positions[0] = 0;
      joint_group_positions[1] = 3.1;
      joint_group_positions[2] = 0;
      joint_group_positions[3] = 1.5;
      joint_group_positions[4] = 0;
      joint_group_positions[5] = 4.6;
      joint_group_positions[6] = 0;
      move_group.setJointValueTarget(joint_group_positions);

      bool success = (move_group.plan(planHome) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      //EXECUTE TRAJECTORY
      move_group.move();
    }


  void jointPlan(std::string movement_group, float x_pos, float y_pos, float z_pos, float orient){
    static const std::string PLANNING_GROUP = movement_group;

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //GOAL POSITION
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.w = orient;
    goal_pose.position.x = x_pos;
    goal_pose.position.y = y_pos;
    goal_pose.position.z = z_pos;
    move_group.setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan planJoint;

    bool success = (move_group.plan(planJoint) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //EXECUTE TRAJECTORY
    move_group.move();
  }



  void cartesianPlan(std::string movement_group, float x_pos, float y_pos, float z_pos, float orient){
    static const std::string PLANNING_GROUP = movement_group;

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //START POSITION
    robot_state::RobotState start_pose(*move_group.getCurrentState());

    //GOAL POSITION
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.w = orient;
    goal_pose.position.x = x_pos;
    goal_pose.position.y = y_pos;
    goal_pose.position.z = z_pos;
    move_group.setPoseTarget(goal_pose);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(goal_pose);

    //SET VELOCITY
    move_group.setMaxVelocityScalingFactor(1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    //EXECUTE TRAJECTORY
    move_group.move();
  }



  void grasp(float radian){
    static const std::string PLANNING_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan planGrasp;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //Joint angles for the three main finger joints
    joint_group_positions[0] = radian;
    joint_group_positions[1] = radian;
    joint_group_positions[2] = radian;
    move_group.setJointValueTarget(joint_group_positions);

    bool success = (move_group.plan(planGrasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //EXECUTE TRAJECTORY
    move_group.move();
  }
  void grasptip(float radian){
    static const std::string PLANNING_GROUP = "grippertip";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan planGrasp;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //Joint angles for the three main finger joints
    joint_group_positions[0] = radian;
    joint_group_positions[1] = radian;
    joint_group_positions[2] = radian;
    move_group.setJointValueTarget(joint_group_positions);

    bool success = (move_group.plan(planGrasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //EXECUTE TRAJECTORY
    move_group.move();
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Movements M;

  M.moveHome();
  ros::Duration(7).sleep(); //Give time to complete movement

  //M.cartesianPlan("arm", 0.5, 0.5, 0.5, 1.0); //Move group "arm" to X = 0.5, Y = 0.5, Z = 0.5 with orientation = 1.0
  //ros::Duration(7).sleep(); //Give time to complete movement
  M.grasp(-1.0);
  ros::Duration(3).sleep();
  M.grasp(1.0);
  ros::Duration(3).sleep();
  M.grasptip(0.6);
  ros::Duration(3).sleep(); //Give time to complete movement
  M.grasptip(-0.6);
  ros::Duration(3).sleep(); //Give time to complete movement

  //M.moveHome();

  ros::shutdown();
  return 0;
}
