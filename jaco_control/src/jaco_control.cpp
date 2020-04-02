#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const double d2r = 0.01745329251; //Convert from degree to radian

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
      joint_group_positions[0] = 0 * d2r;
      joint_group_positions[1] = 180 * d2r;
      joint_group_positions[2] = 0 * d2r;
      joint_group_positions[3] = 90 * d2r;
      joint_group_positions[4] = 180 * d2r;
      joint_group_positions[5] = 90 * d2r;
      joint_group_positions[6] = 0 * d2r;
      move_group.setJointValueTarget(joint_group_positions);

      bool success = (move_group.plan(planHome) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      //EXECUTE TRAJECTORY
      move_group.move();
    }


  void jointPlan(std::string movement_group, float x_pos, float y_pos, float z_pos, float x_ori, float y_ori, float z_ori){
    static const std::string PLANNING_GROUP = movement_group;

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //GOAL POSITION
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.x = x_ori;
    goal_pose.orientation.y = y_ori;
    goal_pose.orientation.z = z_ori;
    goal_pose.position.x = x_pos;
    goal_pose.position.y = y_pos;
    goal_pose.position.z = z_pos;
    move_group.setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan planJoint;

    bool success = (move_group.plan(planJoint) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //EXECUTE TRAJECTORY
    move_group.move();
  }


  void cartesianPlan(std::string movement_group, float x_pos, float y_pos, float z_pos, float x_ori, float y_ori, float z_ori){
    static const std::string PLANNING_GROUP = movement_group;

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //START POSITION
    robot_state::RobotState start_pose(*move_group.getCurrentState());

    //GOAL POSITION
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.x = x_ori;
    goal_pose.orientation.y = y_ori;
    goal_pose.orientation.z = z_ori;
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

    moveit::planning_interface::MoveGroupInterface::Plan planGraspTip;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //Joint angles for the three main finger joints
    joint_group_positions[0] = radian;
    joint_group_positions[1] = radian;
    joint_group_positions[2] = radian;
    move_group.setJointValueTarget(joint_group_positions);

    bool success = (move_group.plan(planGraspTip) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //EXECUTE TRAJECTORY
    move_group.move();
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Movements M;

  //Home setting
  M.moveHome();
  M.grasptip(0);
  M.grasp(0);

  //Move to point and grasp from above and flip
  M.jointPlan("arm", 0.4, 0.0, 0.5, 0.0, 1.0, 0.0);
  M.grasptip(0.5);
  M.grasp(0.8);
  M.jointPlan("arm", 0.4, 0.0, 0.5, 0.0, 0.5, 0.0);

  //Home setting
  M.moveHome();
  M.grasptip(0);
  M.grasp(0);

  //Exit
  ros::shutdown();
  return 0;
}
