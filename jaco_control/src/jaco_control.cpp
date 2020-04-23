#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const double d2r = 0.01745329251; //Convert from degree to radian

//Bottle position
float b_x = -0.6;
float b_y = -0.3;
float b_z = 0.6;

//Cup position
float c_x = -0.6;
float c_y = 0.3;
float c_z = 0.6;

class JacoControl{
  public:
    void moveSleep(){
      static const std::string PLANNING_GROUP = "arm";

      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      moveit::planning_interface::MoveGroupInterface::Plan planHome;
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      joint_group_positions[0] = 0 * d2r;
      joint_group_positions[1] = 120 * d2r;
      joint_group_positions[2] = 0 * d2r;
      joint_group_positions[3] = 30 * d2r;
      joint_group_positions[4] = 0 * d2r;
      joint_group_positions[5] = 220 * d2r;
      joint_group_positions[6] = 0 * d2r;
      move_group.setJointValueTarget(joint_group_positions);

      bool success = (move_group.plan(planHome) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      move_group.move();
    }

    void moveHome(){
      static const std::string PLANNING_GROUP = "arm";

      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      moveit::planning_interface::MoveGroupInterface::Plan planHome;
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      joint_group_positions[0] = 0 * d2r;
      joint_group_positions[1] = 180 * d2r;
      joint_group_positions[2] = 0 * d2r;
      joint_group_positions[3] = 90 * d2r;
      joint_group_positions[4] = 0 * d2r;
      joint_group_positions[5] = 180 * d2r;
      joint_group_positions[6] = 0 * d2r;
      move_group.setJointValueTarget(joint_group_positions);

      bool success = (move_group.plan(planHome) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      //EXECUTE TRAJECTORY
      move_group.move();
    }

  void jointPlan(std::string movement_group, float x_pos, float y_pos, float z_pos, float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = movement_group;

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //GOAL POSITION
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.x = x_ori;
    goal_pose.orientation.y = y_ori;
    goal_pose.orientation.z = z_ori;
    goal_pose.orientation.w = w_ori;
    goal_pose.position.x = x_pos;
    goal_pose.position.y = y_pos;
    goal_pose.position.z = z_pos;
    move_group.setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan planJoint;

    bool success = (move_group.plan(planJoint) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //EXECUTE TRAJECTORY
    move_group.move();
  }

  void cartesianPlan(std::string movement_group, float x_pos, float y_pos, float z_pos, float x_ori, float y_ori, float z_ori, float w_ori){
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
    goal_pose.orientation.w = w_ori;
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

    moveit::planning_interface::MoveGroupInterface::Plan planCartesian;

    planCartesian.trajectory_= trajectory;

    ros::Duration(5).sleep();

    move_group.execute(planCartesian);
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

  void createObject(std::string type, int id, float x_pos, float y_pos, float z_pos, float x_dim, float y_dim, float z_dim, float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = "gripper";
    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    collision_object.header.frame_id = move_group.getPlanningFrame();

    if (id == 1){
    collision_object.id = "Object1";
    }
    else if (id == 2){
    collision_object.id = "Object2";
    }

  /*  else if (id == 3){
    collision_object.id = "Object4";
    }
    else if (id == 4){
    collision_object.id = "Object5";
    }
    else if (id == 5){
    collision_object.id = "Object6";
  } */

    shape_msgs::SolidPrimitive primitive;
    if (type == "Cylinder"){
      primitive.type = primitive.CYLINDER;
    }
    if (type == "Box"){
      primitive.type = primitive.BOX;
    }
    if (type == "Sphere"){
      primitive.type = primitive.SPHERE;
    }
    if (type == "Cone"){
      primitive.type = primitive.CONE;
    }

    primitive.dimensions.resize(3);
    primitive.dimensions[0] = x_dim;
    primitive.dimensions[1] = y_dim;
    primitive.dimensions[2] = z_dim;

    geometry_msgs::Pose figure_pose;
    figure_pose.orientation.w = w_ori;
    figure_pose.orientation.x = x_ori;
    figure_pose.orientation.y = y_ori;
    figure_pose.orientation.z = z_ori;
    figure_pose.position.x = x_pos;
    figure_pose.position.y = y_pos;
    figure_pose.position.z = z_pos;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(figure_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO("Add an object into the world!");

    planning_scene_interface.addCollisionObjects(collision_objects);

    sleep(0.5);
    //move_group.setPlanningTime(10.0);
  }

  void removeObject(int id){
    static const std::string PLANNING_GROUP = "gripper";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    collision_object.header.frame_id = move_group.getPlanningFrame();
    if (id == 1){
      collision_object.id = "Object1";
    }
    else if (id == 2){
      collision_object.id = "Object2";
    }
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    /* Sleep to give Rviz time to show the object is no longer there. */
    sleep(0.5);
  }

  void attachObject(int id){
    static const std::string PLANNING_GROUP = "gripper";
    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    collision_object.header.frame_id = move_group.getPlanningFrame();
    if (id == 1){
    collision_object.id = "Object1";
    move_group.attachObject(collision_object.id);
    }
    else if (id == 2){
    collision_object.id = "Object2";
    move_group.attachObject(collision_object.id);
    }
    sleep(5.0);
  }

  void detachObject(int id){
    static const std::string PLANNING_GROUP = "gripper";
    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    collision_object.header.frame_id = move_group.getPlanningFrame();
    if (id == 1){
    collision_object.id = "Object1";
    move_group.detachObject(collision_object.id);
    }
    else if (id == 2){
    collision_object.id = "Object2";
    move_group.detachObject(collision_object.id);
    }
    sleep(5.0);
  }

  void gripperConstraints(float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "gripper";
    ocm.header.frame_id = "arm";
    ocm.orientation.x = x_ori;
    ocm.orientation.y = y_ori;
    ocm.orientation.z = z_ori;
    ocm.orientation.w = w_ori;
    ocm.weight = 1.0;

    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(constraints);

    //move_group.setPlanningTime(10.0);
  }

  void clearConstraints(){
    static const std::string PLANNING_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.clearPathConstraints();
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  JacoControl JC;

  // /* COMPLETE SIMULTATION TEST

  //Go from sleep to home
  JC.moveSleep();
  ros::Duration(2).sleep();

  //Spawn Bottle
  JC.createObject("Cylinder", 1, b_x, b_y, b_z, 0.2, 0.035, 0.035, 0, 0, 0, 1);
  ros::Duration(2).sleep();
  //Spawn Cup
  JC.createObject("Cylinder", 2, c_x, c_y, c_z, 0.1, 0.05, 0.05, 0, 0, 0, 1);
  ros::Duration(2).sleep();

  //Move Cartesian to bottle
  JC.cartesianPlan("arm", b_x, b_y, b_z, -0.5, -0.5, 0.5, 0.5);

  //Close gripper and tips
  JC.grasptip(0.5);
  JC.grasp(0.5);

  //Attach bottle to gripper
  JC.attachObject(1);

  //Add constraints to gripper to avoid spills on Cartesian path to cup
  JC.gripperConstraints(-0.5, -0.5, 0.5, 0.5);
  //Lift bottle 30 cm up from table
  JC.cartesianPlan("arm", b_x, b_y, b_z+0.3, -0.5, -0.5, 0.5, 0.5);
  //Move bottle 10 cm above and to the left of cup
  JC.cartesianPlan("arm", c_x, c_y-0.1, c_z+0.1, -0.5, -0.5, 0.5, 0.5);
  //Clear constraints to allow pouring
  JC.clearConstraints();

  //Turn gripper to pour
  JC.cartesianPlan("arm", c_x, c_y-0.1, c_z+0.1, -0.71, 0, 0.71, 0.0);
  ros::Duration(3).sleep();

  //Replace bottle on table
  JC.cartesianPlan("arm", b_x, b_y, b_z+0.3, -0.5, -0.5, 0.5, 0.5);
  JC.cartesianPlan("arm", b_x, b_y, b_z, -0.5, -0.5, 0.5, 0.5);

  //Release grip
  JC.grasp(0.0);
  JC.grasptip(0.0);

  //Detach bottle from gripper
  JC.detachObject(1);

  //Remove bottle and cup from simulation
  JC.removeObject(1);
  JC.removeObject(2);

  // */

  ros::shutdown();
  return 0;
}
