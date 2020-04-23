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
  void createobj(std::string objects, int id, float x_position, float y_position, float z_position, float x_dimension, float y_dimension, float z_dimension){
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
    if (objects == "Cylinder"){
      primitive.type = primitive.CYLINDER;
    }
    if (objects == "Box"){
      primitive.type = primitive.BOX;
    }
    if (objects == "Sphere"){
      primitive.type = primitive.SPHERE;
    }
    if (objects == "Cone"){
      primitive.type = primitive.CONE;
    }

    primitive.dimensions.resize(3);
    primitive.dimensions[0] = x_dimension;
    primitive.dimensions[1] = y_dimension;
    primitive.dimensions[2] = z_dimension;

    geometry_msgs::Pose figure_pose;
    figure_pose.orientation.w = 1.0;
    figure_pose.position.x = x_position;
    figure_pose.position.y = y_position;
    figure_pose.position.z = z_position;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(figure_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO("Add an object into the world!");

    planning_scene_interface.addCollisionObjects(collision_objects);

    sleep(2.0);
    move_group.setPlanningTime(10.0);
  }
void removeobj(int id){
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
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);
  /* Sleep to give Rviz time to show the object is no longer there. */
  sleep(4.0);
}
void attachobj(int id){
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
void deattachobj(int id){
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
void complete(){
  moveHome();

  jointPlan("arm", 0.4, 0.2, 0.2, 0.0, 1.0, 0.0);
  grasptip(0);
  grasp(0);
  sleep(2.0);
//  M.createobj("Cylinder",1, 0.4, 0.2, 0.2, 0.2, 0.03, 0.1); //Collision
  createobj("Cylinder",1, 0.4, 0.2, 0.1, 0.2, 0.03, 0.1);  //full motion
  //M.createobj("Cylinder",1, 0.4, 0.2, 0.1, 0.1, 0.1, 0.1); //Full motion + collision

  grasptip(0.8);
  grasp(0.6);
  attachobj(1);

  moveHome();
  grasptip(0);
  grasp(0);
  deattachobj(1);

  jointPlan("arm", 0.5, 0.4, 0.3, 0.0, 1.0, 0.0);
  grasptip(0);
  grasp(0);
  sleep(2.0);
  createobj("Cylinder",2, 0.5, 0.4, 0.2, 0.2, 0.03, 0.1);
  grasptip(0.8);
  grasp(0.6);
  attachobj(2);
  jointPlan("arm", 0.4, 0.3, 0.3, 0.0, 1.0, 0.0);
  grasptip(0);
  grasp(0);
  deattachobj(2);
  moveHome();
  /*M.createobj("Cone",3, 0.1, 0.2, 0.4, 0.7, 0.2, 0.4);
  M.createobj("Sphere",4, 0.3, 0.1, 0.2, 0.8, 0.6, 0.5);
  M.createobj("Box",5, 0.3, 0.1, 0.2, 0.1, 0.7, 0.6);*/

  removeobj(1);
  removeobj(2);
}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Movements M;
  M.complete();
//  M.complete(); //Little test


 //Home setting
/*  M.moveHome();
  M.grasptip(0);
  M.grasp(0); */

  //Move to point and grasp from above and flip
  //createobj() orientation is set to default value (1). It can be changed by adjusting the function making orientation different for XYZ separately.
  //createobj() 1- Figure , 2- ID, 3:5 - XYZ, 6:8 - Dimensions

//  M.jointPlan("arm", 0.4, 0.2, 0.2, 0.0, 1.0, 0.0);
  //  M.createobj("Cylinder",1, 0.4, 0.2, 0.2, 0.2, 0.03, 0.1); //Collision
//  M.createobj("Cylinder",1, 0.4, 0.2, 0.1, 0.2, 0.03, 0.1);  //Full motion no collision

  //M.createobj("Cylinder",1, 0.4, 0.2, 0.1, 0.1, 0.1, 0.1); //Full motion with collision
  //M.createobj("Cylinder",2, 0.3, 0.1, 0.2, 0.5, 0.1, 0.3); // Function to spawn second Cylinder

/*
  M.grasptip(0.8); //Grasp tip and grasp ratios needs to be adjusted manually to avoid collision
  M.grasp(0.6);
  M.attachobj(1); // Attaching object with ID 1

  M.moveHome();
  M.grasptip(0);
  M.grasp(0);
  M.deattachobj(1); //Deattaching object with ID 1


  M.removeobj(1); //Removing object with ID 1 from RViz
//  M.removeobj(2);

*/
  //M.jointPlan("arm", 0.4, 0.0, 0.5, 0.0, 0.5, 0.0);
  ros::shutdown();
  return 0;
}
