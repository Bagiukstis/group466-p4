#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include "global.h"

quaternians q;
bottle b;
cup c;

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

      joint_group_positions[0] = 0 * q.d2r;
      joint_group_positions[1] = 120 * q.d2r;
      joint_group_positions[2] = 0 * q.d2r;
      joint_group_positions[3] = 30 * q.d2r;
      joint_group_positions[4] = 0 * q.d2r;
      joint_group_positions[5] = 220 * q.d2r;
      joint_group_positions[6] = 0 * q.d2r;
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

      joint_group_positions[0] = 0 * q.d2r;
      joint_group_positions[1] = 180 * q.d2r;
      joint_group_positions[2] = 0 * q.d2r;
      joint_group_positions[3] = 90 * q.d2r;
      joint_group_positions[4] = 0 * q.d2r;
      joint_group_positions[5] = 180 * q.d2r;
      joint_group_positions[6] = 0 * q.d2r;
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

  void cartesianPlan(float x_pos, float y_pos, float z_pos, float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = "arm";

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
    else if (id == 3){
    collision_object.id = "Object3";
    }
    else if (id == 4){
    collision_object.id = "Object4";
    }
    else if (id == 5){
    collision_object.id = "Object5";
    }

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
    else if (id == 3){
      collision_object.id = "Object3";
    }
    else if (id == 4){
      collision_object.id = "Object4";
    }
    else if (id == 5){
      collision_object.id = "Object5";
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
  /*  else if (id == 3){
    collision_object.id = "Object3";
    move_group.attachObject(collision_object.id);
    }
    else if (id == 4){
    collision_object.id = "Object4";
    move_group.attachObject(collision_object.id);
    }
    else if (id == 5){
    collision_object.id = "Object5";
    move_group.attachObject(collision_object.id);
  } */
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

  void orientGraspVertical(float x_pos, float y_pos){
    q.ang = atan2(y_pos,x_pos)-(90*q.d2r);

    q.Y = q.ang;          //YAW = Z
    q.P = 0.0;          //PITCH = Y
    q.R = -1.570796;    //ROLL = X

    q.cy = cos(q.Y * 0.5);
    q.sy = sin(q.Y * 0.5);
    q.cp = cos(q.P * 0.5);
    q.sp = sin(q.P * 0.5);
    q.cr = cos(q.R * 0.5);
    q.sr = sin(q.R * 0.5);

    q.w = q.cr * q.cp * q.cy + q.sr * q.sp * q.sy;
    q.x = q.sr * q.cp * q.cy - q.cr * q.sp * q.sy;
    q.y = q.cr * q.sp * q.cy + q.sr * q.cp * q.sy;
    q.z = q.cr * q.cp * q.sy - q.sr * q.sp * q.cy;
  }

  void orientGraspHorizontal(float x_pos, float y_pos){
    q.ang = atan2(y_pos,x_pos)-(90*q.d2r);

    q.Y = q.ang;          //YAW = Z
    q.P = 1.570796;     //PITCH = Y
    q.R = -1.570796;    //ROLL = X


    q.cy = cos(q.Y * 0.5);
    q.sy = sin(q.Y * 0.5);
    q.cp = cos(q.P * 0.5);
    q.sp = sin(q.P * 0.5);
    q.cr = cos(q.R * 0.5);
    q.sr = sin(q.R * 0.5);

    q.w = q.cr * q.cp * q.cy + q.sr * q.sp * q.sy;
    q.x = q.sr * q.cp * q.cy - q.cr * q.sp * q.sy;
    q.y = q.cr * q.sp * q.cy + q.sr * q.cp * q.sy;
    q.z = q.cr * q.cp * q.sy - q.sr * q.sp * q.cy;
  }

  void pourBottleAt(float x_pos, float y_pos, float z_pos){
    pouring pour;
    q.ang = atan2(y_pos,x_pos)-(90*q.d2r);
    q.hyp = sqrt((abs(x_pos)*abs(x_pos))+(abs(y_pos)*abs(y_pos)));

    q.ang2 = atan2(0.1,q.hyp);
    float full_ang = q.ang + q.ang2 + (90*q.d2r);
    float hyp2 = sqrt((0.1*0.1)+(q.hyp*q.hyp));

    pour.x = hyp2 * cos(full_ang);
    pour.y = hyp2 * sin(full_ang);
    pour.z = z_pos + 0.1;

    orientGraspVertical(x_pos, y_pos);
    cartesianPlan(pour.x, pour.y, pour.z, q.x, q.y, q.z, q.w);

    orientGraspHorizontal(x_pos, y_pos);
    cartesianPlan(pour.x, pour.y, pour.z, q.x, q.y, q.z, q.w);

    ros::Duration(5).sleep();

    orientGraspVertical(x_pos, y_pos);
    cartesianPlan(pour.x, pour.y, pour.z, q.x, q.y, q.z, q.w);
  }

  void linearRetract(float x_pos, float y_pos, float z_pos){
    retract retr;
    q.ang = atan2(y_pos,x_pos);
    q.hyp = (sqrt((abs(x_pos)*abs(x_pos))+(abs(y_pos)*abs(y_pos)))-0.2);

    retr.x = q.hyp * cos(q.ang);
    retr.y = q.hyp * sin(q.ang);
    retr.z = z_pos+0.1;

    cartesianPlan(retr.x, retr.y, retr.z, q.x, q.y, q.z, q.w);
  }

  void closeGripper(){
    grasptip(0.55);
    grasp(0.55);
  }

  void openGripper(){
    grasptip(0);
    grasp(0);
  }

  void pickObject(float x_pos, float y_pos, float z_pos, int obj_id){
    orientGraspVertical(x_pos, y_pos);
    ros::Duration(2).sleep();
    cartesianPlan(x_pos, y_pos, z_pos, q.x, q.y, q.z, q.w);
    ros::Duration(5).sleep();
    closeGripper();
    ros::Duration(5).sleep();
    attachObject(obj_id);
    ros::Duration(5).sleep();
    cartesianPlan(x_pos, y_pos, z_pos+0.15, q.x, q.y, q.z, q.w);
  }

  void placeObject(float x_pos, float y_pos, float z_pos, int obj_id){
    orientGraspVertical(x_pos, y_pos);
    cartesianPlan(x_pos, y_pos, z_pos+0.15, q.x, q.y, q.z, q.w);
    cartesianPlan(x_pos, y_pos, z_pos, q.x, q.y, q.z, q.w);
    openGripper();
    detachObject(obj_id);
    linearRetract(x_pos, y_pos, z_pos);
  }

  void simulateUser(){
    createObject("Sphere", 3, 0.45, -0.25, 0.55, 0.15, 0.1, 0.035, 0, 0, 0, 1);
    createObject("Box", 4, 0.45, -0.25, 0.15, 0.18, 0.4, 0.48, 0, 0, 0, 1);
    createObject("Box", 5, -0.7, 0, 0.25, 0.75, 1.0, 0.05, 0, 0, 0, 1);
  }
  void simulateUserAndTable(){
    createObject("Sphere", 3, 0.45, -0.25, 0.55, 0.15, 0.1, 0.035, 0, 0, 0, 1);
    createObject("Box", 4, 0.45, -0.25, 0.15, 0.18, 0.4, 0.48, 0, 0, 0, 1);
    createObject("Box", 5, -0.7, -0.35, 0.125, 0.75, 1.0, 0.05, 0, 0, 0, 1);
  }

  void pour(){
    moveSleep();
    ros::Duration(2).sleep();

    pickObject(b.x, b.y, b.z, 1);

    pourBottleAt(c.x, c.y, c.z);

    placeObject(b.x, b.y, b.z, 1);

    moveSleep();
  }

  void pick(int obj_id){
    if(obj_id == 1){
      pickObject(b.x, b.y, b.z, obj_id);
    }
    else if(obj_id == 2){
      pickObject(c.x, c.y, c.z, obj_id);
    }
    else{
      ROS_INFO("Unrecognized object");
    }
  }

  void place(float x_pos, float y_pos, float z_pos, int obj_id){
    placeObject(x_pos, y_pos, z_pos, obj_id);
    linearRetract(x_pos, y_pos, z_pos);
    moveSleep();
  }


/*
  void loop2(){
    moveSleep();
    ros::Duration(2).sleep();

    //Spawn Bottle
    createObject("Cylinder", 1, b_x, b_y, b_z, 0.2, 0.035, 0.035, 0, 0, 0, 1);
    ros::Duration(2).sleep();
    //Spawn Cup
    createObject("Cylinder", 2, c_x, c_y, c_z, 0.1, 0.05, 0.05, 0, 0, 0, 1);
    ros::Duration(2).sleep();

    pickObject(b_x, b_y, b_z);

    pourBottleAt(c_x, c_y, c_z);

    placeObject(b_x, b_y, b_z);

    moveSleep();

    //Remove bottle and cup from simulation
  //  removeObject(1);
  //  removeObject(2);
  }
  void loop1(){
    moveSleep();
    ros::Duration(2).sleep();

    //Spawn Bottle
    createObject("Cylinder", 1, b_x, b_y, b_z, 0.2, 0.035, 0.035, 0, 0, 0, 1);
    ros::Duration(2).sleep();
    //Spawn Cup
    createObject("Cylinder", 2, c_x, c_y, c_z, 0.1, 0.05, 0.05, 0, 0, 0, 1);
    ros::Duration(2).sleep();

    //Move Cartesian to bottle
    orientGraspVertical(c_x, c_y);
    cartesianPlan(c_x, c_y, c_z, q_x, q_y, q_z, q_w);

    grasptip(0.6);
    grasp(0.6);
    attachObject(2);

  //  cartesianPlan("arm", c_x, c_y, c_z+0.15, q_x, q_y, q_z, q_w);

    gripperConstraints(q_x, q_y, q_z, q_w);
    moveSleep();
    detachObject(2);

    clearConstraints();
  }
  */
 int menu(){
     ROS_INFO("Choose the task that you want to perform");
     ROS_INFO("Possible options:");
     ROS_INFO("1. Grasp an empty cup");
     ROS_INFO("2. Grasp and Pour from the bottle");
     ROS_INFO("0. Exit");
     ROS_INFO("Input an integer:");

     int choice;
  /*   while(ros::ok){
     std::cin >> choice;
     simulateUser();
     if(choice==1){
       loop1();
       removeObject(1);
       removeObject(2);
       ros::shutdown();
       return 0;
     }
    else if(choice==2){
      loop2();
      ROS_INFO("Choose the task that you want to perform");
      ROS_INFO("Possible options:");
      ROS_INFO("1. Grasp a full cup");
      ROS_INFO("0. Exit");
      td::cin >> choice;
              if(choice==1){
                loop1();
                removeObject(1);
                removeObject(2);
                ros::shutdown();
                return 0;
              }
              else if(choice==0){
                ROS_INFO("Exiting");
                removeObject(1);
                removeObject(2);
                ros::shutdown();
                return 0;
              }
              else{
                ROS_INFO("Input was neither 1 or 0, try again");
              }
     }
     else if(choice==0){
       ROS_INFO("Exiting");
       ros::shutdown();
       return 0;
     }
     else{
       ROS_INFO("Input was neither 1 or 2, try again");
     }
    }
   }
   */
 }
 int menu1(){
   while(ros::ok){
   int choice;
   int option;
   int object;

   simulateUser();
   ros::Duration(2).sleep();
   //Spawn Bottle
   createObject("Cylinder", 1, b.x, b.y, b.z, 0.2, 0.035, 0.035, 0, 0, 0, 1);
   ros::Duration(2).sleep();
   //Spawn Cup
   createObject("Cylinder", 2, c.x, c.y, c.z, 0.1, 0.05, 0.05, 0, 0, 0, 1);
   ros::Duration(2).sleep();
   moveSleep();

   ROS_INFO("Choose task:");
   ROS_INFO("1. Pick");
   ROS_INFO("2. Pour");
   ROS_INFO("0. Exit");
   std::cin >> choice;

   if(choice==1){
     ROS_INFO("Choose object:");
     ROS_INFO("1. Bottle");
     ROS_INFO("2. Cup");
     ROS_INFO("0. Exit");
     std::cin >> object;

     if(object == 1){
       //JC.pick(1);
       orientGraspVertical(b.x, b.y);
       cartesianPlan(b.x, b.y, b.z, q.x, q.y, q.z, q.w);
     }

       else if(object == 2){
         //  JC.pick(2);
         orientGraspVertical(c.x, c.y);
         cartesianPlan(c.x, c.y, c.z, q.x, q.y, q.z, q.w);
       }

       else if(object == 0){
         ROS_INFO("Exiting");
         ros::shutdown();
         return 0;
       }

       else{
         ROS_INFO("Unrecognized object");
       }
     ROS_INFO("Put down?");
     ROS_INFO("Enter 1 to place");
     ROS_INFO("Enter 0 to exit");
     std::cin >> option;
     if(option == 1 && object == 1){
       place(b.x, b.y, b.z, object);

     }
     else if(option == 1 && object == 2){
       place(c.x, c.y, c.z, object);
     }
     else if (option == 0){
       ROS_INFO("Exiting");
       ros::shutdown();
       return 0;
     }
     else{
       ROS_INFO("Not a valid input, try again");
     }
   }

   else if(choice==2){
     pour();
     ros::shutdown();
     return 0;
   }
   else if(choice==0){
     ROS_INFO("Exiting");
     ros::shutdown();
     return 0;
   }
 }
 }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  JacoControl JC;
  JC.menu1();

  ros::shutdown();
  return 0;
}
