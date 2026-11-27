#include <chrono>
#include <cmath>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlace {
public:
  PickAndPlace(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());

    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(
        joint_model_group_robot_, initial_joint_positions_robot_);
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    const std::vector<std::string> &joint_names =
        joint_model_group_robot_->getVariableNames();
    for (size_t i = 0; i < joint_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Initial %s: %.4f", joint_names[i].c_str(),
                  initial_joint_positions_robot_[i]);
    }

    add_table();
    add_wall();
    add_tripod();
    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Going to Pre-grasp Position...");
    setup_goal_pose_target(+0.343, +0.132, +0.264, -1.000, +0.000, +0.000,
                           +0.000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();

    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();

    RCLCPP_INFO(LOGGER, "Approaching object...");
    setup_waypoints_target(+0.000, +0.000, -0.060);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();

    RCLCPP_INFO(LOGGER, "Closing Gripper...");
    setup_joint_value_gripper(+0.800);
    plan_trajectory_gripper();
    execute_trajectory_gripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    RCLCPP_INFO(LOGGER, "Retreating...");
    setup_waypoints_target(+0.000, +0.000, +0.060);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();

    RCLCPP_INFO(LOGGER, "Turning to Place Position...");
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    double pan = joint_group_positions_robot_[0];
    double pan_target = (pan < 0.0) ? pan + M_PI : pan - M_PI;
    setup_joint_value_target(
        pan_target, joint_group_positions_robot_[1],
        joint_group_positions_robot_[2], joint_group_positions_robot_[3],
        joint_group_positions_robot_[4], joint_group_positions_robot_[5]);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();


    RCLCPP_INFO(LOGGER, "Releasing Object...");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(LOGGER, "Going to Initial Position...");
    setup_joint_value_target(initial_joint_positions_robot_[0],
                             initial_joint_positions_robot_[1],
                             initial_joint_positions_robot_[2],
                             initial_joint_positions_robot_[3],
                             initial_joint_positions_robot_[4],
                             initial_joint_positions_robot_[5]);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

private:
  void add_table() {
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group_robot_->getPlanningFrame();
    table.id = "table";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {1.0, 0.8, 0.02};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.3;
    box_pose.position.z = -0.02;
    box_pose.orientation.w = 1.0;

    table.primitives.push_back(box);
    table.primitive_poses.push_back(box_pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    bool added = planning_scene_interface_.applyCollisionObject(table);
    RCLCPP_INFO(LOGGER, "Table added: %d", added);
  }

  void add_wall() {
    moveit_msgs::msg::CollisionObject wall;
    wall.header.frame_id = move_group_robot_->getPlanningFrame();
    wall.id = "wall";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {1.5, 0.02, 1.0};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.25;
    box_pose.position.y = 0.61;
    box_pose.position.z = 0.49;
    box_pose.orientation.w = 1.0;

    wall.primitives.push_back(box);
    wall.primitive_poses.push_back(box_pose);
    wall.operation = moveit_msgs::msg::CollisionObject::ADD;

    bool added = planning_scene_interface_.applyCollisionObject(wall);
    RCLCPP_INFO(LOGGER, "Wall added: %d", added);
  }

    void add_tripod() {
    moveit_msgs::msg::CollisionObject tripod;
    tripod.header.frame_id = move_group_robot_->getPlanningFrame();
    tripod.id = "tripod";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.4, 0.4, 0.8};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.70;
    box_pose.position.y = 0.25;
    box_pose.position.z = 0.39;
    box_pose.orientation.w = 1.0;

    tripod.primitives.push_back(box);
    tripod.primitive_poses.push_back(box_pose);
    tripod.operation = moveit_msgs::msg::CollisionObject::ADD;

    bool added = planning_scene_interface_.applyCollisionObject(tripod);
    RCLCPP_INFO(LOGGER, "Tripod added: %d", added);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    joint_group_positions_robot_[0] = angle0;
    joint_group_positions_robot_[1] = angle1;
    joint_group_positions_robot_[2] = angle2;
    joint_group_positions_robot_[3] = angle3;
    joint_group_positions_robot_[4] = angle4;
    joint_group_positions_robot_[5] = angle5;
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void plan_trajectory_kinematics() {
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

  void setup_named_pose_gripper(std::string pose_name) {
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void setup_joint_value_gripper(float angle) {
    move_group_gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint",
                                             angle);
  }

  void plan_trajectory_gripper() {
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    cartesian_waypoints_.push_back(target_pose_robot_);
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
    RCLCPP_INFO(LOGGER, "Cartesian path fraction: %.2f", plan_fraction_robot_);
  }

  void execute_trajectory_cartesian() {
    if (plan_fraction_robot_ >= 0.0) {
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    cartesian_waypoints_.clear();
  }

  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  const JointModelGroup *joint_model_group_robot_;
  RobotStatePtr current_state_robot_;
  std::vector<double> initial_joint_positions_robot_;
  std::vector<double> joint_group_positions_robot_;

  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  std::vector<Pose> cartesian_waypoints_;
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place");
  PickAndPlace pick_and_place_node(base_node);
  pick_and_place_node.execute_trajectory_plan();
  rclcpp::shutdown();
  return 0;
}