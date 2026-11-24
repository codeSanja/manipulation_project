#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

class PickAndPlace {
public:
  PickAndPlace(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    move_group_node_ = rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    move_group_robot_ = std::make_shared<MoveGroupInterface>(move_group_node_, PLANNING_GROUP_ROBOT);

    RCLCPP_INFO(LOGGER, "Planning Frame: %s", move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s", move_group_robot_->getEndEffectorLink().c_str());

    joint_model_group_robot_ = move_group_robot_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ROBOT);
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_, initial_joint_positions_robot_);
    move_group_robot_->setStartStateToCurrentState();

    const std::vector<std::string> &joint_names = joint_model_group_robot_->getVariableNames();
    for (size_t i = 0; i < joint_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Initial %s: %.4f", joint_names[i].c_str(), initial_joint_positions_robot_[i]);
    }

    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Going to Pre-grasp Position...");
    setup_goal_pose_target(+0.340, -0.020, +0.264, -1.000, +0.000, +0.000, +0.000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  const JointModelGroup *joint_model_group_robot_;
  RobotStatePtr current_state_robot_;
  std::vector<double> initial_joint_positions_robot_;

    Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;

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

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node = std::make_shared<rclcpp::Node>("pick_and_place");
  PickAndPlace pick_and_place_node(base_node);
  pick_and_place_node.execute_trajectory_plan();
  rclcpp::shutdown();
  return 0;
}