// The orchestrating node. It loads the configuration and URDF, builds the
// kinematics / collision / controller layers and exposes:
//   * action  ~/move              : execute one MotionStep
//   * action  ~/execute_sequence  : execute a list / named sequence of steps
//   * service ~/manage_collision_object : add/remove/attach world objects
#ifndef BIMANUAL_MANIPULATION_MANIPULATION_SERVER_HPP
#define BIMANUAL_MANIPULATION_MANIPULATION_SERVER_HPP

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>

#include <bimanual_msgs/action/move.hpp>
#include <bimanual_msgs/action/execute_sequence.hpp>
#include <bimanual_msgs/msg/motion_step.hpp>
#include <bimanual_msgs/srv/manage_collision_object.hpp>

#include "bimanual_manipulation/collision_model.hpp"
#include "bimanual_manipulation/config_loader.hpp"
#include "bimanual_manipulation/kinematics.hpp"
#include "bimanual_manipulation/move_group.hpp"

namespace bimanual_manipulation
{

class ManipulationServer : public rclcpp::Node
{
public:
  explicit ManipulationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Heavy initialization (URDF, kinematics, collision). Returns false on error.
  bool initialize();

private:
  using Move = bimanual_msgs::action::Move;
  using ExecuteSequence = bimanual_msgs::action::ExecuteSequence;
  using MotionStep = bimanual_msgs::msg::MotionStep;
  using ManageCollisionObject = bimanual_msgs::srv::ManageCollisionObject;

  // --- setup ---------------------------------------------------------------
  bool loadUrdf(std::string & urdf_xml);
  bool buildModel(const std::string & urdf_xml, std::string & error);

  // --- core execution ------------------------------------------------------
  bool executeStep(const MotionStep & step, std::string & error);
  bool executeNamedOrJoint(
    const GroupConfig & g, const std::vector<double> & target, double vscale,
    double ascale, std::string & error);
  bool executeCartesian(const GroupConfig & g, const MotionStep & step, std::string & error);

  std::map<std::string, double> currentState() const;
  bool currentGroupValues(const GroupConfig & g, std::vector<double> & q, std::string & error) const;
  void makeLimits(const GroupConfig & g, Limits & limits) const;

  // --- ROS callbacks -------------------------------------------------------
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp_action::GoalResponse moveGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Move::Goal>);
  rclcpp_action::CancelResponse moveCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>>);
  void moveAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>>);

  rclcpp_action::GoalResponse seqGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteSequence::Goal>);
  rclcpp_action::CancelResponse seqCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>>);
  void seqAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>>);

  void manageCollisionObject(
    const std::shared_ptr<ManageCollisionObject::Request> req,
    std::shared_ptr<ManageCollisionObject::Response> res);

  // --- state ---------------------------------------------------------------
  urdf::Model urdf_model_;
  KDL::Tree kdl_tree_;
  ManipulationConfig config_;
  CollisionModel collision_;

  std::map<std::string, std::shared_ptr<GroupKinematics>> kinematics_;
  std::map<std::string, std::shared_ptr<MoveGroup>> move_groups_;
  std::map<std::string, std::pair<double, double>> joint_limits_;  // name -> (vmax, amax)

  mutable std::mutex state_mutex_;
  std::map<std::string, double> joint_state_;

  std::atomic<bool> busy_{false};
  std::atomic<bool> cancel_requested_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  std::string urdf_from_topic_;

  rclcpp_action::Server<Move>::SharedPtr move_server_;
  rclcpp_action::Server<ExecuteSequence>::SharedPtr seq_server_;
  rclcpp::Service<ManageCollisionObject>::SharedPtr collision_service_;
};

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_MANIPULATION_SERVER_HPP
