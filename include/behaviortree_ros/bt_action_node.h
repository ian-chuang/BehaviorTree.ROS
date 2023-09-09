#ifndef BEHAVIOR_TREE_BT_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_BT_ACTION_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

namespace BT
{
  template <class ActionT>
  class RosActionNode : public BT::StatefulActionNode
  {
  protected:
    RosActionNode(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf) : BT::StatefulActionNode(name, conf), node_(nh)
    {
    }

  public:
    using BaseClass = RosActionNode<ActionT>;
    using ActionClientType = actionlib::SimpleActionClient<ActionT>;
    using ActionType = ActionT;
    using GoalType = typename ActionT::_action_goal_type::_goal_type;
    using ResultType = typename ActionT::_action_result_type::_result_type;

    RosActionNode() = delete;

    virtual ~RosActionNode() = default;

    /// These ports will be added automatically if this Node is
    /// registered using RegisterRosAction<DeriveClass>()
    static PortsList providedPorts()
    {
      return {
          InputPort<std::string>("server_name", "name of the Action Server"),
          InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)")};
    }

    /// Method called when the Action makes a transition from IDLE to RUNNING.
    /// If it return false, the entire action is immediately aborted, it returns
    /// FAILURE and no request is sent to the server.
    virtual bool sendGoal(GoalType &goal) = 0;

    /// Method (to be implemented by the user) to receive the reply.
    /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
    virtual NodeStatus onResult(const ResultType &res) = 0;

    enum FailureCause
    {
      MISSING_SERVER = 0,
      ABORTED_BY_SERVER = 1,
      REJECTED_BY_SERVER = 2,
      LOST_BY_SERVER = 3,
      RECALLED_BY_SERVER = 4,
      PREEMPTED_BY_SERVER = 5
    };

    /// Called when a service call failed. Can be overriden by the user.
    virtual NodeStatus onFailedRequest(FailureCause failure)
    {
      return NodeStatus::FAILURE;
    }

  protected:
    std::shared_ptr<ActionClientType> action_client_;
    ros::Duration timeout_;
    ros::NodeHandle &node_;

    NodeStatus onStart() override
    {
      std::string server_name;
      if (!getInput<std::string>("server_name", server_name)) {
        ROS_ERROR("Missing required input [server_name]");
        return NodeStatus::FAILURE;
      }
      action_client_ = std::make_shared<ActionClientType>(node_, server_name, true);

      unsigned msec;
      if (!getInput<unsigned>("timeout", msec)) {
        ROS_ERROR("Missing required input [timeout]");
        return NodeStatus::FAILURE;
      }
      timeout_ = ros::Duration(static_cast<double>(msec) * 1e-3);

      bool connected = action_client_->waitForServer(timeout_);
      if (!connected)
      {
        return onFailedRequest(MISSING_SERVER);
      }

      GoalType goal;
      bool valid_goal = sendGoal(goal);
      if (!valid_goal)
      {
        return NodeStatus::FAILURE;
      }
      action_client_->sendGoal(goal);

      return NodeStatus::RUNNING;
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
      bool connected = action_client_->waitForServer(timeout_);
      if (!connected)
      {
        return onFailedRequest(MISSING_SERVER);
      }

      auto action_state = action_client_->getState();

      // Please refer to these states
      if (action_state == actionlib::SimpleClientGoalState::PENDING ||
          action_state == actionlib::SimpleClientGoalState::ACTIVE)
      {
        return NodeStatus::RUNNING;
      }
      else if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        return onResult(*action_client_->getResult());
      }
      else if (action_state == actionlib::SimpleClientGoalState::ABORTED)
      {
        return onFailedRequest(ABORTED_BY_SERVER);
      }
      else if (action_state == actionlib::SimpleClientGoalState::REJECTED)
      {
        return onFailedRequest(REJECTED_BY_SERVER);
      }
      else if (action_state == actionlib::SimpleClientGoalState::LOST)
      {
        return onFailedRequest(LOST_BY_SERVER);
      }
      else if (action_state == actionlib::SimpleClientGoalState::RECALLED)
      {
        return onFailedRequest(RECALLED_BY_SERVER);
      }
      else if (action_state == actionlib::SimpleClientGoalState::PREEMPTED)
      {
        return onFailedRequest(PREEMPTED_BY_SERVER);
      }
      else
      {
        // this shouldn't happen
        ROS_ERROR("Unknown action state");
        return NodeStatus::FAILURE;
      }
    }

    virtual void onHalted() override
    {
      action_client_->cancelGoal();
    }
  };

  /// Method to register the service into a factory.
  /// It gives you the opportunity to set the ros::NodeHandle.
  template <class DerivedT>
  static void RegisterRosAction(BT::BehaviorTreeFactory &factory,
                                const std::string &registration_ID,
                                ros::NodeHandle &node_handle)
  {
    NodeBuilder builder = [&node_handle](const std::string &name, const NodeConfiguration &config)
    {
      return std::make_unique<DerivedT>(node_handle, name, config);
    };

    TreeNodeManifest manifest;
    manifest.type = getType<DerivedT>();
    manifest.ports = DerivedT::providedPorts();
    manifest.registration_ID = registration_ID;
    const auto &basic_ports = RosActionNode<typename DerivedT::ActionType>::providedPorts();
    manifest.ports.insert(basic_ports.begin(), basic_ports.end());
    factory.registerBuilder(manifest, builder);
  }

} // namespace BT

#endif // BEHAVIOR_TREE_BT_ACTION_NODE_HPP_
