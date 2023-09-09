#ifndef BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>
#include <mutex>

namespace BT
{

/**
 * Base Action to implement a ROS Service
 */
template<class MessageT>
class RosSubscriberNode : public BT::ConditionNode
{
protected:

  RosSubscriberNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
   BT::ConditionNode(name, conf), node_(nh) { 

    subscriber_status_ = NodeStatus::IDLE;

    unsigned msec;
    if (!getInput<unsigned>("timeout", msec)) {
      throw BT::RuntimeError("missing required input [timeout]");
    }
    timeout = ros::Duration(static_cast<double>(msec) * 1e-3);

    std::string topic;
    if (!getInput<std::string>("topic_name", topic)) {
      throw BT::RuntimeError("missing required input [topic_name]");
    }

    subscriber_client_ = node_.subscribe(topic, 1, &RosSubscriberNode<MessageT>::callback, this);
  }

public:

  using BaseClass    = RosSubscriberNode<MessageT>;
  using MessageType  = MessageT;
  using MessagePtrType  = typename MessageT::ConstPtr;
//   using ResponseType = typename MessageT::Response;

  RosSubscriberNode() = delete;

  virtual ~RosSubscriberNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("topic_name", "name of the ROS topic"),
      InputPort<unsigned>("timeout", 100, "timeout to connect to server (milliseconds)")
    };
  }

  void callback(const MessageT& msg)
  {
    NodeStatus new_status = onReceive(msg);
    setSubscriberStatus(new_status);
  }

  /// Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus onFailedReceive()
  {
    return NodeStatus::FAILURE;
  }

  /// User must implement this method.
  virtual NodeStatus onReceive(const MessageT& msg) = 0;

protected:

  ros::Subscriber subscriber_client_;
  std::mutex mutex;
  NodeStatus subscriber_status_;
  ros::Duration timeout;

  typename MessageT::ConstPtr message_;

  // The node that will be used for any ROS operations
  ros::NodeHandle& node_;

  NodeStatus getSubscriberStatus() {
    mutex.lock();
    NodeStatus current_status = subscriber_status_;
    mutex.unlock();
    return current_status;
  }

  void setSubscriberStatus(NodeStatus new_status) {
    mutex.lock();
    subscriber_status_ = new_status;
    mutex.unlock();
  }

  bool waitForMessage(ros::Duration timeout) {
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
      uint32_t num_publishers = subscriber_client_.getNumPublishers();

      if (num_publishers == 0) {
        setSubscriberStatus(NodeStatus::IDLE);
      }

      if (num_publishers > 0 && getSubscriberStatus() != NodeStatus::IDLE) {
        return true;
      }
      else {
        if (timeout >= ros::Duration(0)) {
          ros::Time current_time = ros::Time::now();
          if ((current_time - start_time) >= timeout) {
            return false;
          }
        }
        // ros::Duration(0.02).sleep();
      }
    }
    return false;
  }

  BT::NodeStatus tick() override
  {
    if( ! waitForMessage(timeout) ){
      return onFailedReceive();
    }

    return getSubscriberStatus();
  }
};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosSubscriber(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosSubscriberNode< typename DerivedT::MessageType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_
