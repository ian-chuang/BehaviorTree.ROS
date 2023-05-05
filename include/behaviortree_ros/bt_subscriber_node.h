// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <ros/service_client.h>
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
    getInput("timeout", msec);
    timeout = ros::Duration(static_cast<double>(msec) * 1e-3);

    std::string topic = getInput<std::string>("topic_name").value();
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
    set_subscriber_status(new_status);
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

  NodeStatus get_subscriber_status() {
    mutex.lock();
    NodeStatus current_status = subscriber_status_;
    mutex.unlock();
    return current_status;
  }

  void set_subscriber_status(NodeStatus new_status) {
    mutex.lock();
    subscriber_status_ = new_status;
    mutex.unlock();
  }

  bool wait_for_message(ros::Duration timeout) {
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
      uint32_t num_publishers = subscriber_client_.getNumPublishers();

      if (num_publishers == 0) {
        set_subscriber_status(NodeStatus::IDLE);
      }

      if (num_publishers > 0 && get_subscriber_status() != NodeStatus::IDLE) {
        return true;
      }
      else {
        if (timeout >= ros::Duration(0)) {
          ros::Time current_time = ros::Time::now();
          if ((current_time - start_time) >= timeout) {
            return false;
          }
        }
        ros::Duration(0.02).sleep();
      }
    }
    return false;
  }

  BT::NodeStatus tick() override
  {
    if( ! wait_for_message(timeout) ){
      return onFailedReceive();
    }

    return get_subscriber_status();
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
