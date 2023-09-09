#ifndef BEHAVIOR_TREE_ASYNC_BEHAVIOR_HPP_
#define BEHAVIOR_TREE_ASYNC_BEHAVIOR_HPP_

#include <behaviortree_cpp/action_node.h>
#include <future>

namespace BT
{
  // Base class for asynchronous nodes
  class AsyncBehaviorBase : public BT::StatefulActionNode
  {
  public:
    AsyncBehaviorBase(const std::string& name, const NodeConfig& config) : 
      BT::StatefulActionNode(name, config) {}

    // Define the asynchronous task in derived classes
    virtual BT::NodeStatus doWork() = 0;

    NodeStatus onStart() override
    {
      // Start the asynchronous task
      task_future_ = std::async(std::launch::async, &AsyncBehaviorBase::doWork, this);

      // Return RUNNING to indicate that the task is running asynchronously
      return BT::NodeStatus::RUNNING;
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
      // Check if the task has finished
      if (task_future_.valid())
      {
        // Check if the task has finished
        if (task_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
          // The task has finished
          NodeStatus result = task_future_.get();
          return result;
        }
      }

      // The task is still running
      return BT::NodeStatus::RUNNING;
    }

    // Override the halt function to stop the asynchronous task
    void onHalted() override
    {
      // Wait for the task to finish if it's still running
      if (task_future_.valid() && task_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
      {
        task_future_.wait();
      }
    }

  private:
    std::future<BT::NodeStatus> task_future_;
  };
} // namespace BT

#endif // BEHAVIOR_TREE_ASYNC_BEHAVIOR_HPP_
