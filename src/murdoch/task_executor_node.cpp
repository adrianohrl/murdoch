#include "murdoch/task_executor_node.h"

namespace murdoch
{
TaskExecutorNode::TaskExecutorNode(const ros::NodeHandlePtr& nh,
                                   const ros::Rate& rate)
    : ROSNode::ROSNode(nh, rate), failure_(new utilities::NoisyDouble(0.0, 1.0))
{
  execute_sub_ = nh_->subscribe("contract/execute", 10,
                                &TaskExecutorNode::executeCallback, this);
  cancel_sub_ = nh_->subscribe("contract/cancel", 10,
                               &TaskExecutorNode::cancelCallback, this);
  feedback_pub_ =
      nh_->advertise<talmech_msgs::Contract>("contract/feedback", 10);
  result_pub_ = nh_->advertise<talmech_msgs::Contract>("contract/result", 10);
}

TaskExecutorNode::~TaskExecutorNode()
{
  execute_sub_.shutdown();
  cancel_sub_.shutdown();
  feedback_pub_.shutdown();
  result_pub_.shutdown();
}

void TaskExecutorNode::readParameters()
{
  ros::NodeHandle pnh("~");
  pnh.param("failure_probability", failure_probability_, 0.1);
  double duration_mean, duration_std;
  pnh.param("duration/mean", duration_mean, 2.0);
  pnh.param("duration/standard_deviation", duration_std, 1.0);
  duration_.reset(new utilities::NoisyDuration(ros::Duration(duration_mean),
                                               ros::Duration(duration_std)));
}

void TaskExecutorNode::controlLoop()
{
  ContractsIt it(contracts_.begin());
  while (it != contracts_.end())
  {
    ContractPtr contract(*it);
    contract->process();
    if (!contract->isOngoing())
    {
      ROS_INFO_STREAM((contract->hasConcluded() ? "Concluding " : "Aborting ")
                      << *contract << " task execution...");
      result_pub_.publish(contract->toMsg());
      it = contracts_.erase(it);
      continue;
    }
    feedback_pub_.publish(contract->toMsg());
    it++;
  }
}

void TaskExecutorNode::executeCallback(const talmech_msgs::Contract& msg)
{
  bool failure(failure_->probability(failure_->random()) <=
               failure_probability_);
  ContractPtr contract(new Contract(msg, duration_->random(), failure));
  ROS_INFO_STREAM("Starting " << *contract << " task execution...");
  contracts_.push_back(contract);
}

void TaskExecutorNode::cancelCallback(const talmech_msgs::Contract& msg)
{
  for (ContractsIt it(contracts_.begin()); it != contracts_.end(); it++)
  {
    if (**it == msg)
    {
      ROS_INFO_STREAM("Canceling " << **it << " task execution...");
      contracts_.erase(it);
      break;
    }
  }
}
}
