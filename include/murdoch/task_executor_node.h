#ifndef _MURDOCH_TASK_EXECUTOR_NODE_H_
#define _MURDOCH_TASK_EXECUTOR_NODE_H_

#include <talmech/nodes/ros_node.h>
#include <talmech_msgs/Contract.h>
#include <list>
#include "utilities/noisy_duration.h"
#include "utilities/noisy_double.h"
#include <talmech/auction/auction.h>

namespace murdoch
{
struct Contract
{
public:
  typedef boost::shared_ptr<Contract> Ptr;
  typedef boost::shared_ptr<const Contract> ConstPtr;
  Contract(const talmech_msgs::Contract& msg, const ros::Duration& duration,
           bool failure)
      : timestamp_(ros::Time::now()), msg_(msg), duration_(duration),
        failure_(failure)
  {
  }
  virtual ~Contract() {}
  void process()
  {
    msg_.timestamp = ros::Time::now();
    if (timestamp_ + duration_ < msg_.timestamp)
    {
      msg_.status = failure_ ? talmech::auction::status::Aborted
                             : talmech::auction::status::Concluded;
    }
  }
  bool isOngoing() const
  {
    return msg_.status == talmech::auction::status::Ongoing;
  }
  bool hasAborted() const
  {
    return msg_.status == talmech::auction::status::Aborted;
  }
  bool hasConcluded() const
  {
    return msg_.status == talmech::auction::status::Concluded;
  }
  talmech_msgs::Contract toMsg() { return msg_; }
  bool operator==(const talmech_msgs::Contract& msg) const
  {
    return msg_.task.id == msg.task.id;
  }
  bool operator!=(const talmech_msgs::Contract& msg) const
  {
    return !(*this == msg);
  }
  std::string str() const { return msg_.task.id; }
  const char* c_str() const { return str().c_str(); }
  friend std::ostream& operator<<(std::ostream& out, const Contract& contract)
  {
    out << contract.str();
    return out;
  }
private:
  ros::Time timestamp_;
  talmech_msgs::Contract msg_;
  ros::Duration duration_;
  bool failure_;
};
typedef Contract::Ptr ContractPtr;
typedef Contract::ConstPtr ContractConstPtr;
typedef std::list<ContractPtr> Contracts;
typedef Contracts::iterator ContractsIt;
typedef Contracts::const_iterator ContractsConstIt;
class TaskExecutorNode : public talmech::nodes::ROSNode
{
public:
  typedef boost::shared_ptr<TaskExecutorNode> Ptr;
  typedef boost::shared_ptr<const TaskExecutorNode> ConstPtr;
  TaskExecutorNode(const ros::NodeHandlePtr& nh,
                   const ros::Rate& rate = ros::Rate(20));
  virtual ~TaskExecutorNode();

private:
  ros::Subscriber execute_sub_;
  ros::Subscriber cancel_sub_;
  ros::Publisher feedback_pub_;
  ros::Publisher result_pub_;
  Contracts contracts_;
  utilities::NoisyDurationPtr duration_;
  double failure_probability_;
  utilities::NoisyDoublePtr failure_;
  virtual void readParameters();
  virtual void controlLoop();
  void executeCallback(const talmech_msgs::Contract& msg);
  void cancelCallback(const talmech_msgs::Contract& msg);
};
typedef TaskExecutorNode::Ptr TaskExecutorNodePtr;
typedef TaskExecutorNode::ConstPtr TaskExecutorNodeConstPtr;
}

#endif // _MURDOCH_TASK_EXECUTOR_NODE_H_
