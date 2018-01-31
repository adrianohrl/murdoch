#ifndef _MURDOCH_AGENT_NODE_H_
#define _MURDOCH_AGENT_NODE_H_

#include <talmech/agent.h>
#include "utilities/ros_node.h"
#include <ros/subscriber.h>
#include <talmech_msgs/Task.h>

namespace murdoch
{
class AgentNode : public utilities::ROSNode
{
public:
  typedef boost::shared_ptr<AgentNode> Ptr;
  typedef boost::shared_ptr<const AgentNode> ConstPtr;
  AgentNode(const ros::NodeHandlePtr& nh,
            const ros::Rate& rate = ros::Rate(20));
  virtual ~AgentNode();
protected:
  talmech::AgentPtr agent_;
  virtual void readParameters();
private:
  ros::Subscriber task_sub_;
  virtual bool isSettedUp() { return agent_; }
  virtual void controlLoop() { agent_->process(); }
  void taskCallback(const talmech_msgs::Task& msg);
};
typedef AgentNode::Ptr AgentNodePtr;
typedef AgentNode::ConstPtr AgentNodeConstPtr;
}

#endif // _MURDOCH_AGENT_NODE_H_
