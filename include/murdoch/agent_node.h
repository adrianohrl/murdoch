#ifndef _MURDOCH_AGENT_NODE_H_
#define _MURDOCH_AGENT_NODE_H_

#include <talmech/agent.h>
#include "utilities/ros_node.h"
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
  virtual ~AgentNode() {}
protected:
  talmech::AgentPtr agent_;
  virtual void readParameters();
private:
  ros::Subscriber task_sub_;
  virtual bool isSettedUp() { return agent_.get(); }
  virtual void controlLoop() { agent_->process(); }
};
typedef AgentNode::Ptr AgentNodePtr;
typedef AgentNode::ConstPtr AgentNodeConstPtr;
}

#endif // _MURDOCH_AGENT_NODE_H_
