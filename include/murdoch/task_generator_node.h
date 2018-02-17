#ifndef _MURDOCH_TASK_GENERATOR_NODE_H_
#define _MURDOCH_TASK_GENERATOR_NODE_H_

#include "utilities/ros_node.h"
#include "utilities/random_generator.h"

namespace murdoch
{
class TaskGeneratorNode : public utilities::ROSNode
{
public:
  typedef boost::shared_ptr<TaskGeneratorNode> Ptr;
  typedef boost::shared_ptr<const TaskGeneratorNode> ConstPtr;
  TaskGeneratorNode(const ros::NodeHandlePtr& nh,
            const ros::Rate& rate = ros::Rate(20));
  virtual ~TaskGeneratorNode() {}
protected:
  virtual void readParameters();
private:
  utilities::RandomGeneratorPtr generator_;
  virtual void controlLoop() { generator_->process(); }
};
typedef TaskGeneratorNode::Ptr TaskGeneratorNodePtr;
typedef TaskGeneratorNode::ConstPtr TaskGeneratorNodeConstPtr;
}

#endif // _MURDOCH_TASK_GENERATOR_NODE_H_
