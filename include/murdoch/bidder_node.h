#ifndef _MURDOCH_BIDDER_NODE_H_
#define _MURDOCH_BIDDER_NODE_H_

#include <boost/shared_ptr.hpp>
#include "utilities/ros_node.h"

namespace murdoch
{
class BidderNode : public utilities::ROSNode
{
public:
  typedef boost::shared_ptr<BidderNode> Ptr;
  typedef boost::shared_ptr<const BidderNode> ConstPtr;
  BidderNode(const ros::NodeHandlePtr& nh,
             const ros::Rate& rate = ros::Rate(20));
private:
  virtual void controlLoop();
};
typedef BidderNode::Ptr BidderNodePtr;
typedef BidderNode::ConstPtr BidderNodeConstPtr;
}

#endif // _MURDOCH_BIDDER_NODE_H_
